/*
 * floppy_generic.c
 * 
 * Generic floppy drive low-level support routines.
 * Mainly dealing with IRQs, timers and DMA.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* A DMA buffer for running a timer associated with a floppy-data I/O pin. */
struct dma_ring {
    /* Current state of DMA (RDATA): 
     *  DMA_inactive: No activity, buffer is empty. 
     *  DMA_starting: Buffer is filling, DMA+timer not yet active.
     *  DMA_active: DMA is active, timer is operational. 
     *  DMA_stopping: DMA+timer halted, buffer waiting to be cleared. 
     * Current state of DMA (WDATA): 
     *  DMA_inactive: No activity, flux ring and bitcell buffer are empty. 
     *  DMA_starting: Flux ring and bitcell buffer are filling.
     *  DMA_active: Writeback processing is active (to mass storage).
     *  DMA_stopping: Timer halted, buffers waiting to be cleared. */
#define DMA_inactive 0 /* -> {starting, active} */
#define DMA_starting 1 /* -> {active, stopping} */
#define DMA_active   2 /* -> {stopping} */
#define DMA_stopping 3 /* -> {inactive} */
    volatile uint8_t state;
    /* IRQ handler sets this if the read buffer runs dry. */
    volatile uint8_t kick_dma_irq;
    /* Indexes into the buf[] ring buffer. */
    uint16_t cons;
    union {
        uint16_t prod; /* dma_rd: our producer index for flux samples */
        uint16_t prev_sample_unused; /* dma_wr: previous CCRx sample value */
    };
    /* DMA ring buffer of timer values (ARR or CCRx). */
    uint16_t buf[2*1024];
};

/* DMA buffers are permanently allocated while a disk image is loaded, allowing 
 * independent and concurrent management of the RDATA/WDATA pins. */
static struct dma_ring *dma_rd; /* RDATA DMA buffer */
static struct dma_ring *dma_wr; /* WDATA DMA buffer */

/* Statically-allocated floppy drive state. Tracks head movements and 
 * side changes at all times, even when the drive is empty. */
static struct drive {
    uint8_t cyl, head;
    bool_t writing;
    bool_t sel;
    bool_t index_suppressed; /* disable IDX while writing to USB stick */
    uint8_t outp;
    volatile bool_t inserted;
    struct timer chgrst_timer;
    struct {
        struct timer timer;
        bool_t on;
        bool_t changed;
    } motor;
    struct {
#define STEP_started  1 /* started by hi-pri IRQ */
#define STEP_latched  2 /* latched by lo-pri IRQ */
#define STEP_active   (STEP_started | STEP_latched)
#define STEP_settling 4 /* handled by step.timer */
        uint8_t state;
        bool_t inward;
        time_t start;
        struct timer timer;
    } step;
    uint32_t restart_pos;
    struct image *image;
    struct thread io_thread;
} drive;

static struct image *image;

static struct {
    struct timer timer, timer_deassert;
    struct timer custom_timer;
    time_t prev_time;
    bool_t fake_fired;
    uint8_t custom_pulses_ver;
    uint8_t custom_pulses_len;
    /* Durations relative to track start. Must be in increasing order. */
    time_t custom_pulses[MAX_CUSTOM_PULSES];
} index;

static unsigned int drive_calc_track(struct drive *drv);
static void rdata_stop(void);
static void wdata_start(void);
static void wdata_stop(void);

struct exti_irq {
    uint8_t irq, pri;
    uint16_t pr_mask; /* != 0: irq- and exti-pending flags are cleared */
};

#if defined(QUICKDISK)
#include "gotek/quickdisk.c"
#else
#include "gotek/floppy.c"
#endif

/* Initialise IRQs according to statically-defined exti_irqs[]. */
static void floppy_init_irqs(void)
{
    const struct exti_irq *e;
    unsigned int i;

    /* Configure physical interface interrupts. */
    for (i = 0, e = exti_irqs; i < ARRAY_SIZE(exti_irqs); i++, e++) {
        IRQx_set_prio(e->irq, e->pri);
        if (e->pr_mask != 0) {
            /* Do not trigger an initial interrupt on this line. Clear EXTI_PR
             * before IRQ-pending, otherwise IRQ-pending is immediately
             * reasserted. */
            exti->pr = e->pr_mask;
            IRQx_clear_pending(e->irq);
        } else {
            /* Common case: we deliberately trigger the first interrupt to 
             * prime the ISR's state. */
            IRQx_set_pending(e->irq);
        }
    }

    /* Enable physical interface interrupts. */
    for (i = 0, e = exti_irqs; i < ARRAY_SIZE(exti_irqs); i++, e++) {
        IRQx_enable(e->irq);
    }
}

/* Allocate and initialise a DMA ring. */
static struct dma_ring *dma_ring_alloc(void)
{
    struct dma_ring *dma = arena_alloc(sizeof(*dma));
    memset(dma, 0, offsetof(struct dma_ring, buf));
    return dma;
}

/* Allocate floppy resources and mount the given image. 
 * On return: dma_rd, dma_wr, image and index are all valid. */
static void floppy_mount(struct slot *slot)
{
    struct image *im;
    struct dma_ring *_dma_rd, *_dma_wr;
    struct drive *drv = &drive;
    FSIZE_t fastseek_sz;
    DWORD *cltbl;
    FRESULT fr;
    bool_t async = TRUE, retry;

    do {
        retry = FALSE;

        arena_init();

        _dma_rd = dma_ring_alloc();
        _dma_wr = dma_ring_alloc();

        im = arena_alloc(sizeof(*im));
        memset(im, 0, sizeof(*im));

        /* Create a fast-seek cluster table for the image. */
#define MAX_FILE_FRAGS 511 /* up to a 4kB cluster table */
        cltbl = arena_alloc(0);
        *cltbl = (MAX_FILE_FRAGS + 1) * 2;
        fatfs_from_slot(&im->fp, slot, FA_READ);
        fastseek_sz = f_size(&im->fp);
        if (fastseek_sz == 0) {
            /* Empty or dummy file. */
            cltbl = NULL;
        } else {
            im->fp.cltbl = cltbl;
            fr = f_lseek(&im->fp, CREATE_LINKMAP);
            printk("Fast Seek: %u frags\n", (*cltbl / 2) - 1);
            if (fr == FR_OK) {
                DWORD *_cltbl = arena_alloc(*cltbl * 4);
                ASSERT(_cltbl == cltbl);
            } else if (fr == FR_NOT_ENOUGH_CORE) {
                printk("Fast Seek: FAILED\n");
                cltbl = NULL;
            } else {
                F_die(fr);
            }
        }

        /* ~0 avoids sync match within fewer than 32 bits of scan start. */
        im->write_bc_window = ~0;

        if (!async) {
            /* Large buffer to absorb write latencies at mass-storage layer. */
            int ring_kb = (ram_kb >= 64) ? 32 : 8;
            im->bufs.write_bc.len = ring_kb * 1024; /* power of two */
            im->bufs.write_bc.p = arena_alloc(im->bufs.write_bc.len);
        } else {
            /* Size at least 4kb so a 512 byte sector (1k bc) can be fully
             * encoded into the 2kb read_bc, with space to spare. */
            im->bufs.write_bc.len = 4*1024; /* Power of two. */
            im->bufs.write_bc.p = arena_alloc(im->bufs.write_bc.len);
        }

        /* Read BC buffer overlaps the second half of the write BC buffer. This 
         * is because:
         *  (a) The read BC buffer does not need to absorb such large latencies
         *      (reads are much more predictable than writes to mass storage).
         *  (b) By dedicating the first half of the write buffer to writes, we
         *      can safely start processing write flux while read-data is still
         *      processing (eg. in-flight mass storage io). At say 10kB of
         *      dedicated write buffer, this is good for >80ms before colliding
         *      with read buffers, even at HD data rate (1us/bitcell).
         *      This is more than enough time for read
         *      processing to complete. */
        im->bufs.read_bc.len = im->bufs.write_bc.len / 2;
        im->bufs.read_bc.p = (char *)im->bufs.write_bc.p
            + im->bufs.read_bc.len;

        /* Any remaining space is used for staging I/O to mass storage, shared
         * between read and write paths (Change of use of this memory space is
         * fully serialised). */
        im->bufs.write_data.len = arena_avail();
        im->bufs.write_data.p = arena_alloc(im->bufs.write_data.len);
        im->bufs.read_data = im->bufs.write_data;

        /* Minimum allowable buffer space. */
        ASSERT(im->bufs.read_data.len >= 10*1024);

        /* Mount the image file. */
#if defined(QUICKDISK)
        image_open(im, slot, cltbl, FALSE);
#else
        {
            uint8_t cyl = drive_calc_track(drv)>>1;
            /* Skip useless loading if D-A guaranteed. */
            if (cyl != 255)
                image_open(im, slot, cltbl, FALSE);
            /* Now that the number of image cylinders is known, do a precise D-A
             * check. */
            if (in_da_mode(im, cyl)) {
                volume_cache_destroy(); /* Clean up after other image format. */
                image_open(im, slot, cltbl, TRUE);
            }
        }
#endif
        if (async != im->disk_handler->async) {
            async = im->disk_handler->async;
            retry = TRUE;
            continue;
        }
        if (!im->disk_handler->write_track || volume_readonly())
            slot->attributes |= AM_RDO;
        if (slot->attributes & AM_RDO) {
            printk("Image is R/O\n");
        } else {
            image_extend(im);
        }

    } while (f_size(&im->fp) != fastseek_sz || retry);

    /* After image is extended at mount time, we permit no further changes 
     * to the file metadata. Clear the dirent info to ensure this. */
    im->fp.dir_ptr = NULL;
    im->fp.dir_sect = 0;

    _dma_rd->state = DMA_stopping;

    /* Make allocated state globally visible now. */
    drv->image = image = im;
    barrier(); /* image ptr /then/ dma rings */
    dma_rd = _dma_rd;
    dma_wr = _dma_wr;

    drv->index_suppressed = FALSE;
    index.prev_time = time_now();
    index.custom_pulses_len = 0;
    index.custom_pulses_ver = 0xFF;
}

/* Initialise timers and DMA for RDATA/WDATA. */
static void timer_dma_init(void)
{
    bool_t st506 = TRUE;
    /* Enable DMA interrupts. */
    dma1->ifcr = DMA_IFCR_CGIF(dma_rdata_ch) | DMA_IFCR_CGIF(dma_wdata_ch);
    IRQx_set_prio(dma_rdata_irq, RDATA_IRQ_PRI);
    IRQx_set_prio(dma_wdata_irq, WDATA_IRQ_PRI);
    IRQx_enable(dma_rdata_irq);
    IRQx_enable(dma_wdata_irq);

    /* RDATA Timer setup:
     * The counter is incremented at full SYSCLK rate. 
     *  
     * Ch.2 (RDATA) is in PWM mode 1. It outputs O_TRUE for 400ns and then 
     * O_FALSE until the counter reloads. By changing the ARR via DMA we alter
     * the time between (fixed-width) O_TRUE pulses, mimicking floppy drive 
     * timings. */
    /* ITR3=TIM4 for TIM3 */
    tim_rdata_flux->smcr = TIM_SMCR_TS_ITR3 | TIM_SMCR_SMS_TRIGGER;
    tim_rdata_flux->psc = 0;
    if (!st506) {
        tim_rdata_flux->ccmr1 = (TIM_CCMR1_CC2S(TIM_CCS_OUTPUT) |
                            TIM_CCMR1_OC2M(TIM_OCM_PWM1));
        tim_rdata_flux->ccer = TIM_CCER_CC2E | ((O_TRUE==0) ? TIM_CCER_CC2P : 0);
        tim_rdata_flux->arr = sysclk_ns(400)-1;
        tim_rdata_flux->ccr2 = 1;
    } else {
        tim_rdata_flux->ccmr1 = (TIM_CCMR1_CC1S(TIM_CCS_OUTPUT) |
                            TIM_CCMR1_OC1M(TIM_OCM_PWM2));
        tim_rdata_flux->ccer = TIM_CCER_CC1E | ((O_TRUE==1) ? TIM_CCER_CC1P : 0);
        tim_rdata_flux->arr = sysclk_ns(100)-1;
        tim_rdata_flux->ccr1 = 1;
    }
    tim_rdata_flux->cr1 = TIM_CR1_OPM;

    tim_rdata->cr2 = TIM_CR2_MMS_OC1REF | TIM_CR2_CCDS;
    tim_rdata->dier = TIM_DIER_CC1DE;
    tim_rdata->ccmr1 = TIM_CCMR1_CC1S(TIM_CCS_OUTPUT) |
                       TIM_CCMR1_OC1M(TIM_OCM_PWM2) |
                       TIM_CCMR1_OC1PE;
    tim_rdata->psc = 0;

    /* DMA setup: From a circular buffer into the RDATA Timer's CCR1. */
    dma_rdata.cpar = (uint32_t)(unsigned long)&tim_rdata->ccr1;
    dma_rdata.cmar = (uint32_t)(unsigned long)dma_rd->buf;
    dma_rdata.cndtr = ARRAY_SIZE(dma_rd->buf);
    dma_rdata.ccr = (DMA_CCR_PL_HIGH |
                     DMA_CCR_MSIZE_16BIT |
                     DMA_CCR_PSIZE_16BIT |
                     DMA_CCR_MINC |
                     DMA_CCR_CIRC |
                     DMA_CCR_DIR_M2P |
                     DMA_CCR_HTIE |
                     DMA_CCR_TCIE |
                     DMA_CCR_EN);

    /* WDATA Timer setup: 
     * The counter runs from 0x0000-0xFFFF inclusive at full SYSCLK rate.
     *  
     * Ch.1 (WDATA) is in Input Capture mode, sampling on every clock and with
     * no input prescaling or filtering. Samples are captured on the falling 
     * edge of the input (CCxP=1). DMA is used to copy the sample into a ring
     * buffer for batch processing in the DMA-completion ISR. */
    tim_wdata->cr2 = 0;
    tim_wdata->smcr = TIM_SMCR_TS_TI1FP1 | TIM_SMCR_SMS_RESET;
    tim_wdata->dier = TIM_DIER_CC1DE;
    tim_wdata->ccmr1 = TIM_CCMR1_CC1S(TIM_CCS_INPUT_TI1);
    tim_wdata->psc = 0;
    tim_wdata->arr = 0xffff;

    /* DMA setup: From the WDATA Timer's CCRx into a circular buffer. */
    dma_wdata.cpar = (uint32_t)(unsigned long)&tim_wdata->ccr1;
    dma_wdata.cmar = (uint32_t)(unsigned long)dma_wr->buf;
    dma_wdata.cndtr = ARRAY_SIZE(dma_wr->buf);
    dma_wdata.ccr = (DMA_CCR_PL_HIGH |
                     DMA_CCR_MSIZE_16BIT |
                     DMA_CCR_PSIZE_16BIT |
                     DMA_CCR_MINC |
                     DMA_CCR_CIRC |
                     DMA_CCR_DIR_P2M |
                     DMA_CCR_HTIE |
                     DMA_CCR_TCIE |
                     DMA_CCR_EN);
}

static unsigned int drive_calc_track(struct drive *drv)
{
    return drv->cyl*2 + (drv->head & (drv->image->nr_sides - 1));
}

/* Find current rotational position for read-stream restart. */
static void drive_set_restart_pos(struct drive *drv)
{
    uint32_t pos = max_t(int32_t, 0, time_diff(index.prev_time, time_now()));
    pos %= drv->image->stk_per_rev;
    drv->restart_pos = pos;
    drv->index_suppressed = TRUE;
}

/* Called from IRQ context to stop the write stream. */
static void wdata_stop(void)
{
    struct write *write;
    struct drive *drv = &drive;
    uint8_t prev_state = dma_wr->state;

    /* Already inactive? Nothing to do. */
    if ((prev_state == DMA_inactive) || (prev_state == DMA_stopping))
        return;

    /* Ok we're now stopping DMA activity. */
    dma_wr->state = DMA_stopping;

    /* Turn off timer. */
    tim_wdata->ccer = 0;
    tim_wdata->cr1 = 0;

    /* Drain out the DMA buffer. */
    IRQx_set_pending(dma_wdata_irq);

    switch (ff_cfg.write_drain) {
    case WDRAIN_instant:
        /* Restart read exactly where write ended. No more IDX pulses until
         * write-out is complete. */
        drive_set_restart_pos(drv);
        break;
    case WDRAIN_realtime:
        /* Nothing to do. */
        break;
    case WDRAIN_eot:
        /* Position so that an INDEX pulse is quickly triggered. */
        drv->restart_pos = drv->image->stk_per_rev - stk_ms(20);
        drv->index_suppressed = TRUE;
        break;
    }

    /* Remember where this write's DMA stream ended. */
    write = get_write(image, image->wr_prod);
    write->dma_end = ARRAY_SIZE(dma_wr->buf) - dma_wdata.cndtr;
    image->wr_prod++;

#if !defined(QUICKDISK)
    if (!ff_cfg.index_suppression && ff_cfg.write_drain != WDRAIN_realtime) {
        /* Opportunistically insert an INDEX pulse ahead of writeback. */
        drive_change_output(drv, outp_index, TRUE);
        index.fake_fired = TRUE;
        IRQx_set_pending(FLOPPY_SOFTIRQ);
        /* Position read head so it quickly triggers an INDEX pulse. */
        drv->restart_pos = drv->image->stk_per_rev - stk_ms(20);
        drv->index_suppressed = TRUE;
    }
#endif
}

static void wdata_start(void)
{
    struct write *write;
    uint32_t start_pos;

    switch (dma_wr->state) {
    case DMA_starting:
    case DMA_active:
        /* Already active: ignore WGATE glitch. */
        printk("*** WGATE glitch\n");
        return;
    case DMA_stopping:
        if ((image->wr_prod - image->wr_cons) >= ARRAY_SIZE(image->write)) {
            /* The write pipeline is full. Complain to the log. */
            printk("*** Missed write\n");
            return;
        }
        break;
    case DMA_inactive:
        /* The write path is quiescent and ready to process this new write. */
        break;
    }

    dma_wr->state = DMA_starting;

    /* Start timer. */
    tim_wdata->egr = TIM_EGR_UG;
    tim_wdata->sr = 0; /* dummy write, gives h/w time to process EGR.UG=1 */
    tim_wdata->ccer = TIM_CCER_CC1E | TIM_CCER_CC1P;
    tim_wdata->cr1 = TIM_CR1_CEN;

    /* Find rotational start position of the write, in SYSCLK ticks. */
    start_pos = max_t(int32_t, 0, time_diff(index.prev_time, time_now()));
    start_pos %= drive.image->stk_per_rev;
    start_pos *= SYSCLK_MHZ / STK_MHZ;
    write = get_write(image, image->wr_prod);
    write->start = start_pos;
    write->track = drive_calc_track(&drive);

    /* Allow IDX pulses while handling a write. */
    drive.index_suppressed = FALSE;

    /* Exit head-settling state. Ungates INDEX signal. */
    cmpxchg(&drive.step.state, STEP_settling, 0);
}

/* Called from IRQ context to stop the read stream. */
static void rdata_stop(void)
{
    uint8_t prev_state = dma_rd->state;

    /* Already inactive? Nothing to do. */
    if (prev_state == DMA_inactive)
        return;

    /* Ok we're now stopping DMA activity. */
    dma_rd->state = DMA_stopping;

    /* If DMA was not yet active, don't need to touch peripherals. */
    if (prev_state != DMA_active)
        return;

    /* Turn off the output pin */
    gpio_configure_pin(gpio_rdata, pin_rdata, GPO_rdata);

    /* Turn off timer. */
    tim_rdata->cr1 = 0;

    /* track-change = instant: Restart read stream where we left off. */
    if ((ff_cfg.track_change == TRKCHG_instant)
        && !drive.index_suppressed
        && ff_cfg.index_suppression)
        drive_set_restart_pos(&drive);
}

/* Called from user context to start the read stream. */
static void rdata_start(void)
{
    IRQ_global_disable();

    /* Did we race rdata_stop()? Then bail. */
    if (dma_rd->state == DMA_stopping)
        goto out;

    dma_rd->state = DMA_active;

    /* Start timer. */
    tim_rdata->arr = (drive.image->ticks_per_cell/(16/2)) - 1;
    tim_rdata->ccr1 = 0xFFFF;
    tim_rdata->egr = TIM_EGR_UG; /* Load fake sample from preload register. Trigger first DMA */
    tim_rdata->sr = 0; /* dummy write, gives h/w time to process EGR.UG=1 */
    tim_rdata->cr1 = TIM_CR1_CEN;

    /* Enable output. */
    if (drive.sel)
        gpio_configure_pin(gpio_rdata, pin_rdata, AFO_rdata);

    /* Exit head-settling state. Ungates INDEX signal. */
    cmpxchg(&drive.step.state, STEP_settling, 0);

out:
    IRQ_global_enable();
}

static void floppy_read_data(struct drive *drv)
{
    /* Read some track data if there is buffer space. */
    if (image_read_track(drv->image) && dma_rd->kick_dma_irq) {
        /* We buffered some more data and the DMA handler requested a kick. */
        dma_rd->kick_dma_irq = FALSE;
        IRQx_set_pending(dma_rdata_irq);
    }
}

static bool_t dma_rd_handle(struct drive *drv);

static bool_t dma_wr_handle(struct drive *drv)
{
    struct image *im = drv->image;
    struct write *write = get_write(im, im->wr_cons);
    bool_t completed;

    ASSERT((dma_wr->state == DMA_starting) || (dma_wr->state == DMA_stopping));

    /* Start a write. */
    if (!drv->writing) {

        /* Bail out of read mode. */
        if (dma_rd->state != DMA_inactive) {
            ASSERT(dma_rd->state == DMA_stopping);
            if (dma_rd_handle(drv))
                return TRUE;
            ASSERT(dma_rd->state == DMA_inactive);
        }
    
        /* Set up the track for writing. */
        image_setup_track(im, write->track, NULL);

        drv->writing = TRUE;

    }

    /* Continue a write. */
    completed = image_write_track(im);

    /* Is this write now completely processed? */
    if (completed) {

        /* Clear the staging buffer. */
        im->bufs.write_data.cons = 0;
        im->bufs.write_data.prod = 0;

        /* Align the bitcell consumer index for start of next write. */
        im->bufs.write_bc.cons = (write->bc_end + 31) & ~31;

        /* Sync back to mass storage. */
        if (!im->track_handler->async)
            F_sync(&im->fp);

        IRQ_global_disable();
        /* Consume the write from the pipeline buffer. */
        im->wr_cons++;
        /* If the buffer is empty then reset the write-bitcell ring and return 
         * to read operation. */
        if ((im->wr_cons == im->wr_prod) && (dma_wr->state != DMA_starting)) {
            im->bufs.write_bc.cons = im->bufs.write_bc.prod = 0;
            dma_wr->state = DMA_inactive;
        }
        IRQ_global_enable();

        /* This particular write is completed. */
        drv->writing = FALSE;
    }

    return FALSE;
}

bool_t floppy_handle(void)
{
    struct drive *drv = &drive;

    return ((dma_wr->state == DMA_inactive)
            ? dma_rd_handle : dma_wr_handle)(drv);
}

static void IRQ_rdata_dma(void)
{
    const uint16_t buf_mask = ARRAY_SIZE(dma_rd->buf) - 1;
    uint32_t prev_ticks_since_index, ticks, i;
    uint16_t nr_to_wrap, nr_to_cons, nr, dmacons, done;
    time_t now;
    struct drive *drv = &drive;

    /* Clear DMA peripheral interrupts. */
    dma1->ifcr = DMA_IFCR_CGIF(dma_rdata_ch);

    /* If we happen to be called in the wrong state, just bail. */
    if (dma_rd->state != DMA_active)
        return;

    /* Find out where the DMA engine's consumer index has got to. */
    dmacons = ARRAY_SIZE(dma_rd->buf) - dma_rdata.cndtr;

    /* Check for DMA catching up with the producer index (underrun). */
    if (((dmacons < dma_rd->cons)
         ? (dma_rd->prod >= dma_rd->cons) || (dma_rd->prod < dmacons)
         : (dma_rd->prod >= dma_rd->cons) && (dma_rd->prod < dmacons))
        && (dmacons != dma_rd->cons))
        printk("RDATA underrun! %x-%x-%x\n",
               dma_rd->cons, dma_rd->prod, dmacons);

    dma_rd->cons = dmacons;

    /* Find largest contiguous stretch of ring buffer we can fill. */
    nr_to_wrap = ARRAY_SIZE(dma_rd->buf) - dma_rd->prod;
    nr_to_cons = (dmacons - dma_rd->prod - 1) & buf_mask;
    nr = min(nr_to_wrap, nr_to_cons);
    if (nr == 0) /* Buffer already full? Then bail. */
        return;

    /* Now attempt to fill the contiguous stretch with flux data calculated 
     * from buffered image data. */
    prev_ticks_since_index = image_ticks_since_index(drv->image);
    dma_rd->prod += done = image_rdata_flux(
        drv->image, &dma_rd->buf[dma_rd->prod], nr);
    dma_rd->prod &= buf_mask;
    if (done != nr) {
        /* Read buffer ran dry: kick us when more data is available. */
        dma_rd->kick_dma_irq = TRUE;
        printk("kick_dma_irq\n");
    } else if (nr_to_cons - nr >= ARRAY_SIZE(dma_rd->buf)/2) {
        /* Substantial amount left unprocessed. */
        /* We didn't fill the ring: re-enter this ISR to do more work. */
        IRQx_set_pending(dma_rdata_irq);
    }

#if !defined(QUICKDISK)
    ASSERT(drv->image->index_pulses_len < MAX_CUSTOM_PULSES);
    if (drv->image->index_pulses_ver != index.custom_pulses_ver) {
        time_t current_pulse_pos = time_since(index.prev_time);
        uint32_t oldpri;

        oldpri = IRQ_save(TIMER_IRQ_PRI);
        for (i = 0; i < drv->image->index_pulses_len; i++)
            index.custom_pulses[i] = (drv->image->index_pulses[i]>>4)
                / (SYSCLK_MHZ/TIME_MHZ);
        index.custom_pulses_len = drv->image->index_pulses_len;
        index.custom_pulses_ver = drv->image->index_pulses_ver;

        for (i = 0; i < index.custom_pulses_len; i++)
            if (current_pulse_pos <= index.custom_pulses[i])
                break;
        if (i < index.custom_pulses_len)
            timer_set(&index.custom_timer, index.prev_time + index.custom_pulses[i]);
        else
            timer_cancel(&index.custom_timer);
        IRQ_restore(oldpri);
    }
#endif

    if ((dma1->isr & DMA_ISR_GIF(dma_rdata_ch)) && done > 800) {
        uint32_t new_cons = ARRAY_SIZE(dma_rd->buf) - dma_rdata.cndtr;
        uint32_t new = (new_cons - dmacons)&buf_mask;
        if (new > done/2)
            printk("Danger! RD drained=%d new=%d\n", done, new);
    }

    /* Check if we have crossed the index mark. If not, we're done. */
    if (image_ticks_since_index(drv->image) >= prev_ticks_since_index)
        return;

    /* We crossed the index mark: Synchronise index pulse to the bitstream. */
    for (;;) {
        /* Snapshot current position in flux stream, including progress through
         * current timer sample. */
        now = time_now();
        /* Ticks left in current sample. */
        ticks = tim_rdata->arr - tim_rdata->cnt;
        /* Index of next sample. */
        i = ARRAY_SIZE(dma_rd->buf) - dma_rdata.cndtr;
        dma_rd->cons = i;
        break; // FIXME
        /* If another sample was loaded meanwhile, try again for a consistent
         * snapshot. */
        if (dmacons == dma_rd->cons)
            break;
        dma_rd->cons = dmacons;
    }
    ticks += ((dma_rd->prod - i)&buf_mask) * (tim_rdata->arr+1);
    /* Subtract current flux offset beyond the index. */
    ticks -= image_ticks_since_index(drv->image);
    /* Calculate deadline for index timer. */
    ticks /= SYSCLK_MHZ/TIME_MHZ;
    timer_set(&index.timer, now + ticks);
}

static int32_t flux8_to_rll_mul(uint32_t cons, uint16_t cell, uint32_t *_nr, uint32_t bc_dat, uint32_t frac, uint32_t shift) {
    uint32_t nr = 0;
    uint16_t add = (cell>>1) + 2;
    if (FALSE) {
#pragma GCC unroll 8
    for (int i = 0; i < 8; i++, cons = cons+1) {
        int shifts;
        uint16_t curr = dma_wr->buf[cons];

        shifts = ((curr + add) * frac) >> shift;
        bc_dat <<= shifts;
        bc_dat |= 1;
        nr += shifts;
    }
    } else {
    uint16_t *buf = &dma_wr->buf[cons];
    uint16_t curr0;
    uint16_t curr1;
    uint16_t curr2;
    uint16_t curr3;
    asm(
        "ldrh %[nr], [%[buf]]\n"
        "ldrh %[curr0], [%[buf], #2]\n"
        "ldrh %[curr1], [%[buf], #4]\n"
        "ldrh %[curr2], [%[buf], #6]\n"

        "adds %[nr], %[add]\n"
        "muls %[nr], %[frac]\n"
        "asrs %[nr], %[shift]\n"
        "lsls %[bc_dat], %[nr]\n"
        "adds %[bc_dat], #1\n"

        "adds %[curr0], %[add]\n"
        "muls %[curr0], %[frac]\n"
        "asrs %[curr0], %[shift]\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr1], %[add]\n"
        "muls %[curr0], %[frac]\n"
        "asrs %[curr0], %[shift]\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr2], %[add]\n"
        "muls %[curr0], %[frac]\n"
        "asrs %[curr0], %[shift]\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"


        "ldrh %[curr0], [%[buf], #8]\n"
        "ldrh %[curr1], [%[buf], #10]\n"
        "ldrh %[curr2], [%[buf], #12]\n"
        "ldrh %[curr3], [%[buf], #14]\n"

        "adds %[curr0], %[add]\n"
        "muls %[curr0], %[frac]\n"
        "asrs %[curr0], %[shift]\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr1], %[add]\n"
        "muls %[curr0], %[frac]\n"
        "asrs %[curr0], %[shift]\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr2], %[add]\n"
        "muls %[curr0], %[frac]\n"
        "asrs %[curr0], %[shift]\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr3], %[add]\n"
        "muls %[curr0], %[frac]\n"
        "asrs %[curr0], %[shift]\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"
        : [bc_dat] "+l"(bc_dat), [nr] "=&l"(nr), [curr0] "=&l"(curr0), [curr1] "=&h"(curr1), [curr2] "=&h"(curr2), [curr3] "=&h"(curr3)
        : [add] "r"(add), [buf] "r"(buf), [frac] "l"(frac), [shift] "r"(shift)
        : "cc");
    }

    *_nr = nr;
    return bc_dat;
}

static int32_t flux8_to_rll_cell8(uint32_t cons, uint32_t *_nr, uint32_t bc_dat) {
    const uint16_t cell = 8;
    uint32_t nr = 0;
    uint16_t add = (cell>>1) + 2 + 3; /* +3 is hack */
    if (FALSE) {
#pragma GCC unroll 8
    for (int i = 0; i < 8; i++, cons = cons+1) {
        int shifts;
        uint16_t curr = dma_wr->buf[cons];

        shifts = (curr + add) / cell;
        //shifts = (curr + (cell>>1)) / cell;
        bc_dat <<= shifts;
        bc_dat |= 1;
        nr += shifts;
    }
    } else {
    uint16_t *buf = &dma_wr->buf[cons];
    uint16_t curr0;
    uint16_t curr1;
    uint16_t curr2;
    uint16_t curr3;
    asm(
        "ldrh %[nr], [%[buf]]\n"
        "ldrh %[curr0], [%[buf], #2]\n"
        "ldrh %[curr1], [%[buf], #4]\n"
        "ldrh %[curr2], [%[buf], #6]\n"
        "ldrh %[curr3], [%[buf], #8]\n"

        "adds %[nr], %[add]\n"
        "asrs %[nr], #3\n"
        "lsls %[bc_dat], %[nr]\n"
        "adds %[bc_dat], #1\n"

        "adds %[curr0], %[add]\n"
        "asrs %[curr0], #3\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr1], %[add]\n"
        "asrs %[curr0], #3\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr2], %[add]\n"
        "asrs %[curr0], #3\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr3], %[add]\n"
        "asrs %[curr0], #3\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"


        "ldrh %[curr0], [%[buf], #10]\n"
        "ldrh %[curr1], [%[buf], #12]\n"
        "ldrh %[curr2], [%[buf], #14]\n"

        "adds %[curr0], %[add]\n"
        "asrs %[curr0], #3\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr1], %[add]\n"
        "asrs %[curr0], #3\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"

        "adds %[curr0], %[curr2], %[add]\n"
        "asrs %[curr0], #3\n"
        "lsls %[bc_dat], %[curr0]\n"
        "adds %[bc_dat], #1\n"
        "adds %[nr], %[curr0]\n"
        : [bc_dat] "+l"(bc_dat), [nr] "=&l"(nr), [curr0] "=&l"(curr0), [curr1] "=&h"(curr1), [curr2] "=&h"(curr2), [curr3] "=&h"(curr3)
        : [add] "M"(add), [buf] "r"(buf)
        : "cc");
    }

    *_nr = nr;
    return bc_dat;
}

static inline uint32_t _clz(uint32_t x) {
    uint32_t result;
    asm ("clz %0,%1" : "=r" (result) : "r" (x));
    return result;
}

static void IRQ_wdata_dma(void)
{
    const uint16_t buf_mask = ARRAY_SIZE(dma_wr->buf) - 1;
    uint16_t cons, prod, curr;
    uint16_t cell = image->write_bc_ticks, window;
    uint32_t bc_dat = 0, bc_prod;
    uint32_t *bc_buf = image->bufs.write_bc.p;
    unsigned int sync = image->sync;
    unsigned int bc_bufmask = (image->bufs.write_bc.len / 4) - 1;
    struct write *write = NULL;
    uint16_t num_drained;
    const bool_t hack7is8 = TRUE;
    uint32_t cyccnt_start, cyccnt_end;

    window = cell + (cell >> 1);

    /* Clear DMA peripheral interrupts. */
    dma1->ifcr = DMA_IFCR_CGIF(dma_wdata_ch);

    /* If we happen to be called in the wrong state, just bail. */
    if (dma_wr->state == DMA_inactive)
        return;

    /* Find out where the DMA engine's producer index has got to. */
    prod = ARRAY_SIZE(dma_wr->buf) - dma_wdata.cndtr;

    /* Check if we are processing the tail end of a write. */
    barrier(); /* interrogate peripheral /then/ check for write-end. */
    if (image->wr_bc != image->wr_prod) {
        write = get_write(image, image->wr_bc);
        prod = write->dma_end;
    } else if (((prod - dma_wr->cons) & buf_mask) > 31)
        /* Keep alignment for flux8_to_rll, plus some to avoid stepping into
         * the next half of buffer and incurring the loop overhead. */
        prod &= ~31;

    /* Process the flux timings into the raw bitcell buffer. */
    bc_prod = image->bufs.write_bc.prod;
    bc_dat = image->write_bc_window;
    num_drained = (prod - dma_wr->cons)&buf_mask;
    cyccnt_start = dwt->cyccnt;
    for (cons = dma_wr->cons; cons != prod; cons = (cons+1) & buf_mask) {
        if (sync == SYNC_none && (!hack7is8 || cell == 7) && ((prod - cons) & buf_mask) >= 8 && cons + 7 <= buf_mask) {
            uint16_t todo = min_t(uint16_t, (prod - cons) & buf_mask, ARRAY_SIZE(dma_wr->buf) - cons);
            uint16_t end = cons + (todo & ~7);
            uint32_t bc_prod_hi = bc_prod / 32;
            uint32_t bc_prod_lo = bc_prod % 32;
            uint32_t shift = (31-_clz((uint32_t)cell))+16;
            uint32_t frac = ((1<<shift)+(uint32_t)cell) / (uint32_t)cell;
            do {
                uint32_t nr = 0;
                uint32_t tmp_dat;
                if (!hack7is8) // 13-14 (C); 12-13 (full asm)
                    tmp_dat = flux8_to_rll_mul(cons, cell, &nr, bc_dat, frac, shift);
                else // 12-13 (C); 11 (full asm)
                    tmp_dat = flux8_to_rll_cell8(cons, &nr, bc_dat);
                /*if (unlikely(nr > 32)) {
                    printk("Exceeded\n");
                    goto slow;
                }*/
                cons += 8;
                bc_prod_lo += nr;
                if (bc_prod_lo >= 32) {
                    bc_prod_lo -= 32;
                    bc_dat <<= nr - bc_prod_lo;
                    bc_dat |= tmp_dat >> bc_prod_lo;
                    bc_buf[bc_prod_hi++ & bc_bufmask] = htobe32(bc_dat);
                }
                bc_dat = tmp_dat;
            } while (cons != end);
            bc_prod = bc_prod_hi * 32 + bc_prod_lo;
            cons--;
            continue;
        }
//slow:
        curr = dma_wr->buf[cons]+2;
        while (curr > window) {
            curr -= cell;
            bc_dat <<= 1;
            bc_prod++;
            if (!(bc_prod&31))
                bc_buf[((bc_prod-1) / 32) & bc_bufmask] = htobe32(bc_dat);
        }
        bc_dat = (bc_dat << 1) | 1;
        bc_prod++;
        switch (sync) {
        case SYNC_fm:
            /* FM clock sync clock byte is 0xc7. Check for:
             * 1010 1010 1010 1010 1x1x 0x0x 0x1x 1x1x */
            if ((bc_dat & 0xffffd555) == 0x55555015)
                bc_prod = (bc_prod - 31) | 31;
            break;
        case SYNC_mfm:
            if (bc_dat == 0x44894489)
                bc_prod &= ~31;
            break;
        }
        if (!(bc_prod&31))
            bc_buf[((bc_prod-1) / 32) & bc_bufmask] = htobe32(bc_dat);
    }
    cyccnt_end = dwt->cyccnt;

    if (bc_prod & 31)
        bc_buf[(bc_prod / 32) & bc_bufmask] = htobe32(bc_dat << (-bc_prod&31));

    /* Processing the tail end of a write? */
    if (write != NULL) {
        /* Remember where this write's bitcell data ends. */
        write->bc_end = bc_prod;
        if (++image->wr_bc != image->wr_prod)
            IRQx_set_pending(dma_wdata_irq);
        /* Initialise decoder state for the start of the next write. */
        bc_prod = (bc_prod + 31) & ~31;
        bc_dat = ~0;
    }

    /* Save our progress for next time. */
    image->write_bc_window = bc_dat;
    image->bufs.write_bc.prod = bc_prod;
    dma_wr->cons = cons;

    if (dma1->isr & DMA_ISR_GIF(dma_wdata_ch)) {
        uint16_t new_prod = ARRAY_SIZE(dma_wr->buf) - dma_wdata.cndtr;
        printk("Danger! WR drained=%d new=%d cyc/drained=%d\n", num_drained, (new_prod - prod)&buf_mask, (cyccnt_end - cyccnt_start)/num_drained);
    }
}

void floppy_sync(void)
{
    struct drive *drv = &drive;
    struct image *im = drv->image;

    image_sync(im);
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
