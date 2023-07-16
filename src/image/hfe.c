/*
 * hfe.c
 * 
 * HxC Floppy Emulator (HFE) image files.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* NB. Fields are little endian. */
struct disk_header {
    char sig[8];
    uint8_t formatrevision;
    uint8_t nr_tracks, nr_sides;
    uint8_t track_encoding;
    uint16_t bitrate; /* kB/s, approx */
    uint16_t rpm; /* unused, can be zero */
    uint8_t interface_mode;
    uint8_t rsvd; /* set to 1? */
    uint16_t track_list_offset;
    /* from here can write 0xff to end of block... */
    uint8_t write_allowed;
    uint8_t single_step;
    uint8_t t0s0_altencoding, t0s0_encoding;
    uint8_t t0s1_altencoding, t0s1_encoding;
};

/* track_encoding */
enum {
    ENC_ISOIBM_MFM,
    ENC_Amiga_MFM,
    ENC_ISOIBM_FM,
    ENC_Emu_FM,
    ENC_Unknown = 0xff
};

/* interface_mode */
enum {
    IFM_IBMPC_DD,
    IFM_IBMPC_HD,
    IFM_AtariST_DD,
    IFM_AtariST_HD,
    IFM_Amiga_DD,
    IFM_Amiga_HD,
    IFM_CPC_DD,
    IFM_GenericShugart_DD,
    IFM_IBMPC_ED,
    IFM_MSX2_DD,
    IFM_C64_DD,
    IFM_EmuShugart_DD,
    IFM_S950_DD,
    IFM_S950_HD,
    IFM_Disable = 0xfe
};

struct track_header {
    uint16_t offset;
    uint16_t len;
};

/* HFEv3 opcodes. Bit order is reversed to match raw HFE bit order. */
enum {
    OP_Nop      = 0x0f, /* Nop */
    OP_Index    = 0x8f, /* Index mark */
    OP_Bitrate  = 0x4f, /* +1 byte: new bitrate */
    OP_SkipBits = 0xcf, /* +1 byte: skip 1-7 bits in following byte */
    OP_Rand     = 0x2f  /* Random byte (or bits, if following OP_skip) */
};

static void hfe_seek_track(struct image *im, uint16_t track, bool_t async);

static bool_t hfe_open(struct image *im)
{
    struct disk_header dhdr;
    uint16_t bitrate;

    F_read(&im->fp, &dhdr, sizeof(dhdr), NULL);
    if (!strncmp(dhdr.sig, "HXCHFEV3", sizeof(dhdr.sig))) {
        if (dhdr.formatrevision > 0)
            return FALSE;
        im->hfe.is_v3 = TRUE;
    } else if (!strncmp(dhdr.sig, "HXCPICFE", sizeof(dhdr.sig))) {
        if (dhdr.formatrevision > 1)
            return FALSE;
        im->hfe.is_v3 = FALSE;
    } else {
        return FALSE;
    }

    /* Sanity-check the header fields. */
    bitrate = le16toh(dhdr.bitrate);
    if ((dhdr.nr_tracks == 0)
        || (dhdr.nr_sides < 1) || (dhdr.nr_sides > 2)
        || (bitrate == 0)) {
        return FALSE;
    }

    im->hfe.double_step = !dhdr.single_step;
    im->hfe.tlut_base = le16toh(dhdr.track_list_offset);
    im->nr_cyls = dhdr.nr_tracks;
    im->step = im->hfe.double_step ? 2 : 1;
    im->nr_sides = dhdr.nr_sides;
    im->write_bc_ticks = sampleclk_us(500) / bitrate;
    im->ticks_per_cell = im->write_bc_ticks * 16;
    im->sync = SYNC_none;

    /* Aggressively batch our reads at HD data rate, as that can be faster
     * than some USB drives will serve up a single block.*/
    im->hfe.fcache = file_cache_init(&im->fp,
            (im->write_bc_ticks > sampleclk_ns(1500)) ? 4 : 8,
            im->bufs.read_data.p,
            im->bufs.read_data.p + im->bufs.read_data.len);

    /* Get an initial value for ticks per revolution. */
    hfe_seek_track(im, 0, FALSE);
    im->cur_track = -1;

    return TRUE;
}

static void hfe_seek_track(struct image *im, uint16_t track, bool_t async)
{
    struct track_header thdr;

    if (async) {
        file_cache_io_limit(im->hfe.fcache, 1);
        file_cache_read(im->hfe.fcache,
                &thdr, im->hfe.tlut_base*512 + (track/2)*4, sizeof(thdr));
        file_cache_io_limit(im->hfe.fcache, 0 /* no limit */);
    } else {
        F_lseek(&im->fp, im->hfe.tlut_base*512 + (track/2)*4);
        F_read(&im->fp, &thdr, sizeof(thdr), NULL);
    }

    im->hfe.trk_off = le16toh(thdr.offset);
    im->hfe.trk_len = le16toh(thdr.len) / 2;
    im->tracklen_bc = im->hfe.trk_len * 8;
    if (im->hfe.is_v3 && im->tracklen_ticks) {
        /* Opcodes in v3 make it difficult to predict the track's length. Keep
         * the previous track's value since this isn't the first seek. */
    } else {
        im->tracklen_ticks = im->tracklen_bc * im->ticks_per_cell;
        im->stk_per_rev = stk_sampleclk(im->tracklen_ticks / 16);
    }

    file_cache_readahead(im->hfe.fcache,
            (LBA_t)im->hfe.trk_off * 512, im->hfe.trk_len*2, 12*1024);
}

static void hfe_setup_track(
    struct image *im, uint16_t track, uint32_t *start_pos)
{
    struct image_buf *bc = &im->bufs.read_bc;
    uint32_t start_ticks;
    uint8_t cyl = track >> (im->hfe.double_step ? 2 : 1);
    uint8_t side = track & (im->nr_sides - 1);

    track = cyl*2 + side;
    if (track/2 != im->cur_track/2) {
        file_cache_sync_wait(im->hfe.fcache);

        im->cur_track = track;
        hfe_seek_track(im, track, TRUE);
    } else if (track != im->cur_track) {
        im->cur_track = track;
    }

    /* If track does not fit in memory, now is a good time to flush writes to
     * reduce chances of future buffer underrun caused by a very slow write.
     * However if write-drain=realtime, then any delays cut into reads so we
     * just accept the buffer underrun risk. */
    if ((im->hfe.trk_len*2 + 511) / 512 > im->bufs.read_data.len
            && ff_cfg.write_drain != WDRAIN_realtime)
        file_cache_sync_wait(im->hfe.fcache);

    start_ticks = start_pos ? *start_pos : get_write(im, im->wr_cons)->start;

    im->cur_ticks = start_ticks * 16;
    im->cur_bc = udiv64((uint64_t)im->cur_ticks * im->tracklen_bc,
                        im->tracklen_ticks);
    if ((im->cur_ticks >= im->tracklen_ticks) ||
        (im->cur_bc >= im->tracklen_bc)) {
        im->cur_ticks = 0;
        im->cur_bc = 0;
    }
    im->ticks_since_flux = 0;

    bc->prod = bc->cons = 0;

    if (start_pos) {
        /* Read mode. */
        im->hfe.trk_pos = (im->cur_bc/8) & ~255;
        /* Consumer may be ahead of producer, but only until the first read
         * completes. */
        bc->cons = im->cur_bc % (256*8);
    } else {
        /* Write mode. */
        im->hfe.trk_pos = im->cur_bc / 8;
        if (im->hfe.is_v3) {
            /* Provide context to the write to avoid corrupting an opcode. */
            if ((im->hfe.trk_pos & 255) == 0 && im->hfe.trk_pos != 0)
                im->hfe.trk_pos--;
            else if ((im->hfe.trk_pos & 255) == 1)
                im->hfe.trk_pos = (im->hfe.trk_pos+1) % im->hfe.trk_len;
        }
        im->hfe.fresh_seek = TRUE;
    }
}

static bool_t hfe_read_track(struct image *im)
{
    struct image_buf *bc = &im->bufs.read_bc;
    uint8_t *bc_b = bc->p;
    uint32_t bc_len, bc_mask, bc_space, bc_p, bc_c;
    unsigned int nr_sec;

    /* Fill the raw-bitcell ring buffer. */
    bc_p = bc->prod / 8;
    bc_c = bc->cons / 8;
    bc_len = bc->len;
    bc_mask = bc_len - 1;
    bc_space = bc_len - (int16_t)(bc_p - bc_c);

    nr_sec = bc_space/256;
    if (nr_sec == 0) {
        file_cache_progress(im->hfe.fcache);
        return FALSE;
    }

    while (nr_sec--) {
        FSIZE_t off = im->hfe.trk_off * 512
            + im->hfe.trk_pos * 2
            + (im->cur_track&1)*256;
        if (!file_cache_try_read(im->hfe.fcache, &bc_b[bc_p & bc_mask], off, 256))
            break;
        im->hfe.trk_pos += 256;
        if (im->hfe.trk_pos >= im->hfe.trk_len)
            im->hfe.trk_pos = 0;
        bc_p += 256;
    }

    barrier();
    bc->prod = bc_p * 8;

    return TRUE;
}

static uint16_t hfe_rdata_flux(struct image *im, uint16_t *tbuf, uint16_t nr)
{
    struct image_buf *bc = &im->bufs.read_bc;
    uint8_t *bc_b = bc->p;
    uint32_t bc_c = bc->cons, bc_p = bc->prod, bc_mask = bc->len - 1;
    uint32_t ticks = im->ticks_since_flux;
    uint32_t ticks_per_cell = im->ticks_per_cell;
    uint32_t bit_off, todo = nr;
    uint8_t x;
    bool_t is_v3 = im->hfe.is_v3;

    while ((uint32_t)(bc_p - bc_c) >= 3*8) {

        if (im->cur_bc >= im->tracklen_bc) {
            /* Malformed HFE v3 file can trigger this assertion. Requires a
             * multi-byte opcode which extends beyond reported track length. */
            ASSERT(im->cur_bc == im->tracklen_bc);
            im->tracklen_ticks = im->cur_ticks;
            im->stk_per_rev = stk_sampleclk(im->tracklen_ticks / 16);
            im->cur_bc = im->cur_ticks = 0;
            /* Skip tail of current 256-byte block. */
            bc_c = (bc_c + 256*8-1) & ~(256*8-1);
            continue;
        }

        bit_off = bc_c % 8;
        x = bc_b[(bc_c/8) & bc_mask];
        bc_c += 8 - bit_off;
        im->cur_bc += 8 - bit_off;

        if (is_v3 && ((x & 0xf) == 0xf)) {
            /* V3 byte-aligned opcode processing. */
            switch (x) {
            case OP_Nop:
            case OP_Index:
            default:
                continue;
            case OP_Bitrate:
                x = _rbit32(bc_b[(bc_c/8) & bc_mask]) >> 24;
                im->ticks_per_cell = ticks_per_cell = 
                    (sampleclk_us(2) * 16 * x) / 72;
                im->write_bc_ticks = ticks_per_cell / 16;
                bc_c += 8;
                im->cur_bc += 8;
                continue;
            case OP_SkipBits:
                x = (_rbit32(bc_b[(bc_c/8) & bc_mask]) >> 24) & 7;
                bc_c += 8 + x;
                im->cur_bc += 8 + x;
                continue;
            case OP_Rand:
                x = rand();
                break;
            }
        }

        x >>= bit_off;
        im->cur_ticks += (8 - bit_off) * ticks_per_cell;
        while (bit_off < 8) {
            bit_off++;
            ticks += ticks_per_cell;
            if (x & 1) {
                *tbuf++ = (ticks >> 4) - 1;
                ticks &= 15;
                if (!--todo) {
                    bc_c -= 8 - bit_off;
                    im->cur_bc -= 8 - bit_off;
                    im->cur_ticks -= (8 - bit_off) * ticks_per_cell;
                    goto out;
                }
            }
            x >>= 1;
        }

        /* Subdivide a long flux gap to avoid overflowing the 16-bit timer.
         * This mishandles long No Flux Areas slightly, by regularly emitting
         * a flux-reversal pulse every 2^14 sampleclk ticks. */
        if (unlikely((ticks >> (15+4)) != 0)) {
            *tbuf++ = (1u << 14) - 1;
            ticks -= 1u << (14+4);
            if (!--todo)
                goto out;
        }

    }

out:
    bc->cons = bc_c;
    im->ticks_since_flux = ticks;
    return nr - todo;
}

static bool_t hfe_write_track(struct image *im)
{
    bool_t flush;
    struct write *write = get_write(im, im->wr_cons);
    struct image_buf *wr = &im->bufs.write_bc;
    uint8_t *buf = wr->p;
    uint8_t b;
    unsigned int bufmask = wr->len - 1;
    uint8_t *w, *wrbuf;
    uint32_t i, space, c = wr->cons / 8, p = wr->prod / 8;
    bool_t is_v3 = im->hfe.is_v3;

    /* If we are processing final data then use the end index, rounded to
     * nearest. */
    barrier();
    flush = (im->wr_cons != im->wr_bc);
    if (flush)
        p = (write->bc_end + 4) / 8;

    for (;;) {

        uint32_t foff, off = im->hfe.trk_pos;
        UINT nr;

        /* All bytes remaining in the raw-bitcell buffer. */
        nr = space = (p - c) & bufmask;
        /* Limit to end of current 256-byte HFE block. */
        nr = min_t(UINT, nr, 256 - (off & 255));
        /* Limit to end of HFE track. */
        nr = min_t(UINT, nr, im->hfe.trk_len - off);

        /* Bail if no bytes to write. */
        if (nr == 0)
            break;

        foff = im->hfe.trk_off * 512 + ((off & ~255) << 1);
        if ((wrbuf = file_cache_peek_write(im->hfe.fcache, foff)) == NULL) {
            flush = FALSE;
            break;
        }
        w = wrbuf
            + (im->cur_track & 1) * 256
            + (off & 255);

        i = 0;

        if (im->hfe.fresh_seek && is_v3 && off != 0) {
            /* Avoid starting write in the middle of an opcode. */
            if (w[-2] == OP_SkipBits) {
                i++;
            } else {
                switch (w[-1]) {
                case OP_SkipBits:
                    i += 2;
                    break;
                case OP_Bitrate:
                    i++;
                    break;
                default:
                    break;
                }
            }
        }
        im->hfe.fresh_seek = FALSE;

        while (i < nr) {
            if (is_v3 && (w[i] & 0xf) == 0xf) {
                switch (w[i]) {
                case OP_SkipBits:
                    /* Keep the write byte-aligned. This changes the length of
                     * the track by 8+skip bitcells, but overwriting OP_SkipBits
                     * should be rare. */
                    w[i++] = OP_Nop;
                    continue;

                case OP_Bitrate:
                    /* Assume bitrate does not change for the entire track, and
                     * write_bc_ticks already adjusted when reading. */
                    i++;
                    /* fallthrough */
                case OP_Nop:
                case OP_Index:
                default:
                    /* Preserve opcode. But making sure not to write past end of
                     * buffer. */
                    i++;
                    continue;

                case OP_Rand:
                    /* Replace with data. */
                    break;
                }
            }
            b = _rbit32(buf[c++ & bufmask]) >> 24;
            /* HFEv3 can't handle a run of 1s as it will appear like an opcode.
             * If we encounter such a run, then either it is garbage or the
             * file needs twice the bitrate. Assume garbage; a bad bitrate would
             * fail rapidly. */
            if (is_v3 && (b & 0xf) == 0xf)
                b ^= 2;
            w[i++] = b;
        }

        im->hfe.trk_pos += i; /* i may be larger than nr due to opcodes. */
        if (im->hfe.trk_pos >= im->hfe.trk_len) {
            ASSERT(im->hfe.trk_pos == im->hfe.trk_len);
            im->hfe.trk_pos = 0;
        }
    }

    if (flush)
        file_cache_sync(im->hfe.fcache);
    else
        file_cache_progress(im->hfe.fcache);

    wr->cons = c * 8;

    return flush;
}

static void hfe_sync(struct image *im)
{
    file_cache_sync_wait(im->hfe.fcache);
    file_cache_shutdown(im->hfe.fcache);
}

const struct image_handler hfe_image_handler = {
    .open = hfe_open,
    .setup_track = hfe_setup_track,
    .read_track = hfe_read_track,
    .rdata_flux = hfe_rdata_flux,
    .write_track = hfe_write_track,
    .sync = hfe_sync,

    .async = TRUE,
};

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
