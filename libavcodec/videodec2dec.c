/*
 * PS4 Videodec2 H.264 hardware decoder.
 *
 * Hands whole access units to libSceVideodec2 and receives decoded NV12
 * frames.  The decode flow follows the implementation validated on console
 * by the Moonlight-ps4 project (src/video/decoder_orbis.c), adapted to
 * FFmpeg's decode callback model.
 *
 * Every buffer the firmware can see (decoder working areas, the rotated
 * access-unit buffers and the framebuffers) lives in sceKernel direct
 * memory.  The Vdec worker reaches them through physical addresses, so
 * handing it ordinary heap memory (malloc / av_malloc) faults on the first
 * entropy parse.  Only the NV12 frame we hand back to the caller comes from
 * ff_get_buffer().
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdint.h>
#include <string.h>

#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "hwconfig.h"
#include "internal.h"
#include "videodec2dec.h"
#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/error.h"
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixfmt.h"

/* Number of extra framebuffers beyond the pipeline depth: one for the frame
 * the caller may still be reading, plus one of slack for the pipelined
 * decoder returning output with an extra call of delay. */
#define VD2_FB_SLACK 2

/* AU size preallocated per rotated slot. Grows on demand. */
#define VD2_AU_INITIAL_SIZE (1024 * 1024)

/* Consecutive isErrorFrame results before the decoder is reset. */
#define VD2_ERR_RESET_THRESHOLD 30

typedef struct Videodec2DecodeContext {
    AVClass *av_class;

    videodec2_api_t api;
    OrbisVideodec2Decoder dec;
    int loaded;

    int width;    /* visible width  */
    int height;   /* visible height */

    /* Decoder working memories (see videodec2dec.h for the types). */
    Videodec2Dmem cpu_mem;
    Videodec2Dmem gpu_mem;
    Videodec2Dmem cpugpu_mem;

    /* Rotated framebuffers handed to Decode(). */
    Videodec2Dmem fb_mem;
    size_t  fb_stride;   /* bytes per framebuffer */
    int     fb_count;
    int     fb_next;

    /* Rotated access-unit (bitstream) buffers. With depth > 1 the Vdec
     * worker may still be parsing AU(N) after Decode() returns, so the
     * next AU must not overwrite it. */
    Videodec2Dmem au_mem[ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH];
    int     au_count;
    int     au_next;

    uint8_t sps_scratch[256];
    int     sps_patched;
    int     sps_logged;

    int err_streak;
    int logged_sample;
    int logged_au;
    int logged_decode_err;

    /* Options. */
    int pipeline_depth;
    int thread_prio;
} Videodec2DecodeContext;

/* -------------------------------------------------------------------------
 * SPS rewriting: force num_ref_frames = 1.
 *
 * The PS4 Videodec2 decoder misbehaves with the reference counts NVENC and
 * Sunshine emit (deep DPB), which on console shows up as ~150ms Decode()
 * calls followed by an IDR storm.  Moonlight-ps4 fixes it by parsing the SPS
 * and setting num_ref_frames to 1 (gs_sps_fix).  The same rewrite is done
 * here at the bit level: walk the SPS up to num_ref_frames, replace that one
 * ue(v), and copy the remaining RBSP verbatim.  Any parse error makes us
 * fall back to the untouched SPS rather than emit a broken one.
 * ---------------------------------------------------------------------- */

/* Remove emulation prevention bytes (00 00 03 -> 00 00). */
static int vd2_ebsp_to_rbsp(const uint8_t *src, int len, uint8_t *dst, int dst_size)
{
    int zeros = 0, n = 0;

    for (int i = 0; i < len; i++) {
        if (zeros >= 2 && src[i] == 0x03) {
            zeros = 0;
            continue;
        }
        if (n >= dst_size)
            return -1;
        dst[n++] = src[i];
        zeros = src[i] ? 0 : zeros + 1;
    }
    return n;
}

/* Insert emulation prevention bytes. */
static int vd2_rbsp_to_ebsp(const uint8_t *src, int len, uint8_t *dst, int dst_size)
{
    int zeros = 0, n = 0;

    for (int i = 0; i < len; i++) {
        if (zeros >= 2 && src[i] <= 0x03) {
            if (n >= dst_size)
                return -1;
            dst[n++] = 0x03;
            zeros = 0;
        }
        if (n >= dst_size)
            return -1;
        dst[n++] = src[i];
        zeros = src[i] ? 0 : zeros + 1;
    }
    return n;
}

typedef struct VD2BitReader {
    const uint8_t *data;
    int            size_bits;
    int            bit;
    int            error;
} VD2BitReader;

static unsigned vd2_br_u(VD2BitReader *br, int bits)
{
    unsigned v = 0;

    if (bits < 0 || bits > 32 || br->bit + bits > br->size_bits) {
        br->error = 1;
        return 0;
    }
    for (int i = 0; i < bits; i++, br->bit++)
        v = (v << 1) | ((br->data[br->bit >> 3] >> (7 - (br->bit & 7))) & 1);
    return v;
}

static unsigned vd2_br_ue(VD2BitReader *br)
{
    int zeros = 0;

    while (!br->error && br->bit < br->size_bits) {
        int bit = (br->data[br->bit >> 3] >> (7 - (br->bit & 7))) & 1;
        br->bit++;
        if (bit)
            break;
        if (++zeros > 31) {
            br->error = 1;
            return 0;
        }
    }
    if (br->error)
        return 0;
    return (1u << zeros) - 1 + vd2_br_u(br, zeros);
}

static int vd2_br_se(VD2BitReader *br)
{
    unsigned ue = vd2_br_ue(br);

    if (br->error)
        return 0;
    return (ue & 1) ? (int)((ue + 1) >> 1) : -(int)(ue >> 1);
}

/* Number of bits a ue(v) with value v occupies. */
static int vd2_ue_len(unsigned v)
{
    unsigned x = v + 1;
    int len = 1;

    while (x >>= 1)
        len += 2;   /* one leading zero + one payload bit per iteration */
    return len;
}

static int vd2_profile_has_chroma_info(unsigned profile)
{
    switch (profile) {
    case 100: case 110: case 122: case 244: case 44: case 83:
    case 86:  case 118: case 128: case 138: case 139: case 134: case 135:
        return 1;
    default:
        return 0;
    }
}

/* Skip the fields between seq_parameter_set_id and num_ref_frames. */
static int vd2_skip_sps_to_num_ref(VD2BitReader *br, unsigned profile)
{
    if (vd2_profile_has_chroma_info(profile)) {
        unsigned chroma_format_idc = vd2_br_ue(br);

        if (br->error)
            return -1;
        if (chroma_format_idc == 3)
            (void)vd2_br_u(br, 1);          /* separate_colour_plane_flag */
        (void)vd2_br_ue(br);                /* bit_depth_luma_minus8   */
        (void)vd2_br_ue(br);                /* bit_depth_chroma_minus8 */
        (void)vd2_br_u(br, 1);              /* qpprime_y_zero_transform_bypass_flag */
        if (vd2_br_u(br, 1)) {              /* seq_scaling_matrix_present_flag */
            int lists = chroma_format_idc != 3 ? 8 : 12;
            for (int i = 0; i < lists && !br->error; i++) {
                if (!vd2_br_u(br, 1))
                    continue;
                /* scaling_list(): 16 entries for list 0..5, 64 afterwards. */
                int count = i < 6 ? 16 : 64;
                int last = 8, next = 8;
                for (int j = 0; j < count && !br->error; j++) {
                    if (next != 0) {
                        int delta = vd2_br_se(br);
                        next = (last + delta + 256) % 256;
                    }
                    if (next != 0)
                        last = next;
                }
            }
        }
    }
    if (br->error)
        return -1;

    (void)vd2_br_ue(br);                    /* log2_max_frame_num_minus4 */
    {
        unsigned poc_type = vd2_br_ue(br);

        if (br->error)
            return -1;
        if (poc_type == 0) {
            (void)vd2_br_ue(br);            /* log2_max_pic_order_cnt_lsb_minus4 */
        } else if (poc_type == 1) {
            unsigned n;
            (void)vd2_br_u(br, 1);          /* delta_pic_order_always_zero_flag */
            (void)vd2_br_se(br);            /* offset_for_non_ref_pic */
            (void)vd2_br_se(br);            /* offset_for_top_to_bottom_field */
            n = vd2_br_ue(br);
            if (br->error || n > 255)
                return -1;
            for (unsigned i = 0; i < n; i++)
                (void)vd2_br_se(br);
        }
    }
    return br->error ? -1 : 0;
}

/*
 * Rewrite num_ref_frames in an SPS NAL given without its start code but
 * including the NAL header byte.  Writes to dst and returns the new NAL
 * size, or -1 when the SPS is left untouched.
 */
static int videodec2_fix_sps(const uint8_t *nal, int nal_len,
                             uint8_t *dst, int dst_size)
{
    uint8_t rbsp[256], out[256];
    VD2BitReader br;
    int rbsp_len, out_rbsp_len, n, prefix_bytes, rem_bits;
    int field_bit_pos, field_bit_len;
    unsigned profile, num_ref;

    if (nal_len <= 1 || nal_len + 8 > (int)sizeof(rbsp))
        return -1;

    /* NAL header byte is kept as-is; parse the RBSP payload. */
    rbsp_len = vd2_ebsp_to_rbsp(nal + 1, nal_len - 1, rbsp, sizeof(rbsp));
    if (rbsp_len <= 0)
        return -1;

    /* Pass 1: locate num_ref_frames. */
    br.data      = rbsp;
    br.size_bits = rbsp_len * 8;
    br.bit       = 0;
    br.error     = 0;

    profile = vd2_br_u(&br, 8);
    (void)vd2_br_u(&br, 8);     /* constraint flags */
    (void)vd2_br_u(&br, 8);     /* level_idc */
    (void)vd2_br_ue(&br);       /* sps id */
    if (vd2_skip_sps_to_num_ref(&br, profile) < 0)
        return -1;

    field_bit_pos = br.bit;
    num_ref       = vd2_br_ue(&br);
    if (br.error)
        return -1;
    field_bit_len = br.bit - field_bit_pos;

    if (num_ref == 1)
        return -1;              /* nothing to do */

    /* Pass 2: rebuild the RBSP with the field replaced. */
    prefix_bytes = field_bit_pos >> 3;
    rem_bits     = field_bit_pos & 7;

    if (prefix_bytes >= (int)sizeof(out))
        return -1;

    /* Start from the untouched prefix (whole bytes only). */
    memset(out, 0, sizeof(out));
    memcpy(out, rbsp, prefix_bytes);

    {
        int      total_bits = field_bit_pos + vd2_ue_len(1) +
                              (rbsp_len * 8 - field_bit_pos - field_bit_len);
        int      out_bytes  = (total_bits + 7) >> 3;
        uint32_t bitpos;

        if (out_bytes + 8 > dst_size || out_bytes >= (int)sizeof(out))
            return -1;

        bitpos = (uint32_t)prefix_bytes * 8;

#define VD2_PUT_BIT(b) do {                                              \
        if ((b))                                                         \
            out[bitpos >> 3] |= 1 << (7 - (bitpos & 7));                  \
        bitpos++;                                                        \
    } while (0)

        /* The trailing bits of the last copied byte. */
        for (int i = 0; i < rem_bits; i++) {
            int bit = (rbsp[prefix_bytes] >> (7 - i)) & 1;
            VD2_PUT_BIT(bit);
        }
        /* ue(v) with value 1 -> "010". */
        VD2_PUT_BIT(0);
        VD2_PUT_BIT(1);
        VD2_PUT_BIT(0);

        /* Copy the remaining RBSP bits verbatim. */
        for (int i = field_bit_pos + field_bit_len; i < rbsp_len * 8; i++) {
            int bit = (rbsp[i >> 3] >> (7 - (i & 7))) & 1;
            VD2_PUT_BIT(bit);
        }
#undef VD2_PUT_BIT

        out_rbsp_len = out_bytes;
    }

    n = vd2_rbsp_to_ebsp(out, out_rbsp_len, dst + 1, dst_size - 1);
    if (n <= 0)
        return -1;

    dst[0] = nal[0];
    return n + 1;
}

/* -------------------------------------------------------------------------
 * Annex-B access unit assembly.
 * ---------------------------------------------------------------------- */

/* Find the next start code at or after pos.  Returns the offset of its
 * first byte and stores the start code length (3 or 4), or -1 if none was
 * found.  The scan starts at pos + 3 so a zero that is already part of the
 * previous NAL is not stolen by the next start code. */
static int vd2_find_start_code(const uint8_t *buf, int len, int pos, int *sc_len)
{
    for (int i = pos; i + 3 <= len; i++) {
        if (buf[i] != 0 || buf[i + 1] != 0 || buf[i + 2] != 1)
            continue;
        /* A 4-byte start code has a leading zero that belongs to it. */
        if (i > 0 && buf[i - 1] == 0 && (i < 2 || buf[i - 2] != 0)) {
            *sc_len = 4;
            return i - 1;
        }
        *sc_len = 3;
        return i;
    }
    return -1;
}

/*
 * Copy one annex-B access unit into dst, rewriting any SPS it contains.
 * The firmware needs every NAL of the AU (SPS, PPS, SEI and the slice), so
 * the whole packet is walked instead of looking at the first start code.
 * Returns the number of bytes written, or a negative AVERROR.
 */
static int videodec2_build_au(Videodec2DecodeContext *s, uint8_t *dst, size_t cap,
                              const uint8_t *src, int len)
{
    int pos = 0, off = 0, found = 0;

    while (pos < len) {
        int sc_pos, sc_len = 0, nal_start, nal_end, next_pos, nal_len, written;
        uint8_t *out_nal;
        size_t   need;

        sc_pos = vd2_find_start_code(src, len, pos, &sc_len);
        if (sc_pos < 0)
            break;

        found    = 1;
        nal_start = sc_pos + sc_len;
        if (nal_start >= len)
            break;

        next_pos = vd2_find_start_code(src, len, nal_start, &sc_len);
        if (next_pos < 0)
            next_pos = len;
        nal_end = next_pos;

        /* Give a trailing zero to the next start code rather than to this
         * NAL. */
        while (nal_end > nal_start && src[nal_end - 1] == 0)
            nal_end--;

        nal_len = nal_end - nal_start;
        if (nal_len <= 0) {
            pos = next_pos;
            continue;
        }

        need = (size_t)off + 4 + (size_t)nal_len + 8;
        if (need > cap)
            return AVERROR(ENOMEM);

        /* Normalise to a 4-byte start code. */
        dst[off + 0] = 0;
        dst[off + 1] = 0;
        dst[off + 2] = 0;
        dst[off + 3] = 1;
        out_nal = dst + off + 4;

        if ((src[nal_start] & 0x1F) == 7) {
            written = videodec2_fix_sps(src + nal_start, nal_len,
                                        out_nal, (int)(cap - off - 4));
            if (written < 0) {
                memcpy(out_nal, src + nal_start, nal_len);
                written = nal_len;
            } else if (!s->sps_logged) {
                av_log(NULL, AV_LOG_VERBOSE,
                       "videodec2: SPS %d -> %d bytes (num_ref_frames=1)\n",
                       nal_len, written);
                s->sps_logged = 1;
                s->sps_patched = 1;
            }
        } else {
            memcpy(out_nal, src + nal_start, nal_len);
            written = nal_len;
        }

        off += 4 + written;
        pos = next_pos;
    }

    if (!found) {
        /* Not annex-B (should not happen after the mp4toannexb bsf). */
        if ((size_t)len > cap)
            return AVERROR(ENOMEM);
        memcpy(dst, src, len);
        return len;
    }
    return off;
}

/* -------------------------------------------------------------------------
 * Lifetime.
 * ---------------------------------------------------------------------- */

static void videodec2_free_memories(Videodec2DecodeContext *s)
{
    for (int i = 0; i < ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH; i++)
        videodec2_dmem_free(&s->au_mem[i]);
    s->au_next = 0;
    s->au_count = 0;

    videodec2_dmem_free(&s->fb_mem);
    videodec2_dmem_free(&s->cpu_mem);
    videodec2_dmem_free(&s->gpu_mem);
    videodec2_dmem_free(&s->cpugpu_mem);

    s->fb_stride = 0;
    s->fb_count  = 0;
    s->fb_next   = 0;
}

static av_cold int videodec2_init_decoder(AVCodecContext *avctx)
{
    Videodec2DecodeContext *s = avctx->priv_data;
    OrbisVideodec2DecoderMemoryInfo dm;
    OrbisVideodec2DecoderConfigInfo dc;
    size_t align, stride;
    int ret, count;

    if (avctx->codec_id != AV_CODEC_ID_H264) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: only H.264 is supported.\n");
        return AVERROR(EINVAL);
    }

    avctx->pix_fmt = AV_PIX_FMT_NV12;
    avctx->hwaccel = NULL;

    if (avctx->coded_width <= 0 || avctx->coded_height <= 0) {
        av_log(avctx, AV_LOG_ERROR,
               "videodec2: dimensions must be known at init time "
               "(%dx%d).\n", avctx->coded_width, avctx->coded_height);
        return AVERROR_INVALIDDATA;
    }

    s->width  = avctx->coded_width;
    s->height = avctx->coded_height;

    ret = videodec2_load(&s->api);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "videodec2: failed to load libSceVideodec2 (0x%08x).\n",
               (unsigned)ret);
        return AVERROR_EXTERNAL;
    }
    s->loaded = 1;

    ret = videodec2_ensure_compute_queue(&s->api);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "videodec2: failed to create compute queue (0x%08x).\n",
               (unsigned)ret);
        ret = AVERROR_EXTERNAL;
        goto fail;
    }

    videodec2_fill_decoder_config_ex(&dc, s->api.queue, s->width, s->height,
                                     s->pipeline_depth, s->thread_prio);

    memset(&dm, 0, sizeof(dm));
    dm.thisSize = sizeof(dm);
    ret = s->api.QueryDecoderMemoryInfo(&dc, &dm);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "videodec2: QueryDecoderMemoryInfo failed (0x%08x).\n",
               (unsigned)ret);
        goto fail;
    }

    av_log(avctx, AV_LOG_VERBOSE,
           "videodec2: mem cpu=%"PRIu64" gpu=%"PRIu64" cpuGpu=%"PRIu64
           " fb=%"PRIu64" align=%u (%dx%d depth=%d prio=%d).\n",
           dm.cpuMemorySize, dm.gpuMemorySize, dm.cpuGpuMemorySize,
           dm.maxFrameBufferSize, dm.frameBufferAlignment,
           s->width, s->height, s->pipeline_depth, s->thread_prio);

    /*
     * cpuMemory / cpuGpuMemory are the decoder's CPU working areas (entropy
     * parsing, DPB) and must be ONION; on WC_GARLIC every access is uncached,
     * Decode() takes ~150ms per frame and the pipeline overflows.  gpuMemory
     * stays GARLIC so the GPU sees write-combined memory.
     */
    if (dm.cpuMemorySize) {
        ret = videodec2_dmem_alloc(&s->cpu_mem, (size_t)dm.cpuMemorySize,
                                   ORBIS_VIDEODEC2_DMEM_TYPE_ONION);
        if (ret < 0)
            goto fail;
        dm.cpuMemory = s->cpu_mem.ptr;
    }
    if (dm.gpuMemorySize) {
        ret = videodec2_dmem_alloc(&s->gpu_mem, (size_t)dm.gpuMemorySize,
                                   ORBIS_VIDEODEC2_DMEM_TYPE_GARLIC);
        if (ret < 0)
            goto fail;
        dm.gpuMemory = s->gpu_mem.ptr;
    }
    if (dm.cpuGpuMemorySize) {
        ret = videodec2_dmem_alloc(&s->cpugpu_mem, (size_t)dm.cpuGpuMemorySize,
                                   ORBIS_VIDEODEC2_DMEM_TYPE_ONION);
        if (ret < 0)
            goto fail;
        dm.cpuGpuMemory = s->cpugpu_mem.ptr;
    }

    /* Framebuffers: depth + slack, rotated per Decode(). Written by the GPU,
     * so GARLIC. */
    if (dm.maxFrameBufferSize) {
        align = dm.frameBufferAlignment ? dm.frameBufferAlignment
                                        : ORBIS_VIDEODEC2_DMEM_ALIGN;
        if (align < ORBIS_VIDEODEC2_DMEM_ALIGN)
            align = ORBIS_VIDEODEC2_DMEM_ALIGN;
        if (align > (1u << 24)) {
            ret = AVERROR_INVALIDDATA;
            goto fail;
        }
        stride = FFALIGN((size_t)dm.maxFrameBufferSize, align);

        count = s->pipeline_depth + VD2_FB_SLACK;
        if (count > ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH + VD2_FB_SLACK)
            count = ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH + VD2_FB_SLACK;

        if (stride > SIZE_MAX / (size_t)count) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        s->fb_stride = stride;
        s->fb_count  = count;
        s->fb_next   = 0;

        ret = videodec2_dmem_alloc(&s->fb_mem, stride * (size_t)count,
                                   ORBIS_VIDEODEC2_DMEM_TYPE_GARLIC);
        if (ret < 0)
            goto fail;
    }

    /* AU buffers: one per rotated slot, ONION because the Vdec CPU worker
     * reads them during entropy parsing. */
    s->au_count = s->fb_count > 0 ? s->fb_count : 2;
    if (s->au_count > ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH)
        s->au_count = ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH;
    s->au_next = 0;
    for (int i = 0; i < s->au_count; i++) {
        ret = videodec2_dmem_alloc(&s->au_mem[i], VD2_AU_INITIAL_SIZE,
                                   ORBIS_VIDEODEC2_DMEM_TYPE_ONION);
        if (ret < 0)
            goto fail;
    }

    av_log(avctx, AV_LOG_VERBOSE,
           "videodec2: framebuf stride=%"PRIu64" count=%d au=%d\n",
           (uint64_t)s->fb_stride, s->fb_count, s->au_count);

    ret = s->api.CreateDecoder(&dc, &dm, &s->dec);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: CreateDecoder failed (0x%08x).\n",
               (unsigned)ret);
        ret = AVERROR_EXTERNAL;
        goto fail;
    }

    return 0;

fail:
    videodec2_free_memories(s);
    if (s->dec && s->api.DeleteDecoder) {
        s->api.DeleteDecoder(s->dec);
        s->dec = NULL;
    }
    if (s->loaded) {
        videodec2_unload(&s->api);
        s->loaded = 0;
    }
    return ret;
}

static av_cold int videodec2_close_decoder(AVCodecContext *avctx)
{
    Videodec2DecodeContext *s = avctx->priv_data;

    if (s->dec && s->api.DeleteDecoder)
        s->api.DeleteDecoder(s->dec);
    s->dec = NULL;

    videodec2_free_memories(s);

    if (s->loaded) {
        videodec2_unload(&s->api);
        s->loaded = 0;
    }
    return 0;
}

/* Grow an AU slot if the access unit does not fit. */
static int videodec2_au_reserve(Videodec2DecodeContext *s, int idx, size_t need)
{
    Videodec2Dmem *dm = &s->au_mem[idx];

    if (need <= dm->size)
        return 0;

    /* Overshoot so it converges to the peak instead of reallocating on every
     * rising frame (bitrate and IDRs swing frame to frame). */
    if (need > SIZE_MAX / 2)
        return AVERROR(ENOMEM);
    need += need / 2;
    need = FFALIGN(need, ORBIS_VIDEODEC2_DMEM_ALIGN);

    videodec2_dmem_free(dm);
    return videodec2_dmem_alloc(dm, need, ORBIS_VIDEODEC2_DMEM_TYPE_ONION);
}

/* -------------------------------------------------------------------------
 * Decode.
 * ---------------------------------------------------------------------- */

static int videodec2_decode(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame, AVPacket *pkt)
{
    Videodec2DecodeContext *s = avctx->priv_data;
    OrbisVideodec2InputData in;
    OrbisVideodec2FrameBuffer fb;
    OrbisVideodec2OutputInfo out;
    int idx, ret, au_len, pitch, w, h, disp_w, disp_h;
    uint8_t *au;

    *got_frame = 0;

    if (pkt->size <= 0)
        return 0;

    if (!s->dec)
        return AVERROR_EXTERNAL;

    if (!s->fb_mem.ptr || s->fb_count <= 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: no framebuffer allocated.\n");
        return AVERROR_EXTERNAL;
    }

    idx = s->au_next;
    ret = videodec2_au_reserve(s, idx, (size_t)pkt->size + 512);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: AU allocation failed.\n");
        return ret;
    }
    au = s->au_mem[idx].ptr;
    s->au_next = (s->au_next + 1) % s->au_count;

    au_len = videodec2_build_au(s, au, s->au_mem[idx].size, pkt->data, pkt->size);
    if (au_len < 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: AU assembly failed.\n");
        return au_len;
    }

    if (!s->logged_au) {
        av_log(avctx, AV_LOG_VERBOSE, "videodec2: first AU %d bytes.\n", au_len);
        s->logged_au = 1;
    }

    memset(&in, 0, sizeof(in));
    in.thisSize = sizeof(in);
    in.auData   = au;
    in.auSize   = (uint64_t)au_len;
    in.ptsData  = pkt->pts;
    in.dtsData  = pkt->dts;

    memset(&fb, 0, sizeof(fb));
    fb.thisSize        = sizeof(fb);
    fb.frameBuffer     = (uint8_t *)s->fb_mem.ptr +
                         (size_t)s->fb_next * s->fb_stride;
    fb.frameBufferSize = s->fb_stride;
    s->fb_next = (s->fb_next + 1) % s->fb_count;

    memset(&out, 0, sizeof(out));
    out.thisSize = sizeof(out);

    ret = s->api.Decode(s->dec, &in, &fb, &out);
    if (ret < 0) {
        if (!s->logged_decode_err) {
            av_log(avctx, AV_LOG_ERROR, "videodec2: Decode failed (0x%08x).\n",
                   (unsigned)ret);
            s->logged_decode_err = 1;
        }
        /* A hard Decode() failure leaves the DPB unusable. */
        if (s->api.Reset)
            s->api.Reset(s->dec);
        s->err_streak = 0;
        return 0;
    }
    s->logged_decode_err = 0;

    if (!out.isValid || !out.frameBuffer) {
        if (out.isErrorFrame && ++s->err_streak >= VD2_ERR_RESET_THRESHOLD) {
            av_log(avctx, AV_LOG_WARNING,
                   "videodec2: %d error frames in a row; resetting.\n",
                   s->err_streak);
            if (s->api.Reset)
                s->api.Reset(s->dec);
            s->err_streak = 0;
        }
        return 0;
    }
    s->err_streak = 0;

    pitch = out.framePitchInBytes ? (int)out.framePitchInBytes
                                  : (int)out.framePitch;
    w     = out.frameWidth  ? (int)out.frameWidth  : s->width;
    h     = out.frameHeight ? (int)out.frameHeight : s->height;

    /* Crop the macroblock padding (1080 -> 1088) for presentation.  The
     * padded height is still needed to locate the UV plane. */
    disp_w = (s->width  > 0 && w > s->width)  ? s->width  : w;
    disp_h = (s->height > 0 && h > s->height) ? s->height : h;

    if (pitch <= 0 || w <= 0 || h <= 0) {
        av_log(avctx, AV_LOG_ERROR,
               "videodec2: bogus output geometry %dx%d pitch=%d.\n",
               w, h, pitch);
        return 0;
    }
    if (disp_w > pitch)
        disp_w = pitch;

    ret = ff_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    {
        const uint8_t *src = (const uint8_t *)out.frameBuffer;
        int copy_w = FFMIN(disp_w, frame->linesize[0]);

        for (int y = 0; y < disp_h; y++)
            memcpy(frame->data[0] + (size_t)y * frame->linesize[0],
                   src + (size_t)y * pitch, copy_w);

        {
            const uint8_t *uv = src + (size_t)pitch * h;
            int uv_w = FFMIN(disp_w, frame->linesize[1]);
            int uv_h = (disp_h + 1) / 2;

            for (int y = 0; y < uv_h; y++)
                memcpy(frame->data[1] + (size_t)y * frame->linesize[1],
                       uv + (size_t)y * pitch, uv_w);
        }
    }

    frame->pts     = pkt->pts;
    frame->pkt_dts = pkt->dts;

    if (!s->logged_sample) {
        av_log(avctx, AV_LOG_VERBOSE,
               "videodec2: out fmt=0x%x %ux%u pitchB=%u crop=%dx%d.\n",
               out.frameFormat, out.frameWidth, out.frameHeight,
               out.framePitchInBytes, disp_w, disp_h);
        s->logged_sample = 1;
    }

    *got_frame = 1;
    return pkt->size;
}

static void videodec2_flush(AVCodecContext *avctx)
{
    Videodec2DecodeContext *s = avctx->priv_data;

    if (s->dec && s->api.Reset)
        s->api.Reset(s->dec);
    s->fb_next    = 0;
    s->au_next    = 0;
    s->err_streak = 0;
}

#define OFFSET(x) offsetof(Videodec2DecodeContext, x)
#define VD2VD AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM
static const AVOption videodec2_options[] = {
    { "pipeline_depth", "Videodec2 decode pipeline depth (1-4)", OFFSET(pipeline_depth),
      AV_OPT_TYPE_INT, { .i64 = 2 }, 1, ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH, VD2VD },
    { "thread_prio", "Vdec CPU worker thread priority", OFFSET(thread_prio),
      AV_OPT_TYPE_INT, { .i64 = ORBIS_VIDEODEC2_THREAD_PRIO_DEFAULT }, 16, 4095, VD2VD },
    { NULL }
};

static const AVClass videodec2_dec_class = {
    .class_name = "videodec2_dec",
    .item_name  = av_default_item_name,
    .option     = videodec2_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_h264_videodec2_decoder = {
    .p.name         = "h264_videodec2",
    CODEC_LONG_NAME("H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10 (PS4 Videodec2)"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_H264,
    .priv_data_size = sizeof(Videodec2DecodeContext),
    .init           = videodec2_init_decoder,
    .close          = videodec2_close_decoder,
    FF_CODEC_DECODE_CB(videodec2_decode),
    .flush          = videodec2_flush,
    .p.priv_class   = &videodec2_dec_class,
    .p.capabilities = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING |
                      AV_CODEC_CAP_HARDWARE,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP |
                      FF_CODEC_CAP_NOT_INIT_THREADSAFE,
    .bsfs           = "h264_mp4toannexb",
    .p.wrapper_name = "videodec2",
};
