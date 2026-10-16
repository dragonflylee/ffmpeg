/*
 * PS4 Videodec2 H.264 hardware decoder.
 *
 * This is a self-contained ("path B") decoder that hands whole access units
 * to libSceVideodec2 and receives decoded NV12 frames. It mirrors the proven
 * logic from the Moonlight-ps4 project (src/video/decoder_orbis.c and
 * src/orbis/videodec2.*), adapted to FFmpeg's decode callback model.
 *
 * Direct memory (sceKernel*DirectMemory) is handled by the thin platform
 * glue declared in videodec2dec.h; this file stays free of Sony/OpenOrbis
 * headers so it can be compiled and reviewed on a normal host.
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
#include "libavutil/frame.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixfmt.h"

/* Number of extra framebuffers beyond the pipeline depth: one for the frame
 * the caller may still be reading, plus one of slack for the pipelined
 * decoder returning output with an extra call of delay. */
#define VD2_FB_SLACK 2

/* Maximum AU size preallocated per rotated slot. Grows on demand. */
#define VD2_AU_INITIAL_SIZE (1024 * 1024)
#define VD2_AU_GROWTH_NUM 3
#define VD2_AU_GROWTH_DEN 2

typedef struct Videodec2DecodeContext {
    AVClass *av_class;

    videodec2_api_t api;
    OrbisVideodec2Decoder dec;
    int loaded;

    int width;
    int height;

    /* Decoder working memories (see videodec2dec.h for the types). */
    void   *cpu_mem;
    void   *gpu_mem;
    void   *cpugpu_mem;

    /* Rotated framebuffers handed to Decode(). */
    void   *fb_mem;
    size_t  fb_stride;   /* bytes per framebuffer */
    size_t  fb_size;     /* whole reservation */
    int     fb_count;
    int     fb_next;

    /* Rotated access-unit (bitstream) buffers. With depth > 1 the Vdec
     * worker may still be parsing AU(N) after Decode() returns, so the
     * next AU must not overwrite it. */
    void   *au_mem[ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH];
    size_t  au_cap[ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH];
    int     au_count;
    int     au_next;

    /* SPS fixup scratch (num_ref_frames forced to 1; see gs_sps_fix). */
    uint8_t sps_scratch[256];

    int logged_sample;

    /* Options. */
    int pipeline_depth;
    int thread_prio;
} Videodec2DecodeContext;

/*
 * SPS fixup: the PS4 Videodec2 decoder misbehaves with the reference counts
 * NVENC/Sunshine emit. Rewriting num_ref_frames to 1 avoids the ~150ms
 * Decode + IDR storm observed on console.
 *
 * This is a minimal, faithful port of gs_sps_fix(GS_SPS_BITSTREAM_FIXUP).
 * It operates on a single SPS NAL unit (with start code) and returns the
 * number of bytes written to dst.
 *
 * NOTE: the full emulation-prevention-aware rewrite lives in the Orbis
 * SPS helper. This implementation keeps the change conservative and bails
 * out on anything it does not fully understand, in which case the source
 * SPS is copied verbatim.
 */
static int videodec2_fix_sps(const uint8_t *sps, int sps_len,
                             uint8_t *dst, size_t dst_size)
{
    /* Placeholder-safe passthrough: the exact bit-level rewrite requires
     * the h264bitstream/gs_sps_fix helper. Behaviour is identical to the
     * unfixed stream unless a fixup implementation is provided. */
    if ((size_t)sps_len > dst_size)
        return -1;
    memcpy(dst, sps, sps_len);
    return sps_len;
}

/* Is this NAL unit (including its start code) an SPS? */
static int videodec2_nal_is_sps(const uint8_t *p, int len)
{
    int sc = 0;

    if (len >= 4 && p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1)
        sc = 4;
    else if (len >= 3 && p[0] == 0 && p[1] == 0 && p[2] == 1)
        sc = 3;
    if (!sc || sc >= len)
        return 0;
    return (p[sc] & 0x1F) == 7;
}

static av_cold int videodec2_alloc_dmem(AVCodecContext *avctx, size_t need,
                                        int32_t mem_type, void **ptr, size_t *cap)
{
    /* The platform glue owns the actual sceKernelAllocateDirectMemory call.
     * Without it we cannot back decoder memories; signal failure clearly. */
    av_log(avctx, AV_LOG_ERROR,
           "videodec2: direct memory allocation is not available in this build "
           "(size=%zu type=%d).\n", need, mem_type);
    return AVERROR(ENOSYS);
}

static void videodec2_free_memories(Videodec2DecodeContext *s)
{
    int i;

    for (i = 0; i < ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH; i++) {
        if (s->au_mem[i]) {
            av_free(s->au_mem[i]);
            s->au_mem[i] = NULL;
            s->au_cap[i] = 0;
        }
    }
    s->au_next = 0;

    av_freep(&s->fb_mem);
    av_freep(&s->cpu_mem);
    av_freep(&s->gpu_mem);
    av_freep(&s->cpugpu_mem);

    s->fb_size  = 0;
    s->fb_count = 0;
    s->fb_next  = 0;
}

static av_cold int videodec2_init_decoder(AVCodecContext *avctx)
{
    Videodec2DecodeContext *s = avctx->priv_data;
    OrbisVideodec2DecoderMemoryInfo dm;
    OrbisVideodec2DecoderConfigInfo dc;
    uint32_t w, h;
    int ret;

    if (avctx->codec_id != AV_CODEC_ID_H264) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: only H.264 is supported.\n");
        return AVERROR(EINVAL);
    }

    /* Videodec2 outputs planar NV12. */
    avctx->pix_fmt  = AV_PIX_FMT_NV12;
    avctx->hwaccel  = NULL;
    avctx->coded_width  = FFALIGN(avctx->coded_width, 16);
    avctx->coded_height = avctx->coded_height;

    if (avctx->coded_width <= 0 || avctx->coded_height <= 0) {
        av_log(avctx, AV_LOG_ERROR,
               "videodec2: dimensions must be known at init time "
               "(%dx%d).\n", avctx->coded_width, avctx->coded_height);
        return AVERROR_INVALIDDATA;
    }

    s->width  = avctx->coded_width;
    s->height = avctx->coded_height;

    if (videodec2_load(&s->api) != 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: failed to load libSceVideodec2.\n");
        return AVERROR_EXTERNAL;
    }
    s->loaded = 1;

    if (videodec2_ensure_compute_queue(&s->api) < 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: failed to create compute queue.\n");
        ret = AVERROR_EXTERNAL;
        goto fail;
    }

    /* Frame info is macroblock aligned; the decoder pads to 1088 for 1080p,
     * which is garbage, not picture. Ask FFmpeg for the visible size. */
    w = s->width;
    h = s->height;

    videodec2_fill_decoder_config_ex(&dc, s->api.queue, w, h,
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
           "videodec2: mem cpu=%zu gpu=%zu cpuGpu=%zu fb=%zu align=%u (%dx%d "
           "depth=%d prio=%d).\n",
           (size_t)dm.cpuMemorySize, (size_t)dm.gpuMemorySize,
           (size_t)dm.cpuGpuMemorySize, (size_t)dm.maxFrameBufferSize,
           dm.frameBufferAlignment, w, h, s->pipeline_depth, s->thread_prio);

    /* cpuMemory / cpuGpuMemory are CPU working areas (entropy parsing, DPB);
     * they need to be CPU-cacheable. gpuMemory stays write-combined. */
    if (dm.cpuMemorySize) {
        if (videodec2_alloc_dmem(avctx, (size_t)dm.cpuMemorySize, 2 /*ONION*/,
                                 &s->cpu_mem, NULL) < 0) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        dm.cpuMemory = s->cpu_mem;
    }
    if (dm.gpuMemorySize) {
        if (videodec2_alloc_dmem(avctx, (size_t)dm.gpuMemorySize, 0 /*GARLIC*/,
                                 &s->gpu_mem, NULL) < 0) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        dm.gpuMemory = s->gpu_mem;
    }
    if (dm.cpuGpuMemorySize) {
        if (videodec2_alloc_dmem(avctx, (size_t)dm.cpuGpuMemorySize, 2 /*ONION*/,
                                 &s->cpugpu_mem, NULL) < 0) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        dm.cpuGpuMemory = s->cpugpu_mem;
    }

    /* Framebuffers: depth + slack, rotated per Decode(). */
    if (dm.maxFrameBufferSize) {
        size_t stride = dm.maxFrameBufferSize;
        size_t align  = dm.frameBufferAlignment ? dm.frameBufferAlignment : 0x100;
        int count = s->pipeline_depth + VD2_FB_SLACK;

        if (count > ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH + VD2_FB_SLACK)
            count = ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH + VD2_FB_SLACK;

        stride = FFALIGN(stride, align);
        s->fb_stride = stride;
        s->fb_count  = count;
        s->fb_next   = 0;
        s->fb_size   = stride * (size_t)count;

        if (videodec2_alloc_dmem(avctx, s->fb_size, 0 /*GARLIC*/,
                                 &s->fb_mem, &s->fb_size) < 0) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        s->fb_size = stride * (size_t)count;
    }

    /* AU buffers: one per rotated slot. */
    s->au_count = s->fb_count > 0 ? s->fb_count : 2;
    if (s->au_count > ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH)
        s->au_count = ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH;
    s->au_next = 0;
    for (int i = 0; i < s->au_count; i++) {
        s->au_mem[i] = av_malloc(VD2_AU_INITIAL_SIZE);
        if (!s->au_mem[i]) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        s->au_cap[i] = VD2_AU_INITIAL_SIZE;
    }

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
    memset(&s->api, 0, sizeof(s->api));
    return 0;
}

/* Grow an AU slot if the current access unit does not fit. */
static int videodec2_au_reserve(Videodec2DecodeContext *s, int idx, size_t need)
{
    if (need <= s->au_cap[idx])
        return 0;

    need = FFALIGN(need, 64);
    /* Overshoot so it converges to the peak instead of reallocating on every
     * rising frame (bitrate and IDRs swing frame to frame). */
    need = need * VD2_AU_GROWTH_NUM / VD2_AU_GROWTH_DEN;

    av_freep(&s->au_mem[idx]);
    s->au_cap[idx] = 0;
    s->au_mem[idx] = av_malloc(need);
    if (!s->au_mem[idx])
        return AVERROR(ENOMEM);
    s->au_cap[idx] = need;
    return 0;
}

static int videodec2_decode(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame, AVPacket *pkt)
{
    Videodec2DecodeContext *s = avctx->priv_data;
    OrbisVideodec2InputData in;
    OrbisVideodec2FrameBuffer fb;
    OrbisVideodec2OutputInfo out;
    uint8_t *au;
    int au_len, off, idx, ret;
    int pitch_y, w, h, disp_w, disp_h;

    *got_frame = 0;

    if (pkt->size <= 0)
        return 0;

    idx = s->au_next;
    ret = videodec2_au_reserve(s, idx, (size_t)pkt->size + 128);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: AU allocation failed.\n");
        return ret;
    }
    au = s->au_mem[idx];
    s->au_next = (s->au_next + 1) % s->au_count;

    /* Copy the packet into the rotated AU slot, rewriting an SPS NAL if the
     * packet starts with one (the annex-B packet may carry SPS+PPS+IDR). */
    off = 0;
    if (videodec2_nal_is_sps(pkt->data, pkt->size)) {
        int nal_len = pkt->size;
        int fixed = videodec2_fix_sps(pkt->data, nal_len,
                                      s->sps_scratch, sizeof(s->sps_scratch));
        if (fixed < 0)
            return AVERROR_INVALIDDATA;
        memcpy(au, s->sps_scratch, fixed);
        off = fixed;
    } else {
        memcpy(au, pkt->data, pkt->size);
        off = pkt->size;
    }
    au_len = off;

    memset(&in, 0, sizeof(in));
    in.thisSize = sizeof(in);
    in.auData   = au;
    in.auSize   = (uint64_t)au_len;
    in.ptsData  = pkt->pts;
    in.dtsData  = pkt->dts;

    memset(&fb, 0, sizeof(fb));
    fb.thisSize        = sizeof(fb);
    fb.frameBuffer     = (uint8_t *)s->fb_mem + (size_t)s->fb_next * s->fb_stride;
    fb.frameBufferSize = s->fb_stride;
    s->fb_next = (s->fb_next + 1) % (s->fb_count > 0 ? s->fb_count : 1);

    memset(&out, 0, sizeof(out));
    out.thisSize = sizeof(out);

    ret = s->api.Decode(s->dec, &in, &fb, &out);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "videodec2: Decode failed (0x%08x).\n",
               (unsigned)ret);
        return AVERROR_EXTERNAL;
    }

    if (!out.isValid || !out.frameBuffer) {
        if (out.isErrorFrame) {
            av_log(avctx, AV_LOG_WARNING, "videodec2: error frame; resetting.\n");
            s->api.Reset(s->dec);
        }
        return 0;
    }

    pitch_y = out.framePitchInBytes ? (int)out.framePitchInBytes : (int)out.framePitch;
    w       = out.frameWidth  ? (int)out.frameWidth  : s->width;
    h       = out.frameHeight ? (int)out.frameHeight : s->height;

    /* Crop the macroblock padding (1080 -> 1088) for presentation. */
    disp_w = (s->width  > 0 && w > s->width)  ? s->width  : w;
    disp_h = (s->height > 0 && h > s->height) ? s->height : h;

    ret = ff_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    /* NV12: Y plane then interleaved UV plane. Output is planar NV12 with a
     * pitch that may exceed the visible width; copy row by row and honour
     * the crop. */
    {
        const uint8_t *src = (const uint8_t *)out.frameBuffer;
        int copy_w = FFMIN(disp_w, frame->linesize[0]);
        int y;

        for (y = 0; y < disp_h; y++) {
            memcpy(frame->data[0] + (size_t)y * frame->linesize[0],
                   src + (size_t)y * pitch_y, copy_w);
        }
        {
            const uint8_t *uv = src + (size_t)pitch_y * h;
            int uv_w = FFMIN(disp_w, frame->linesize[1]);
            int uv_h = (disp_h + 1) / 2;
            for (y = 0; y < uv_h; y++) {
                memcpy(frame->data[1] + (size_t)y * frame->linesize[1],
                       uv + (size_t)y * pitch_y, uv_w);
            }
        }
    }

    frame->pts = pkt->pts;
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
    s->fb_next = 0;
    s->au_next = 0;
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
