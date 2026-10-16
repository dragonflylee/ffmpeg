/*
 * PS4 Videodec2 (libSceVideodec2) type definitions.
 *
 * The structures and constants below are a faithful port of the values
 * validated on console by the Moonlight-ps4 project
 * (src/orbis/videodec2.h and src/orbis/video_out_c.h).  The Sony headers
 * shipped with the OpenOrbis toolchain only declare the entry points as
 * "void sceX();", so every type used here has to be carried locally.
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

#ifndef AVCODEC_VIDEODEC2DEC_H
#define AVCODEC_VIDEODEC2DEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Codec id understood by the decoder config. Only AVC was validated. */
#define ORBIS_VIDEODEC2_CODEC_AVC 1

/*
 * Validated on console:
 *   resourceType=0           -> 0x811D0203
 *   without profile/level    -> 0x811D0205
 *   cpuThreadPriority=16     -> 0x811D0208
 */
#define ORBIS_VIDEODEC2_RESOURCE_TYPE_EMBEDDED 1u
#define ORBIS_VIDEODEC2_PROFILE_HIGH           100u
#define ORBIS_VIDEODEC2_LEVEL_51               51u
#define ORBIS_VIDEODEC2_THREAD_PRIO_DEFAULT    700
#define ORBIS_VIDEODEC2_AFFINITY_ALL           0x3Full

/*
 * Direct memory (libkernel) constants.  These are the type ids accepted by
 * sceKernelAllocateDirectMemory(), i.e. the ORBIS_KERNEL_* enumerators and
 * not something invented per-call.  Getting one wrong is silent
 * (checkMemoryType is disabled in the decoder config) and the failure only
 * shows up as a crash inside the Vdec worker thread.
 */
#define ORBIS_VIDEODEC2_DMEM_ALIGN       0x4000u
#define ORBIS_VIDEODEC2_DMEM_TYPE_ONION  0   /* ORBIS_KERNEL_WB_ONION  */
#define ORBIS_VIDEODEC2_DMEM_TYPE_GARLIC 3   /* ORBIS_KERNEL_WC_GARLIC */
#define ORBIS_VIDEODEC2_DMEM_PROT_RW     0x33u

/*
 * Number of frames Videodec2 may have in flight. Each one needs its own
 * framebuffer, so this also bounds how many the caller has to allocate.
 */
#define ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH 4

typedef struct OrbisVideodec2ComputeConfigInfo {
    uint64_t thisSize;
    uint16_t computePipeId;
    uint16_t computeQueueId;
    bool     checkMemoryType;
    uint8_t  reserved0;
    uint16_t reserved1;
} OrbisVideodec2ComputeConfigInfo;

typedef struct OrbisVideodec2ComputeMemoryInfo {
    uint64_t thisSize;
    uint64_t cpuGpuMemorySize;
    void    *cpuGpuMemory;
} OrbisVideodec2ComputeMemoryInfo;

typedef struct OrbisVideodec2DecoderConfigInfo {
    uint64_t thisSize;
    uint32_t resourceType;
    uint32_t codecType;
    uint32_t profile;
    uint32_t maxLevel;
    int32_t  maxFrameWidth;
    int32_t  maxFrameHeight;
    int32_t  maxDpbFrameCount;
    uint32_t decodePipelineDepth;
    void    *computeQueue;
    uint64_t cpuAffinityMask;
    int32_t  cpuThreadPriority;
    bool     optimizeProgressiveVideo;
    bool     checkMemoryType;
    uint8_t  reserved0;
    uint8_t  reserved1;
    void    *extraConfigInfo;
} OrbisVideodec2DecoderConfigInfo;

typedef struct OrbisVideodec2DecoderMemoryInfo {
    uint64_t thisSize;
    uint64_t cpuMemorySize;
    void    *cpuMemory;
    uint64_t gpuMemorySize;
    void    *gpuMemory;
    uint64_t cpuGpuMemorySize;
    void    *cpuGpuMemory;
    uint64_t maxFrameBufferSize;
    uint32_t frameBufferAlignment;
    uint32_t reserved0;
} OrbisVideodec2DecoderMemoryInfo;

typedef struct OrbisVideodec2InputData {
    uint64_t thisSize;
    void    *auData;
    uint64_t auSize;
    uint64_t ptsData;
    uint64_t dtsData;
    uint64_t attachedData;
} OrbisVideodec2InputData;

typedef struct OrbisVideodec2OutputInfo {
    uint64_t thisSize;
    bool     isValid;
    bool     isErrorFrame;
    uint8_t  pictureCount;
    uint32_t codecType;
    uint32_t frameWidth;
    uint32_t framePitch;
    uint32_t frameHeight;
    void    *frameBuffer;
    uint64_t frameBufferSize;
    uint32_t frameFormat;
    uint32_t framePitchInBytes;
} OrbisVideodec2OutputInfo;

typedef struct OrbisVideodec2FrameBuffer {
    uint64_t thisSize;
    void    *frameBuffer;
    uint64_t frameBufferSize;
    bool     isAccepted;
} OrbisVideodec2FrameBuffer;

typedef void *OrbisVideodec2Decoder;
typedef void *OrbisVideodec2ComputeQueue;

static inline void videodec2_fill_decoder_config_ex(OrbisVideodec2DecoderConfigInfo *dc,
                                                    OrbisVideodec2ComputeQueue q,
                                                    int w, int h,
                                                    int pipelineDepth, int cpuPriority)
{
    if (pipelineDepth < 1)
        pipelineDepth = 1;
    if (pipelineDepth > ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH)
        pipelineDepth = ORBIS_VIDEODEC2_MAX_PIPELINE_DEPTH;
    if (cpuPriority <= 0)
        cpuPriority = ORBIS_VIDEODEC2_THREAD_PRIO_DEFAULT;

    memset(dc, 0, sizeof(*dc));
    dc->thisSize                 = sizeof(*dc);
    dc->resourceType             = ORBIS_VIDEODEC2_RESOURCE_TYPE_EMBEDDED;
    dc->codecType                = ORBIS_VIDEODEC2_CODEC_AVC;
    dc->profile                  = ORBIS_VIDEODEC2_PROFILE_HIGH;
    dc->maxLevel                 = ORBIS_VIDEODEC2_LEVEL_51;
    /* Width is passed unchanged; only the height needs macroblock alignment
     * (1080 -> 1088).  Aligning the width perturbs the reported visible
     * size and the output pitch. */
    dc->maxFrameWidth            = w;
    /* 1088: macroblock-align; 1080 in config -> slow Decode / rare paths. */
    dc->maxFrameHeight           = (h + 15) & ~15;
    /* dpb=1 + stream with refs -> Decode ~150ms and IDR storm. 4 = typical NVENC. */
    dc->maxDpbFrameCount         = 4;
    /* depth=1 serialises submit->wait against the compute queue every frame. */
    dc->decodePipelineDepth      = (uint32_t)pipelineDepth;
    dc->computeQueue             = q;
    dc->cpuAffinityMask          = ORBIS_VIDEODEC2_AFFINITY_ALL;
    dc->cpuThreadPriority        = cpuPriority;
    dc->optimizeProgressiveVideo = true;
    /* checkMemoryType=0: validated.  With 1 + AU Onion the Vdec worker
     * SIGSEGVs, and it would otherwise reject the CPU working areas. */
    dc->checkMemoryType          = false;
}

static inline void videodec2_fill_decoder_config(OrbisVideodec2DecoderConfigInfo *dc,
                                                 OrbisVideodec2ComputeQueue q,
                                                 int w, int h)
{
    videodec2_fill_decoder_config_ex(dc, q, w, h, 1,
                                     ORBIS_VIDEODEC2_THREAD_PRIO_DEFAULT);
}

typedef int32_t (*sceVideodec2QueryComputeMemoryInfo_t)(OrbisVideodec2ComputeMemoryInfo *);
typedef int32_t (*sceVideodec2AllocateComputeQueue_t)(const OrbisVideodec2ComputeConfigInfo *,
                                                      const OrbisVideodec2ComputeMemoryInfo *,
                                                      OrbisVideodec2ComputeQueue *);
typedef int32_t (*sceVideodec2ReleaseComputeQueue_t)(OrbisVideodec2ComputeQueue);
typedef int32_t (*sceVideodec2QueryDecoderMemoryInfo_t)(const OrbisVideodec2DecoderConfigInfo *,
                                                        OrbisVideodec2DecoderMemoryInfo *);
typedef int32_t (*sceVideodec2CreateDecoder_t)(const OrbisVideodec2DecoderConfigInfo *,
                                                const OrbisVideodec2DecoderMemoryInfo *,
                                                OrbisVideodec2Decoder *);
typedef int32_t (*sceVideodec2DeleteDecoder_t)(OrbisVideodec2Decoder);
typedef int32_t (*sceVideodec2Decode_t)(OrbisVideodec2Decoder,
                                        const OrbisVideodec2InputData *,
                                        OrbisVideodec2FrameBuffer *,
                                        OrbisVideodec2OutputInfo *);
typedef int32_t (*sceVideodec2Flush_t)(OrbisVideodec2Decoder,
                                       OrbisVideodec2FrameBuffer *,
                                       OrbisVideodec2OutputInfo *);
typedef int32_t (*sceVideodec2Reset_t)(OrbisVideodec2Decoder);

/*
 * A direct-memory allocation.  The reservation handle ("off") is what gets
 * passed back to sceKernelReleaseDirectMemory(), so it is kept next to the
 * CPU mapping instead of being thrown away.
 */
typedef struct Videodec2Dmem {
    void   *ptr;   /* CPU mapping, dereferenced by us and by the firmware */
    int64_t off;   /* physical offset handle returned by Allocate */
    size_t  size;  /* mapped/released size, already aligned */
} Videodec2Dmem;

typedef struct videodec2_api_t {
    /*
     * Handle returned by sceKernelLoadStartModule(); -1 when no module is
     * loaded.  This is an int, NOT the void* that POSIX dlopen() returns:
     * PS4 system sprx files cannot be opened with dlopen(), and the value
     * is consumed by sceKernelDlsym().
     */
    int module;
    OrbisVideodec2ComputeQueue queue;
    /* Backing store of the compute queue; must outlive ReleaseComputeQueue. */
    Videodec2Dmem queue_mem;
    sceVideodec2QueryComputeMemoryInfo_t  QueryComputeMemoryInfo;
    sceVideodec2AllocateComputeQueue_t    AllocateComputeQueue;
    sceVideodec2ReleaseComputeQueue_t     ReleaseComputeQueue;
    sceVideodec2QueryDecoderMemoryInfo_t  QueryDecoderMemoryInfo;
    sceVideodec2CreateDecoder_t           CreateDecoder;
    sceVideodec2DeleteDecoder_t           DeleteDecoder;
    sceVideodec2Decode_t                  Decode;
    sceVideodec2Flush_t                   Flush;
    sceVideodec2Reset_t                   Reset;
} videodec2_api_t;

/*
 * Platform glue, implemented in videodec2dec_orbis.c.  Returns 0 on
 * success, a raw 0x8xxxxxxx SCE error otherwise.
 */
int  videodec2_load(videodec2_api_t *api);
void videodec2_unload(videodec2_api_t *api);
int  videodec2_ensure_compute_queue(videodec2_api_t *api);

/*
 * Direct-memory helpers.  On PS4 these wrap sceKernelAllocateDirectMemory()
 * / MapDirectMemory() / ReleaseDirectMemory(); on a host build they fail
 * with AVERROR(ENOSYS) instead of handing out normal heap memory that the
 * firmware would then read through a physical address.
 */
int  videodec2_dmem_alloc(Videodec2Dmem *dm, size_t size, int32_t mem_type);
void videodec2_dmem_free(Videodec2Dmem *dm);

#endif /* AVCODEC_VIDEODEC2DEC_H */
