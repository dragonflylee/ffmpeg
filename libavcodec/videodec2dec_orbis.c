/*
 * PS4 Videodec2 platform glue.
 *
 * This is the only file in the Videodec2 decoder that talks to the Sony /
 * OpenOrbis runtime: module loading (sceKernelLoadStartModule + Dlsym), the
 * compute queue, and sceKernel direct memory.  On a normal host build every
 * entry point reports failure so that the codec fails cleanly at
 * avcodec_open2() instead of misbehaving.
 *
 * The sequence and the memory types mirror the implementation validated on
 * console by the Moonlight-ps4 project (src/orbis/videodec2_loader.c):
 *
 *   LoadStartModule(libSceVdecCore / VdecSavc / VdecSavc2 / Vdecwrap)
 *   LoadStartModule(libSceVideodec2)
 *   Dlsym(each entry point)
 *   QueryComputeMemoryInfo -> AllocateDirectMemory(ONION) -> AllocateComputeQueue
 *   (decoder memories, see videodec2dec.c)
 *   CreateDecoder
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

#include "libavutil/error.h"
#include "libavutil/log.h"
#include "libavutil/macros.h"
#include "videodec2dec.h"

#ifdef __ORBIS__

#include <stdio.h>

#include <orbis/libkernel.h>

/* System modules the decoder depends on, in load order. */
static const char *const vd2_deps[] = {
    "/system/common/lib/libSceVdecCore.sprx",
    "/system/common/lib/libSceVdecSavc.sprx",
    "/system/common/lib/libSceVdecSavc2.sprx",
    "/system/common/lib/libSceVdecwrap.sprx",
};

int videodec2_dmem_alloc(Videodec2Dmem *dm, size_t size, int32_t mem_type)
{
    size_t alloc_size;
    int64_t off = 0;
    void *ptr = NULL;
    int32_t rc;

    if (!dm)
        return AVERROR(EINVAL);

    memset(dm, 0, sizeof(*dm));

    if (!size)
        return AVERROR(EINVAL);

    alloc_size = (size + ORBIS_VIDEODEC2_DMEM_ALIGN - 1) &
                 ~(size_t)(ORBIS_VIDEODEC2_DMEM_ALIGN - 1);

    rc = sceKernelAllocateDirectMemory(0, sceKernelGetDirectMemorySize(),
                                       alloc_size, ORBIS_VIDEODEC2_DMEM_ALIGN,
                                       mem_type, &off);
    if (rc < 0) {
        av_log(NULL, AV_LOG_ERROR,
               "videodec2: sceKernelAllocateDirectMemory(%zu, type=%d) "
               "failed (0x%08x).\n", alloc_size, mem_type, (unsigned)rc);
        return AVERROR(ENOMEM);
    }

    rc = sceKernelMapDirectMemory(&ptr, alloc_size, ORBIS_VIDEODEC2_DMEM_PROT_RW,
                                  0, off, ORBIS_VIDEODEC2_DMEM_ALIGN);
    if (rc < 0 || !ptr) {
        av_log(NULL, AV_LOG_ERROR,
               "videodec2: sceKernelMapDirectMemory(%zu, type=%d) failed "
               "(0x%08x).\n", alloc_size, mem_type, (unsigned)rc);
        sceKernelReleaseDirectMemory(off, alloc_size);
        return AVERROR(ENOMEM);
    }

    dm->ptr  = ptr;
    dm->off  = off;
    dm->size = alloc_size;
    return 0;
}

void videodec2_dmem_free(Videodec2Dmem *dm)
{
    if (!dm || !dm->ptr)
        return;
    sceKernelReleaseDirectMemory(dm->off, dm->size);
    dm->ptr  = NULL;
    dm->off  = 0;
    dm->size = 0;
}

static int vd2_load_module(const char *path)
{
    int rc = (int)sceKernelLoadStartModule(path, 0, NULL, 0, NULL, NULL);

    if (rc < 0)
        av_log(NULL, AV_LOG_VERBOSE,
               "videodec2: LoadStartModule(%s) => 0x%08x\n", path, (unsigned)rc);
    return rc;
}

static int vd2_dlsym(int mod, const char *name, void **out)
{
    int rc = sceKernelDlsym(mod, name, out);

    if (rc != 0 || !*out) {
        av_log(NULL, AV_LOG_ERROR, "videodec2: Dlsym(%s) => 0x%08x\n",
               name, (unsigned)rc);
        *out = NULL;
        return -1;
    }
    return 0;
}

int videodec2_load(videodec2_api_t *api)
{
    int mod;

    if (!api)
        return AVERROR(EINVAL);

    memset(api, 0, sizeof(*api));
    api->module = -1;

    /* The Vdec worker needs the Savc/Core modules present before the
     * Videodec2 module is started; a missing one is not fatal here, the
     * Dlsym on Videodec2 below is what ultimately decides. */
    for (size_t i = 0; i < FF_ARRAY_ELEMS(vd2_deps); i++)
        (void)vd2_load_module(vd2_deps[i]);

    mod = vd2_load_module("/system/common/lib/libSceVideodec2.sprx");
    if (mod < 0)
        mod = vd2_load_module("libSceVideodec2.sprx");
    if (mod < 0)
        return mod;

    api->module = mod;

#define VD2_SYMBOL(field, name)                                          \
    do {                                                                 \
        if (vd2_dlsym(mod, name, (void **)&api->field) < 0) {             \
            videodec2_unload(api);                                       \
            return AVERROR_EXTERNAL;                                     \
        }                                                                \
    } while (0)

    VD2_SYMBOL(QueryComputeMemoryInfo, "sceVideodec2QueryComputeMemoryInfo");
    VD2_SYMBOL(AllocateComputeQueue,   "sceVideodec2AllocateComputeQueue");
    VD2_SYMBOL(ReleaseComputeQueue,    "sceVideodec2ReleaseComputeQueue");
    VD2_SYMBOL(QueryDecoderMemoryInfo, "sceVideodec2QueryDecoderMemoryInfo");
    VD2_SYMBOL(CreateDecoder,          "sceVideodec2CreateDecoder");
    VD2_SYMBOL(DeleteDecoder,          "sceVideodec2DeleteDecoder");
    VD2_SYMBOL(Decode,                 "sceVideodec2Decode");
    VD2_SYMBOL(Flush,                  "sceVideodec2Flush");
    VD2_SYMBOL(Reset,                  "sceVideodec2Reset");

#undef VD2_SYMBOL

    return 0;
}

void videodec2_unload(videodec2_api_t *api)
{
    if (!api)
        return;

    /* The compute queue lives inside its ONION mapping: release the queue
     * before the memory it points into. */
    if (api->queue && api->ReleaseComputeQueue) {
        int32_t rc = api->ReleaseComputeQueue(api->queue);
        if (rc < 0)
            av_log(NULL, AV_LOG_WARNING,
                   "videodec2: ReleaseComputeQueue => 0x%08x\n", (unsigned)rc);
    }
    api->queue = NULL;
    videodec2_dmem_free(&api->queue_mem);

    memset(api, 0, sizeof(*api));
    api->module = -1;
}

int videodec2_ensure_compute_queue(videodec2_api_t *api)
{
    OrbisVideodec2ComputeConfigInfo cc;
    OrbisVideodec2ComputeMemoryInfo cm;
    /* Pipe/queue pairs that were observed to work; pipe/queue 0/0 is the
     * usual one and the rest are fallbacks when another component holds it. */
    static const struct { uint16_t pipe, queue; } try_list[] = {
        { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 },
        { 2, 0 }, { 3, 0 }, { 4, 0 },
    };
    int32_t rc;

    if (!api || api->module < 0)
        return AVERROR(EINVAL);
    if (api->queue)
        return 0;

    memset(&cm, 0, sizeof(cm));
    cm.thisSize = sizeof(cm);
    rc = api->QueryComputeMemoryInfo(&cm);
    if (rc < 0) {
        av_log(NULL, AV_LOG_ERROR,
               "videodec2: QueryComputeMemoryInfo failed (0x%08x).\n",
               (unsigned)rc);
        return rc;
    }

    /* The queue is created inside this mapping and both the CPU and the GPU
     * touch it, so it has to be ONION direct memory, not heap memory. */
    if (videodec2_dmem_alloc(&api->queue_mem, (size_t)cm.cpuGpuMemorySize,
                             ORBIS_VIDEODEC2_DMEM_TYPE_ONION) < 0)
        return AVERROR(ENOMEM);
    cm.cpuGpuMemory = api->queue_mem.ptr;

    rc = -1;
    for (size_t i = 0; i < FF_ARRAY_ELEMS(try_list); i++) {
        OrbisVideodec2ComputeQueue q = NULL;

        memset(&cc, 0, sizeof(cc));
        cc.thisSize        = sizeof(cc);
        cc.computePipeId   = try_list[i].pipe;
        cc.computeQueueId  = try_list[i].queue;
        cc.checkMemoryType = true;

        rc = api->AllocateComputeQueue(&cc, &cm, &q);
        if (rc == 0 && q) {
            api->queue = q;
            return 0;
        }

        av_log(NULL, AV_LOG_VERBOSE,
               "videodec2: AllocateComputeQueue(pipe=%u queue=%u) => 0x%08x\n",
               try_list[i].pipe, try_list[i].queue, (unsigned)rc);

        if ((unsigned)rc == 0x811d0109u)
            av_log(NULL, AV_LOG_WARNING,
                   "videodec2: compute memory is not ONION.\n");
        else if ((unsigned)rc == 0x811d010au)
            av_log(NULL, AV_LOG_WARNING,
                   "videodec2: compute memory is not GARLIC.\n");
    }

    videodec2_dmem_free(&api->queue_mem);
    api->queue = NULL;
    return rc < 0 ? rc : AVERROR_EXTERNAL;
}

#else /* !__ORBIS__ */

/*
 * Host build: no PS4 runtime.  Everything fails cleanly so that opening the
 * decoder reports "unavailable" instead of handing the firmware memory it
 * cannot access.
 */
int videodec2_dmem_alloc(Videodec2Dmem *dm, size_t size, int32_t mem_type)
{
    (void)size;
    (void)mem_type;
    if (dm)
        memset(dm, 0, sizeof(*dm));
    return AVERROR(ENOSYS);
}

void videodec2_dmem_free(Videodec2Dmem *dm)
{
    if (dm)
        memset(dm, 0, sizeof(*dm));
}

int videodec2_load(videodec2_api_t *api)
{
    if (api) {
        memset(api, 0, sizeof(*api));
        api->module = -1;
    }
    return AVERROR(ENOSYS);
}

void videodec2_unload(videodec2_api_t *api)
{
    if (api) {
        memset(api, 0, sizeof(*api));
        api->module = -1;
    }
}

int videodec2_ensure_compute_queue(videodec2_api_t *api)
{
    (void)api;
    return AVERROR(ENOSYS);
}

#endif /* __ORBIS__ */
