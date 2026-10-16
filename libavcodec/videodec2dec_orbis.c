/*
 * PS4 Videodec2 platform glue.
 *
 * This file contains the only code in the Videodec2 decoder that talks to the
 * Sony / OpenOrbis runtime (sceKernel*, libSceVideodec2). It is compiled only
 * when the target provides those symbols; on a normal host build
 * videodec2_load() fails cleanly so the decoder simply reports "unavailable".
 *
 * The API symbol names and the direct-memory helpers below mirror the
 * validated implementation from the Moonlight-ps4 project.
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

#include "videodec2dec.h"

#ifdef __ORBIS__

#include <dlfcn.h>
#include <stdlib.h>
#include <string.h>

#include <orbis/libkernel.h>
#include <orbis/Videodec.h>

#define VD2_SYMBOL(api, field, name)                                          \
    do {                                                                      \
        (api)->field = (void *)dlsym((api)->module, name);                    \
        if (!(api)->field) {                                                  \
            videodec2_unload(api);                                            \
            return -1;                                                        \
        }                                                                     \
    } while (0)

int videodec2_load(videodec2_api_t *api)
{
    memset(api, 0, sizeof(*api));

    /* Name validated in the Moonlight-ps4 spike. */
    api->module = dlopen("libSceVideodec2.sprx", RTLD_LAZY | RTLD_LOCAL);
    if (!api->module)
        api->module = dlopen("libSceVideodec.sprx", RTLD_LAZY | RTLD_LOCAL);
    if (!api->module)
        return -1;

    VD2_SYMBOL(api, QueryComputeMemoryInfo, "sceVideodec2QueryComputeMemoryInfo");
    VD2_SYMBOL(api, AllocateComputeQueue,   "sceVideodec2AllocateComputeQueue");
    VD2_SYMBOL(api, ReleaseComputeQueue,    "sceVideodec2ReleaseComputeQueue");
    VD2_SYMBOL(api, QueryDecoderMemoryInfo, "sceVideodec2QueryDecoderMemoryInfo");
    VD2_SYMBOL(api, CreateDecoder,          "sceVideodec2CreateDecoder");
    VD2_SYMBOL(api, DeleteDecoder,          "sceVideodec2DeleteDecoder");
    VD2_SYMBOL(api, Decode,                 "sceVideodec2Decode");
    VD2_SYMBOL(api, Flush,                  "sceVideodec2Flush");
    VD2_SYMBOL(api, Reset,                  "sceVideodec2Reset");

    return 0;
}

void videodec2_unload(videodec2_api_t *api)
{
    if (api->queue && api->ReleaseComputeQueue)
        api->ReleaseComputeQueue(api->queue);
    api->queue = NULL;
    if (api->module)
        dlclose(api->module);
    api->module = 0;
    memset(api, 0, sizeof(*api));
}

int videodec2_ensure_compute_queue(videodec2_api_t *api)
{
    OrbisVideodec2ComputeConfigInfo cc;
    OrbisVideodec2ComputeMemoryInfo cm;
    int32_t rc;

    if (api->queue)
        return 0;

    memset(&cm, 0, sizeof(cm));
    cm.thisSize = sizeof(cm);
    rc = api->QueryComputeMemoryInfo(&cm);
    if (rc < 0)
        return rc;

    /* ONION: CPU-cacheable and GPU-visible, matching the validated config. */
    cm.cpuGpuMemory = malloc((size_t)cm.cpuGpuMemorySize);
    if (!cm.cpuGpuMemory)
        return -1;

    memset(&cc, 0, sizeof(cc));
    cc.thisSize        = sizeof(cc);
    cc.computePipeId   = 0;
    cc.computeQueueId  = 0;
    cc.checkMemoryType = false;

    rc = api->AllocateComputeQueue(&cc, &cm, &api->queue);
    if (rc < 0) {
        free(cm.cpuGpuMemory);
        api->queue = NULL;
        return rc;
    }
    return 0;
}

#else /* !__ORBIS__ */

/*
 * Host build: the hardware is not present. Report unavailable so that the
 * codec fails cleanly at avcodec_open2() rather than misbehaving.
 */
int videodec2_load(videodec2_api_t *api)
{
    memset(api, 0, sizeof(*api));
    return -1;
}

void videodec2_unload(videodec2_api_t *api)
{
    memset(api, 0, sizeof(*api));
}

int videodec2_ensure_compute_queue(videodec2_api_t *api)
{
    (void)api;
    return -1;
}

#endif /* __ORBIS__ */
