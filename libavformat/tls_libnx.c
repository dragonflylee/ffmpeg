/*
 * TLS/SSL Protocol
 * Copyright (c) 2018 Thomas Volkert
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

#include <switch.h>

#include "avformat.h"
#include "internal.h"
#include "url.h"
#include "tls.h"
#include "libavutil/parseutils.h"

typedef struct TLSContext {
    const AVClass *class;
    TLSShared tls_shared;
    SslContext context;
    SslConnection conn;
} TLSContext;

static int tls_close(URLContext *h)
{
    TLSContext *tls_ctx = h->priv_data;

    sslConnectionClose(&tls_ctx->conn);
    sslContextClose(&tls_ctx->context);

    ffurl_closep(&tls_ctx->tls_shared.tcp);
    return 0;
}

static int tls_open(URLContext *h, const char *uri, int flags, AVDictionary **options)
{
    TLSContext *tls_ctx = h->priv_data;
    TLSShared *shr = &tls_ctx->tls_shared;
    int ret;
    Result rc;

    if (shr->listen) {
        av_log(h, AV_LOG_ERROR, "TLS Listen Sockets with libnx is not implemented.\n");
        return AVERROR(EINVAL);
    }

    if ((ret = ff_tls_open_underlying(shr, h, uri, options)) < 0)
        goto fail;

    rc = sslCreateContext(&tls_ctx->context, SslVersion_Auto);
    if (R_FAILED(rc)) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    rc = sslContextCreateConnection(&tls_ctx->context, &tls_ctx->conn);

    if (R_SUCCEEDED(rc)) {
        rc = sslConnectionSetOption(&tls_ctx->conn, SslOptionType_DoNotCloseSocket, true);
    }

    if (R_SUCCEEDED(rc)) {
        int sockfd = ffurl_get_file_handle(shr->tcp);
        ret = socketSslConnectionSetSocketDescriptor(&tls_ctx->conn, sockfd);
        if (ret == -1 && errno != ENOENT) {
            ret = AVERROR(EIO);
            goto fail;
        }
    }

    if (R_SUCCEEDED(rc)) {
        rc = sslConnectionSetHostName(&tls_ctx->conn, shr->host, strlen(shr->host));
    }

    /* This will fail on system-versions where this option isn't available,
     * so ignore errors from this. */
    if (R_SUCCEEDED(rc)) {
        sslConnectionSetOption(&tls_ctx->conn, SslOptionType_SkipDefaultVerify, true);
    }

    if (R_SUCCEEDED(rc)) {
        u32 verifyopt = SslVerifyOption_DateCheck;
        if (shr->verify) verifyopt |= SslVerifyOption_PeerCa;
        rc = sslConnectionSetVerifyOption(&tls_ctx->conn, verifyopt);
    }

    if (R_SUCCEEDED(rc)) {
        SslIoMode iomode = SslIoMode_Blocking;
        if (h->flags & AVIO_FLAG_NONBLOCK) iomode = SslIoMode_NonBlocking;
        rc = sslConnectionSetIoMode(&tls_ctx->conn, iomode);
    }

    while (rc = sslConnectionDoHandshake(&tls_ctx->conn, NULL, NULL, NULL, 0)) {
        if (R_VALUE(rc) == MAKERESULT(123, 204)) { // PR_WOULD_BLOCK_ERROR
            svcSleepThread(100000000);
            continue;
        }
        if (R_VALUE(rc) == MAKERESULT(123, 207)) {
            av_log(h, AV_LOG_ERROR, "The certificate is not correctly signed by the trusted CA.\n");
        } else {
            av_log(h, AV_LOG_ERROR, "sslConnectionDoHandshake returned: 0x%X\n", R_VALUE(rc));
        }
        ret = AVERROR(EIO);
        goto fail;
    }
    return 0;

fail:
    tls_close(h);
    return ret;
}

static int tls_read(URLContext *h, uint8_t *buf, int size)
{
    TLSContext *tls_ctx = h->priv_data;
    Result rc = 0;
    u32 out_size = 0;
    rc = sslConnectionRead(&tls_ctx->conn, buf, size, &out_size);
    if (R_SUCCEEDED(rc)) {
        return out_size; // return read length
    }
    if (R_VALUE(rc) == MAKERESULT(123, 204)) { // PR_WOULD_BLOCK_ERROR
        return AVERROR(EAGAIN);
    }
    av_log(h, AV_LOG_WARNING, "sslConnectionRead returned 0x%X\n", R_VALUE(rc));
    return AVERROR(EIO);
}

static int tls_write(URLContext *h, const uint8_t *buf, int size)
{
    TLSContext *tls_ctx = h->priv_data;
    Result rc = 0;
    u32 out_size = 0;
    rc = sslConnectionWrite(&tls_ctx->conn, buf, size, &out_size);
    if (R_SUCCEEDED(rc)) {
        return out_size; // return written length
    }
    if (R_VALUE(rc) == MAKERESULT(123, 204)) { // PR_WOULD_BLOCK_ERROR 
        return AVERROR(EAGAIN);
    }
    av_log(h, AV_LOG_WARNING, "sslConnectionWrite returned 0x%X\n", R_VALUE(rc));
    return AVERROR(EIO);
}

static int tls_get_file_handle(URLContext *h)
{
    TLSContext *c = h->priv_data;
    return ffurl_get_file_handle(c->tls_shared.tcp);
}

static int tls_get_short_seek(URLContext *h)
{
    TLSContext *s = h->priv_data;
    return ffurl_get_short_seek(s->tls_shared.tcp);
}

static const AVOption options[] = {
    TLS_COMMON_OPTIONS(TLSContext, tls_shared),
    { NULL }
};

static const AVClass tls_class = {
    .class_name = "tls",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const URLProtocol ff_tls_protocol = {
    .name           = "tls",
    .url_open2      = tls_open,
    .url_read       = tls_read,
    .url_write      = tls_write,
    .url_close      = tls_close,
    .url_get_file_handle = tls_get_file_handle,
    .url_get_short_seek  = tls_get_short_seek,
    .priv_data_size = sizeof(TLSContext),
    .flags          = URL_PROTOCOL_FLAG_NETWORK,
    .priv_data_class = &tls_class,
};
