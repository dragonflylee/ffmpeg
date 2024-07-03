/*
 * libsmb2 support by proconsule
 * based on libsmbclient.c by Lukasz Marek <lukasz.m.luki@gmail.com>
 */

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>

#include <smb2/smb2.h>
#include <smb2/libsmb2.h>
#include <smb2/libsmb2-raw.h>

#include "libavutil/avstring.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "avformat.h"
#include "internal.h"
#include "url.h"

typedef struct {
    const AVClass *class;
    struct smb2_context *smb2;
    struct smb2fh *fh;
    struct smb2dir *dir;
    struct smb2_url *url;
    int64_t filesize;
    int trunc;
    int timeout;
    char *workgroup;
} LIBSMBContext;

static av_cold int libsmbc_connect(URLContext *h, const char *url) {
    LIBSMBContext *libsmbc = h->priv_data;

    libsmbc->smb2 = smb2_init_context();
    if (libsmbc->smb2 == NULL) {
        int ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "Failed to init context\n");
        return ret;
    }

    libsmbc->url = smb2_parse_url(libsmbc->smb2, url);
    if (libsmbc->url == NULL) {
        int ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "File parse url\n");
        return ret;
    }

    smb2_set_security_mode(libsmbc->smb2, SMB2_NEGOTIATE_SIGNING_ENABLED);
    if (smb2_connect_share(libsmbc->smb2, libsmbc->url->server, libsmbc->url->share, libsmbc->url->user) != 0) {
        int ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "smb2_connect_share failed. %s\n", smb2_get_error(libsmbc->smb2));
        return ret;
    }

    return 0;
}

static av_cold int libsmbc_close(URLContext *h) {
    LIBSMBContext *libsmbc = h->priv_data;
    if (libsmbc->fh) {
        smb2_close(libsmbc->smb2, libsmbc->fh);
        libsmbc->fh = NULL;
    }
    if (libsmbc->url) {
        smb2_destroy_url(libsmbc->url);
        libsmbc->url = NULL;
    }
    smb2_disconnect_share(libsmbc->smb2);
    smb2_destroy_context(libsmbc->smb2);
    return 0;
}

static av_cold int libsmbc_open(URLContext *h, const char *url, int flags) {
    LIBSMBContext *libsmbc = h->priv_data;
    int ret;
    struct smb2_stat_64 st;
    libsmbc->filesize = -1;

    if ((ret = libsmbc_connect(h, url)) < 0) goto fail;

    libsmbc->fh = smb2_open(libsmbc->smb2, libsmbc->url->path, O_RDONLY);
    if (libsmbc->fh == NULL) {
        ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "File open failed: %s\n", strerror(errno));
        goto fail;
    }

    if (smb2_stat(libsmbc->smb2, libsmbc->url->path, &st) < 0) {
        av_log(h, AV_LOG_WARNING, "Cannot stat file: %s\n", strerror(errno));
    } else {
        libsmbc->filesize = st.smb2_size;
    }

    return 0;
fail:
    libsmbc_close(h);
    return ret;
}

static int64_t libsmbc_seek(URLContext *h, int64_t pos, int whence) {
    LIBSMBContext *libsmbc = h->priv_data;
    int64_t newpos;

    if (whence == AVSEEK_SIZE) {
        if (libsmbc->filesize == -1) {
            av_log(h, AV_LOG_ERROR, "Error during seeking: filesize is unknown.\n");
            return AVERROR(EIO);
        } else
            return libsmbc->filesize;
    }

    if ((newpos = smb2_lseek(libsmbc->smb2, libsmbc->fh, pos, whence, NULL)) < 0) {
        int err = errno;
        av_log(h, AV_LOG_ERROR, "Error during seeking: %s\n", strerror(err));
        return AVERROR(err);
    }

    return newpos;
}

static int libsmbc_read(URLContext *h, unsigned char *buf, int size) {
    LIBSMBContext *libsmbc = h->priv_data;
    int bytes_read;

    if ((bytes_read = smb2_read(libsmbc->smb2, libsmbc->fh, buf, size)) < 0) {
        int ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "Read error: %s\n", strerror(errno));
        return ret;
    }

    return bytes_read ? bytes_read : AVERROR_EOF;
}

static int libsmbc_write(URLContext *h, const unsigned char *buf, int size) {
    LIBSMBContext *libsmbc = h->priv_data;
    int bytes_written;

    if ((bytes_written = smb2_write(libsmbc->smb2, libsmbc->fh, buf, size)) < 0) {
        int ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "Write error: %s\n", strerror(errno));
        return ret;
    }

    return bytes_written;
}

static int libsmbc_open_dir(URLContext *h) {
    LIBSMBContext *libsmbc = h->priv_data;
    int ret;

    if ((ret = libsmbc_connect(h, h->filename)) < 0) goto fail;

    libsmbc->dir = smb2_opendir(libsmbc->smb2, libsmbc->url->path);
    if (libsmbc->dir == NULL) {
        ret = AVERROR(errno);
        av_log(h, AV_LOG_ERROR, "Dir open failed: %s\n", strerror(errno));
        goto fail;
    }

    return 0;
fail:
    libsmbc_close(h);
    return ret;
}

static int libsmbc_read_dir(URLContext *h, AVIODirEntry **next) {
    LIBSMBContext *libsmbc = h->priv_data;
    AVIODirEntry *entry;
    struct smb2dirent *dirent;
    int skip_entry;

    *next = entry = ff_alloc_dir_entry();
    if (!entry) return AVERROR(ENOMEM);

    do {
        skip_entry = 0;
        dirent = smb2_readdir(libsmbc->smb2, libsmbc->dir);
        if (dirent == NULL) {
            av_freep(next);
            return AVERROR(errno);
        }
        switch (dirent->st.smb2_type) {
        case SMB2_TYPE_FILE:
            entry->type = AVIO_ENTRY_FILE;
            break;
        case SMB2_TYPE_DIRECTORY:
            entry->type = AVIO_ENTRY_DIRECTORY;
            break;
        case SMB2_TYPE_LINK:
            entry->type = AVIO_ENTRY_SYMBOLIC_LINK;
            break;
        default:
            skip_entry = 1;
        }
    } while (skip_entry || !strcmp(dirent->name, ".") || !strcmp(dirent->name, ".."));

    entry->name = av_strdup(dirent->name);
    entry->size = dirent->st.smb2_size;
    entry->modification_timestamp = dirent->st.smb2_mtime;
    entry->access_timestamp = dirent->st.smb2_atime;
    entry->status_change_timestamp = dirent->st.smb2_ctime;

    return 0;
}

static int libsmbc_close_dir(URLContext *h) {
    LIBSMBContext *libsmbc = h->priv_data;
    if (libsmbc->dir) {
        smb2_closedir(libsmbc->smb2, libsmbc->dir);
        libsmbc->dir = NULL;
    }
    libsmbc_close(h);
    return 0;
}

static int libsmbc_delete(URLContext *h) { return AVERROR(EPERM); }

static int libsmbc_move(URLContext *h_src, URLContext *h_dst) { return AVERROR(EPERM); }

#define OFFSET(x) offsetof(LIBSMBContext, x)
#define D AV_OPT_FLAG_DECODING_PARAM
#define E AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    {"timeout", "set timeout in ms of socket I/O operations", OFFSET(timeout), AV_OPT_TYPE_INT, {.i64 = -1}, -1, INT_MAX, D | E},
    {"truncate", "truncate existing files on write", OFFSET(trunc), AV_OPT_TYPE_INT, {.i64 = 1}, 0, 1, E},
    {"workgroup", "set the workgroup used for making connections", OFFSET(workgroup), AV_OPT_TYPE_STRING, {0}, 0, 0, D | E}, {NULL}};

static const AVClass libsmb2_context_class = {
    .class_name = "libsmb2",
    .item_name = av_default_item_name,
    .option = options,
    .version = LIBAVUTIL_VERSION_INT,
};

const URLProtocol ff_libsmb2_protocol = {
    .name = "smb",
    .url_open = libsmbc_open,
    .url_read = libsmbc_read,
    .url_write = libsmbc_write,
    .url_seek = libsmbc_seek,
    .url_close = libsmbc_close,
    .url_delete = libsmbc_delete,
    .url_move = libsmbc_move,
    .url_open_dir = libsmbc_open_dir,
    .url_read_dir = libsmbc_read_dir,
    .url_close_dir = libsmbc_close_dir,
    .priv_data_size = sizeof(LIBSMBContext),
    .priv_data_class = &libsmb2_context_class,
    .flags = URL_PROTOCOL_FLAG_NETWORK,
};
