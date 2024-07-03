/*
 * libssh2 support by proconsule
 * based on libssh.c by Lukasz Marek <lukasz.m.luki@gmail.com>
 */

#include <fcntl.h>

#include <libssh2.h>
#include <libssh2_sftp.h>

#include "libavutil/avstring.h"
#include "libavutil/attributes.h"
#include "libavutil/opt.h"
#include "libavutil/mem.h"
#include "libavformat/avio.h"
#include "network.h"
#include "avformat.h"
#include "internal.h"
#include "url.h"

typedef struct {
    const AVClass *class;
    URLContext *tcp;
    LIBSSH2_SESSION *session;
    LIBSSH2_SFTP *sftp;
    LIBSSH2_SFTP_HANDLE *file;
    int64_t filesize;
    int rw_timeout;
    int trunc;
    char *priv_key;
} LIBSSHContext;

static int ssh2_error(int error) {
    switch (error) {
    case LIBSSH2_ERROR_NONE:
        return 0;
    case LIBSSH2_ERROR_ALLOC:
        return ENOMEM;
    case LIBSSH2_ERROR_SOCKET_TIMEOUT:
        return ETIMEDOUT;
    case LIBSSH2_ERROR_EAGAIN:
        return EAGAIN;
    case LIBSSH2_ERROR_SFTP_PROTOCOL:
        return EINVAL;
    default:
        return EIO;
    }
}

static av_cold int libssh2_close(URLContext *h)
{
    LIBSSHContext *libssh2 = h->priv_data;
    if (libssh2->file) {
        libssh2_sftp_close(libssh2->file);
        libssh2->file = NULL;
    }
    if (libssh2->sftp) {
        libssh2_sftp_shutdown(libssh2->sftp);
        libssh2->sftp = NULL;
    }
    if (libssh2->session) {
        libssh2_session_disconnect(libssh2->session, "Normal Shutdown");
        libssh2_session_free(libssh2->session);
        libssh2->session = NULL;
    }
    ffurl_closep(&libssh2->tcp);
    libssh2_exit();
    return 0;
}

static av_cold int libssh2_authentication(LIBSSHContext *libssh2, const char *user, const char *password) {
    size_t username_len = user ? strlen(user) : 0;
    size_t password_len = password ? strlen(password) : 0;
    return libssh2_userauth_password_ex(libssh2->session, user, username_len, password, password_len, NULL);
}

static av_cold int libssh2_connect(URLContext *h, const char *url, char *path, size_t path_size)
{
    LIBSSHContext *libssh2 = h->priv_data;
    AVDictionary *opts = NULL;
    struct sockaddr_in sin;
    char proto[10], hostname[255], credencials[1024], buf[512];
    int port = 22, ret;
    const char *user = NULL, *pass = NULL;
    char *end = NULL;

    av_url_split(proto, sizeof(proto),
                 credencials, sizeof(credencials),
                 hostname, sizeof(hostname),
                 &port,
                 path, path_size,
                 url);

    if (!(*path)) av_strlcpy(path, "/", path_size);

    // a port of 0 will use a port from ~/.ssh/config or the default value 22
    if (port < 0 || port > 65535) port = 0;

    ff_url_join(buf, sizeof(buf), "tcp", NULL, hostname, port, NULL);
    ret = ffurl_open_whitelist(&libssh2->tcp, buf, AVIO_FLAG_READ_WRITE, 
        &h->interrupt_callback, &opts, h->protocol_whitelist, h->protocol_blacklist, h);
    if (ret < 0) return ret;

    ret = libssh2_init(0);
    if (ret) {
        av_log(libssh2, AV_LOG_ERROR, "SSH2 session creation failed: %d\n", ret);
        return AVERROR(ENOMEM);
    }

    libssh2->session = libssh2_session_init();
    if (!libssh2->session) {
        av_log(libssh2, AV_LOG_ERROR, "unable to create session\n");
        return AVERROR(EIO);
    }

    int socket = ffurl_get_file_handle(libssh2->tcp);
    ret = libssh2_session_handshake(libssh2->session, socket);
    if (ret) {
        av_log(libssh2, AV_LOG_ERROR, "Failure establishing SSH session: %d\n", ret);
        return AVERROR(ssh2_error(ret));
    }

    user = av_strtok(credencials, ":", &end);
    pass = av_strtok(end, ":", &end);

    if (libssh2_authentication(libssh2, user, pass))
        return AVERROR(EACCES);

    libssh2->sftp = libssh2_sftp_init(libssh2->session);
    if (!libssh2->sftp) {
        av_log(libssh2, AV_LOG_ERROR, "Error initializing sftp session:\n");
        return AVERROR(EIO);
    }

    libssh2_session_set_blocking(libssh2->session, 1);
    return 0;
}

static av_cold int libssh2_open(URLContext *h, const char *url, int flags)
{
    int ret;
    LIBSSHContext *libssh2 = h->priv_data;
    LIBSSH2_SFTP_ATTRIBUTES attr;
    char path[MAX_URL_SIZE];

    if ((ret = libssh2_connect(h, url, path, sizeof(path))) < 0) goto fail;

    libssh2->file = libssh2_sftp_open(libssh2->sftp, path, LIBSSH2_FXF_READ, 0);
    if (libssh2->file == NULL) {
        av_log(libssh2, AV_LOG_ERROR, "Error opening sftp file: \n");
        goto fail;
    }

    libssh2_sftp_fstat_ex(libssh2->file, &attr, 0);
    libssh2->filesize = (size_t)attr.filesize;
    return 0;

fail:
    libssh2_close(h);
    return AVERROR(ssh2_error(ret));
}

static int64_t libssh2_seek(URLContext *h, int64_t pos, int whence)
{
    LIBSSHContext *libssh2 = h->priv_data;
    int64_t newpos;

    if (libssh2->filesize == -1 && (whence == AVSEEK_SIZE || whence == SEEK_END)) {
        av_log(h, AV_LOG_ERROR, "Error during seeking.\n");
        return AVERROR(EIO);
    }

    switch (whence) {
    case AVSEEK_SIZE:
        return libssh2->filesize;
    case SEEK_SET:
        newpos = pos;
        break;
    case SEEK_CUR:
        newpos = libssh2_sftp_tell64(libssh2->file) + pos;
        break;
    case SEEK_END:
        newpos = libssh2->filesize + pos;
        break;
    default:
        return AVERROR(EINVAL);
    }

    if (newpos < 0) {
        av_log(h, AV_LOG_ERROR, "Seeking to nagative position.\n");
        return AVERROR(EINVAL);
    }

    libssh2_sftp_seek64(libssh2->file, newpos);

    return newpos;
}

static int libssh2_read(URLContext *h, unsigned char *buf, int size)
{
    LIBSSHContext *libssh2 = h->priv_data;
    int bytes_read;

    if ((bytes_read = libssh2_sftp_read(libssh2->file, buf, size)) < 0) {
        av_log(libssh2, AV_LOG_ERROR, "Read error.\n");
        return AVERROR(EIO);
    }
    return bytes_read ? bytes_read : AVERROR_EOF;
}

static int libssh2_write(URLContext *h, const unsigned char *buf, int size)
{
    LIBSSHContext *libssh2 = h->priv_data;
    int bytes_written;

    if ((bytes_written = libssh2_sftp_write(libssh2->file, buf, size)) < 0) {
        av_log(libssh2, AV_LOG_ERROR, "Write error.\n");
        return AVERROR(EIO);
    }
    return bytes_written;
}

static int libssh2_open_dir(URLContext *h)
{
    int ret;
    LIBSSHContext *libssh2 = h->priv_data;
    char path[MAX_URL_SIZE];

    if ((ret = libssh2_connect(h, h->filename, path, sizeof(path))) < 0) {
        libssh2_close(h);
        return ret;
    }

    libssh2->file = libssh2_sftp_opendir(libssh2->sftp, path);
    if (!libssh2->file) {
        libssh2_close(h);
        return AVERROR(EIO);
    }
    return 0;
}

static int libssh2_read_dir(URLContext *h, AVIODirEntry **next)
{
    int ret;
    char name[512];
    LIBSSH2_SFTP_ATTRIBUTES attrs;
    AVIODirEntry *entry;
    int skip_entry;
    LIBSSHContext *libssh2 = h->priv_data;

    *next = entry = ff_alloc_dir_entry();
    if (!entry) return AVERROR(ENOMEM);

    do {
        skip_entry = 0;

        memset(name, 0, sizeof(name));
        ret = libssh2_sftp_readdir(libssh2->file, name, sizeof(name), &attrs);
        if (ret <= 0) {
            av_freep(next);
            return 0;
        }

        if (LIBSSH2_SFTP_S_ISDIR(attrs.permissions))
            entry->type = AVIO_ENTRY_DIRECTORY;
        else if (LIBSSH2_SFTP_S_ISREG(attrs.permissions))
            entry->type = AVIO_ENTRY_FILE;
        else if (LIBSSH2_SFTP_S_ISLNK(attrs.permissions))
            entry->type = AVIO_ENTRY_SYMBOLIC_LINK;
        else
            skip_entry = 1;

    } while (skip_entry || !strcmp(name, ".") || !strcmp(name, ".."));

    entry->name = av_strdup(name);
    entry->filemode = attrs.permissions; 
    if (attrs.flags & LIBSSH2_SFTP_ATTR_SIZE) {
        entry->size = attrs.filesize;
    }
    if (attrs.flags & LIBSSH2_SFTP_ATTR_ACMODTIME) {
        entry->modification_timestamp = attrs.mtime;
        entry->access_timestamp = attrs.atime;
    }
    if (attrs.flags & LIBSSH2_SFTP_ATTR_UIDGID) {
        entry->group_id = attrs.gid;
        entry->user_id = attrs.uid;
    }
    return 0;
}

static int libssh2_delete(URLContext *h) { return AVERROR(EPERM); }

static int libssh2_move(URLContext *h_src, URLContext *h_dst) { return AVERROR(EPERM); }

#define OFFSET(x) offsetof(LIBSSHContext, x)
#define D AV_OPT_FLAG_DECODING_PARAM
#define E AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {{"timeout", "set timeout of socket I/O operations", OFFSET(rw_timeout),
                                       AV_OPT_TYPE_INT, {.i64 = -1}, -1, INT_MAX, D | E},
    {"truncate", "Truncate existing files on write", OFFSET(trunc), AV_OPT_TYPE_INT, {.i64 = 1}, 0, 1, E},
    {"private_key", "set path to private key", OFFSET(priv_key), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, D | E},
    {NULL}};

static const AVClass libssh2_context_class = {
    .class_name = "libssh2",
    .item_name = av_default_item_name,
    .option = options,
    .version = LIBAVUTIL_VERSION_INT,
};

const URLProtocol ff_libssh2_protocol = {
    .name = "sftp",
    .url_open = libssh2_open,
    .url_read = libssh2_read,
    .url_write = libssh2_write,
    .url_seek = libssh2_seek,
    .url_close = libssh2_close,
    .url_delete = libssh2_delete,
    .url_move = libssh2_move,
    .url_open_dir = libssh2_open_dir,
    .url_read_dir = libssh2_read_dir,
    .url_close_dir = libssh2_close,
    .priv_data_size = sizeof(LIBSSHContext),
    .priv_data_class = &libssh2_context_class,
    .flags = URL_PROTOCOL_FLAG_NETWORK,
};
