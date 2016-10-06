/**
 * Copyright (c) 2015 Parrot S.A.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file tlm_shm_dump.c
 *
 * @brief Tool to dump the contents of a shared memory used by telemetry.
 *
 * It does not depends on libshdata or libtelemetry directly. It uses
 * directly knowledge of internal data structures of both libraries. This
 * makes it independent in term of source code but tightly coupled in term
 * of data format.
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

/* Just for the enum of variable types */
#include "libtelemetry.h"

#ifndef STATIC_ASSERT
#  define STATIC_ASSERT(x) void __STATIC_ASSERT__(char[(x) ? 1 : -1])
#endif /* !STATIC_ASSERT */

/** Alignment of variables in internal memory */
#define ALIGNMENT  8

/** Align a size or offset up to ALIGNMENT */
#define ALIGN_UP(x) (((x) + ALIGNMENT - 1) & ~(ALIGNMENT - 1))

#define DIAG_PFX "TLMSHMDUMP: "

#define diag(_fmt, ...) \
	fprintf(stderr, DIAG_PFX _fmt "\n", ##__VA_ARGS__)

#define diag_errno(_func) \
	diag("%s error=%d(%s)", _func, errno, strerror(errno))

#define diag_fd_errno(_func, _fd) \
	diag("%s (fd=%d) : err=%d(%s)", _func, _fd, errno, strerror(errno))

/* libshdata magic number */
#define TLM_SHM_SHD_MAGIC_NUMBER 0x65756821

/* Magic number at start of shared memory metadata header */
#define TLM_SHM_HDR_MAGIC	0x214d4c54	/* TLM! */

/**
 * Section header stored in shared memory.
 */
struct tlm_shm_section_hdr {
	uint64_t magic_number;
	uint32_t lib_version_maj;
	uint32_t lib_version_min;
	size_t   blob_size;
	uint32_t max_nb_samples;
	uint32_t rate;
	size_t   metadata_hdr_size;
	int32_t  nb_creations;
	int32_t  write_index;
	int32_t  wtid;
};
#if defined(__WORDSIZE) && (__WORDSIZE == 64)
STATIC_ASSERT(sizeof(struct tlm_shm_section_hdr) == 56);
#elif defined(__arm__)
STATIC_ASSERT(sizeof(struct tlm_shm_section_hdr) == 48);
#else
STATIC_ASSERT(sizeof(struct tlm_shm_section_hdr) == 44);
#endif

/**
 * Header of a sample stored in shared memory.
 */
struct tlm_shm_sample_hdr {
	struct timespec  ts;
	struct timespec  exp;
	int32_t          nb_writes;
};
#if defined(__WORDSIZE) && (__WORDSIZE == 64)
STATIC_ASSERT(sizeof(struct tlm_shm_sample_hdr) == 40);
#else
STATIC_ASSERT(sizeof(struct tlm_shm_sample_hdr) == 20);
#endif

/**
 * Variable description record stored in shared memory.
 */
struct tlm_shm_var_rec {
	uint32_t  reclen;   /**< Size of record (struct + name + 0 + padding) */
	uint32_t  namelen;  /**< Size of name (not including '\0') */
	uint32_t  type;     /**< Type of variable */
	uint32_t  size;     /**< Size of variable */
	uint32_t  count;    /**< Number of elements for arrays */
	uint32_t  flags;    /**< Additional flags */
	char      name[];   /**< Name follows this structure */
};
STATIC_ASSERT(sizeof(struct tlm_shm_var_rec) == 24);

/**
 * Variable information.
 */
struct tlm_shm_var_info {
	struct tlm_shm_var_rec  *rec;  /**< Description record */
	size_t  off;                   /**< Offset in blob */
	size_t  size;                  /**< Size in blob */
};

/**
 * Dump context.
 */
struct tlm_shm_ctx {
	int                      fd;
	void                     *data;
	size_t                   datasize;
	const struct tlm_shm_section_hdr  *section_hdr;
	const void               *metadata_hdr;
	const void               *samples;
	uint32_t                 varcount;
	struct tlm_shm_var_info  *varinfos;
};

/**
 */
static int tlm_shm_close(struct tlm_shm_ctx *ctx)
{
	uint32_t i = 0;
	if (ctx == NULL)
		return -EINVAL;

	/* Free variable info table */
	if (ctx->varinfos != NULL) {
		for (i = 0; i < ctx->varcount; i++)
			free(ctx->varinfos[i].rec);
		free(ctx->varinfos);
	}

	/** Unmap memory */
	if (ctx->data != NULL && ctx->data != MAP_FAILED)
		munmap(ctx->data, ctx->datasize);

	/* Close section */
	if (ctx->fd >= 0)
		close(ctx->fd);

	/* Free context itself */
	free(ctx);

	return 0;
}

/**
 */
static struct tlm_shm_ctx *tlm_shm_open(const char *shmpath)
{
	struct tlm_shm_ctx *ctx = NULL;
	struct stat st;
	size_t metadata_off = 0, metadata_size = 0;
	size_t samples_off = 0, samples_size = 0;

	if (shmpath == NULL)
		return NULL;

	/* Allocate context structure */
	ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL)
		goto error;
	ctx->fd = -1;

	/* Use open instead of shm_open to be able to dump any section even
	 * if not in /dev/shm */
	ctx->fd = open(shmpath, O_RDONLY);
	if (ctx->fd < 0) {
		diag("Failed to open '%s': err=%d(%s)",
				shmpath, errno, strerror(errno));
		goto error;
	}

	/* Get size of section */
	if (fstat(ctx->fd, &st) < 0) {
		diag_fd_errno("fstat", ctx->fd);
		goto error;
	}

	/* Map section */
	ctx->datasize = st.st_size;
	ctx->data = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, ctx->fd, 0);
	if (ctx->data == MAP_FAILED) {
		diag_fd_errno("mmap", ctx->fd);
		goto error;
	}

	/* Make sure section header was mapped */
	if (ctx->datasize < sizeof(struct tlm_shm_section_hdr)) {
		diag("Section size is too small: %zu (%zu)",
				ctx->datasize,
				sizeof(struct tlm_shm_section_hdr));
		goto error;
	}

	/* Section header pointer */
	ctx->section_hdr = ctx->data;

	/* Check that the section is indeed a shared memory section */
	if (ctx->section_hdr->magic_number != TLM_SHM_SHD_MAGIC_NUMBER) {
		diag("Section was not created with libshdata");
		goto error;
	}

	/* Determine offsets and sizes of section parts */
	metadata_off = ALIGN_UP(sizeof(struct tlm_shm_section_hdr));
	metadata_size = ALIGN_UP(ctx->section_hdr->metadata_hdr_size);
	samples_off = metadata_off + metadata_size;
	samples_size = ctx->section_hdr->max_nb_samples *
			(ALIGN_UP(sizeof(struct tlm_shm_sample_hdr) +
					ctx->section_hdr->blob_size));

	/* Section metadata header and samples */
	ctx->metadata_hdr = (const uint8_t *)ctx->data + metadata_off;
	ctx->samples = (const uint8_t *)ctx->data + samples_off;

	/* Check that everything is OK */
	if (metadata_off + metadata_size > ctx->datasize) {
		diag("Metadata header does not fit in section");
		goto error;
	}
	if (samples_off + samples_size > ctx->datasize) {
		diag("Samples zone does not fit in section");
		goto error;
	}

	return ctx;

	/* Cleanup in case of error */
error:
	if (ctx != NULL)
		tlm_shm_close(ctx);
	return NULL;
}

/**
 */
static struct tlm_shm_var_rec *tlm_shm_read_var_desc_record(
		const void *src, size_t maxsrc)
{
	struct tlm_shm_var_rec rec;
	struct tlm_shm_var_rec *prec = NULL;

	/* Check validity of buffer */
	if (maxsrc < sizeof(rec)) {
		diag("Buffer too small: %zu (%zu)",
				maxsrc, sizeof(rec));
		return NULL;
	}
	memcpy(&rec, src, sizeof(rec));
	if (maxsrc < rec.reclen) {
		diag("Buffer too small: %zu (%u)", maxsrc, rec.reclen);
		return NULL;
	}
	if (maxsrc < sizeof(rec) + rec.namelen + 1) {
		diag("Buffer too small: %zu (%zu)", maxsrc,
				sizeof(rec) + rec.namelen + 1);
		return NULL;
	}
	if (*((const uint8_t *)src + sizeof(rec) + rec.namelen) != '\0') {
		diag("String not null terminated");
		return NULL;
	}

	prec = calloc(rec.reclen, 1);
	memcpy(prec, src, rec.reclen);
	return prec;
}

/**
 */
static int tlm_shm_read_metadata_hdr(struct tlm_shm_ctx *ctx)
{
	int res = 0, ok = 0;
	uint32_t i = 0;
	uint32_t magic = 0;
	const uint8_t *hdrptr = NULL;
	size_t hdroff = 0, hdrsize = 0, varoff = 0, varsize = 0;
	struct tlm_shm_var_rec *rec = NULL;

	if (ctx == NULL)
		return -EINVAL;

	/* Setup pointer and offset */
	hdrptr = ctx->metadata_hdr;
	hdroff = 0;
	hdrsize = ctx->section_hdr->metadata_hdr_size;

	/* Magic */
	memcpy(&magic, hdrptr, sizeof(uint32_t));
	hdrptr += sizeof(uint32_t);
	hdroff += sizeof(uint32_t);
	if (magic != 0x214D4C54) {
		diag("Bad magic: 0x%08x", magic);
		return 0;
	}

	/* How many variables are in the section ? */
	memcpy(&ctx->varcount, hdrptr, sizeof(uint32_t));
	hdrptr += sizeof(uint32_t);
	hdroff += sizeof(uint32_t);

	/* Allocate variable info table */
	ctx->varinfos = calloc(ctx->varcount, sizeof(*ctx->varinfos));
	if (ctx->varinfos == NULL) {
		res = -ENOMEM;
		goto out;
	}

	/* Read all descriptions */
	for (i = 0; i < ctx->varcount; i++) {
		/* Read record */
		rec = tlm_shm_read_var_desc_record(hdrptr, hdrsize - hdroff);
		if (rec == NULL)
			break;

		/* Check associated variable fits in blob */
		varsize = rec->size * rec->count;
		if (varoff + varsize > ctx->section_hdr->blob_size) {
			diag("Variable '%s' does not fit in blob size",
					rec->name);
			free(rec);
			rec = NULL;
			break;
		}

		/* Check that type and size match */
		switch (rec->type) {
		case TLM_TYPE_BOOL:
			ok = rec->size == 1;
			break;
		case TLM_TYPE_UINT8:
			ok = rec->size == 1;
			break;
		case TLM_TYPE_INT8:
			ok = rec->size == 1;
			break;
		case TLM_TYPE_UINT16:
			ok = rec->size == 2;
			break;
		case TLM_TYPE_INT16:
			ok = rec->size == 2;
			break;
		case TLM_TYPE_UINT32:
			ok = rec->size == 4;
			break;
		case TLM_TYPE_INT32:
			ok = rec->size == 4;
			break;
		case TLM_TYPE_UINT64:
			ok = rec->size == 8;
			break;
		case TLM_TYPE_INT64:
			ok = rec->size == 8;
			break;
		case TLM_TYPE_FLOAT32:
			ok = rec->size == 4;
			break;
		case TLM_TYPE_FLOAT64:
			ok = rec->size == 8;
			break;
		case TLM_TYPE_STRING:
			ok = rec->size == 1;
			break;
		case TLM_TYPE_BINARY:
			ok = 1;
			break;
		default:
			ok = 0;
			break;
		}

		if (!ok) {
			diag("Variable '%s': size/type mismatch %u/%s",
					rec->name, rec->size,
					tlm_get_var_type_str(rec->type));
		} else {
			/* Ok, save information */
			ctx->varinfos[i].rec = rec;
			ctx->varinfos[i].off = varoff;
			ctx->varinfos[i].size = varsize;
		}

		/* In any case, continue with next variable */
		varoff += varsize;
		hdrptr += rec->reclen;
		hdroff += rec->reclen;
		if (!ok) {
			free(rec);
			rec = NULL;
		}
	}

out:
	return res;
}

/**
 */
static void dump_header(struct tlm_shm_ctx *ctx)
{
	const struct tlm_shm_section_hdr *hdr = ctx->section_hdr;
	const struct tlm_shm_var_rec *rec = NULL;
	uint32_t i = 0;

	fprintf(stdout, "lib_version_maj = %u\n", hdr->lib_version_maj);
	fprintf(stdout, "lib_version_min = %u\n", hdr->lib_version_min);
	fprintf(stdout, "blob_size = %zu\n", hdr->blob_size);
	fprintf(stdout, "max_nb_samples = %u\n", hdr->max_nb_samples);
	fprintf(stdout, "rate = %u\n", hdr->rate);
	fprintf(stdout, "metadata_hdr_size = %zu\n", hdr->metadata_hdr_size);
	fprintf(stdout, "nb_creations = %d\n", hdr->nb_creations);
	fprintf(stdout, "write_index = %d\n", hdr->write_index);
	fprintf(stdout, "wtid = %d\n", hdr->wtid);

	fprintf(stdout, "varcount = %u\n", ctx->varcount);
	for (i = 0; i < ctx->varcount; i++) {
		rec = ctx->varinfos[i].rec;
		if (rec != NULL) {
			fprintf(stdout, "%u;%s;%s;size=%u;count=%u;flags=%u\n",
					i, rec->name,
					tlm_get_var_type_str(rec->type),
					rec->size, rec->count, rec->flags);
		}
	}
}

#define PRINT_VAR(_type, _fmt, _ptr, _len) \
	do { \
		_type val; \
		assert(sizeof(_type) <= (_len)); \
		memcpy(&val, (_ptr), sizeof(_type)); \
		fprintf(stdout, (_fmt), (val)); \
	} while (0)

/**
 */
static void dump_var(struct tlm_shm_ctx *ctx,
		const struct tlm_shm_var_rec *rec,
		const void *buf, size_t len)
{
	uint32_t i = 0, j = 0;
	const uint8_t *ptr = NULL;
	size_t off = 0;

	fprintf(stdout, "  %s:", rec->name);
	if (rec->count > 1)
		fprintf(stdout, "[");

	ptr = buf;
	off = 0;
	for (i = 0; i < rec->count; i++) {
		switch (rec->type) {
		case TLM_TYPE_BOOL:
			PRINT_VAR(uint8_t, "%u", ptr, len - off);
			break;

		case TLM_TYPE_UINT8:
			PRINT_VAR(uint8_t, "%u", ptr, len - off);
			break;

		case TLM_TYPE_INT8:
			PRINT_VAR(int8_t, "%d", ptr, len - off);
			break;

		case TLM_TYPE_UINT16:
			PRINT_VAR(uint16_t, "%u", ptr, len - off);
			break;

		case TLM_TYPE_INT16:
			PRINT_VAR(int16_t, "%d", ptr, len - off);
			break;

		case TLM_TYPE_UINT32:
			PRINT_VAR(uint32_t, "%u", ptr, len - off);
			break;

		case TLM_TYPE_INT32:
			PRINT_VAR(int32_t, "%d", ptr, len - off);
			break;

		case TLM_TYPE_UINT64:
			PRINT_VAR(uint64_t, "%" PRIu64, ptr, len - off);
			break;

		case TLM_TYPE_INT64:
			PRINT_VAR(int64_t, "%" PRIi64, ptr, len - off);
			break;

		case TLM_TYPE_FLOAT32:
			PRINT_VAR(float, "%f", ptr, len - off);
			break;

		case TLM_TYPE_FLOAT64: {
			PRINT_VAR(double, "%f", ptr, len - off);
			break;
		}

		case TLM_TYPE_STRING:
			fprintf(stdout, "\'");
			for (j = 0; j < rec->size; j++) {
				if (ptr[j] == '\0')
					break;
				fprintf(stdout, "%c", ptr[j]);
			}
			fprintf(stdout, "\'");
			break;

		case TLM_TYPE_BINARY:
			for (j = 0; j < rec->size; j++)
				fprintf(stdout, "%02x", ptr[j]);
			break;

		default:
			break;
		}

		ptr += rec->size;
		off += rec->size;
		if (i != rec->count - 1)
			fprintf(stdout, ", ");
	}

	if (rec->count > 1)
		fprintf(stdout, "]");
	fprintf(stdout, "\n");
}

/**
 */
static void dump_sample(struct tlm_shm_ctx *ctx, uint32_t alignment,
		uint32_t idx, const struct tlm_shm_sample_hdr *hdr,
		const void *buf, size_t len)
{
	const void *varbuf = NULL;
	const struct tlm_shm_var_info *varinfo = NULL;
	uint32_t i = 0;

	fprintf(stdout, "----------\n");
	fprintf(stdout, "%0*d:ts=%ld.%09ld;exp=%ld.%09ld;seq=%d\n",
			alignment, idx,
			hdr->ts.tv_sec, hdr->ts.tv_nsec,
			hdr->exp.tv_sec, hdr->exp.tv_nsec,
			hdr->nb_writes);

	for (i = 0; i < ctx->varcount; i++) {
		varinfo = &ctx->varinfos[i];
		if (varinfo->rec != NULL) {
			varbuf = (const uint8_t *)buf + varinfo->off;
			dump_var(ctx, varinfo->rec, varbuf, varinfo->size);
		}
	}
}

/**
 */
static void dump_samples(struct tlm_shm_ctx *ctx)
{
	const void *buf = NULL;
	const struct tlm_shm_sample_hdr *hdr = NULL;
	size_t len = 0;
	uint32_t idx = 0;
	uint32_t alignment = 0;

	len = ALIGN_UP(sizeof(struct tlm_shm_sample_hdr) +
			ctx->section_hdr->blob_size);
	alignment =  floor(log10(ctx->section_hdr->max_nb_samples)) + 1;
	for (idx = 0; idx < ctx->section_hdr->max_nb_samples; idx++) {
		buf = (const uint8_t *)ctx->samples + idx * len;
		hdr = buf;
		dump_sample(ctx, alignment, idx, hdr,
				(const uint8_t *)buf + sizeof(*hdr),
				ctx->section_hdr->blob_size);
	}
}

/**
 */
static void dump_latest_sample(struct tlm_shm_ctx *ctx)
{
	const void *buf = NULL;
	const struct tlm_shm_sample_hdr *hdr = NULL;
	size_t len = 0;
	uint32_t idx = 0;
	uint32_t alignment = 0;

	len = ALIGN_UP(sizeof(struct tlm_shm_sample_hdr) +
			ctx->section_hdr->blob_size);
	alignment =  floor(log10(ctx->section_hdr->max_nb_samples)) + 1;
	if (ctx->section_hdr->write_index < 0) {
		fprintf(stdout, "No sample written yet\n");
		return;
	}
	idx = ctx->section_hdr->write_index;
	buf = (const uint8_t *)ctx->samples + idx * len;
	hdr = buf;

	dump_sample(ctx, alignment, idx, hdr,
			(const uint8_t *)buf + sizeof(*hdr),
			ctx->section_hdr->blob_size);
}

/**
 */
static void usage(const char *progname)
{
	fprintf(stderr, "usage: %s [<options>] <shmpath>\n", progname);
	fprintf(stderr, "Dump the contents of a shared memory section\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Global options:\n");
	fprintf(stderr, "  -h, --help        this help\n");
	fprintf(stderr, "  -l, --latest      show only latest sample\n");
	fprintf(stderr, "  -m, --monitor     monitor and dump new content\n");
	fprintf(stderr, "\n");
}

/**
 */
int main(int argc, char *argv[])
{
	int res = 0;
	int argidx = 0;
	int only_latest_sample = 0;
	int monitor = 0;
	const char *shmpath = NULL;
	struct tlm_shm_ctx *ctx = NULL;
	struct timespec sleep_duration;

	/* Parse options */
	for (argidx = 1; argidx < argc; argidx++) {
		if (argv[argidx][0] != '-') {
			/* End of options */
			break;
		} else if (strcmp(argv[argidx], "-l") == 0
				|| strcmp(argv[argidx], "--latest") == 0) {
			/* Latest sample */
			only_latest_sample = 1;
		} else if (strcmp(argv[argidx], "-m") == 0
				|| strcmp(argv[argidx], "--monitor") == 0) {
			/* Latest sample */
			monitor = 1;
			sleep_duration.tv_sec = 0;
			sleep_duration.tv_nsec = 100000000; /* 100 ms */
		} else if (strcmp(argv[argidx], "-h") == 0
				|| strcmp(argv[argidx], "--help") == 0) {
			/* Help */
			usage(argv[0]);
			goto out;
		} else {
			diag("Unknown option: '%s'", argv[argidx]);
			usage(argv[0]);
			goto error;
		}
	}

	if (argc - argidx >= 1) {
		shmpath = argv[argidx++];
	} else {
		diag("Missing shared memory path");
		goto error;
	}

	/* Open shared memory section */
	ctx = tlm_shm_open(shmpath);
	if (ctx == NULL)
		goto error;

	do {

		/* Read metadata header */
		if (tlm_shm_read_metadata_hdr(ctx) < 0)
			goto error;

		if (only_latest_sample) {
			/* Dump latest sample */
			dump_latest_sample(ctx);
		} else {
			/* Dump all*/
			dump_header(ctx);
			dump_samples(ctx);
		}

		if (!monitor)
			break;

		nanosleep(&sleep_duration, NULL);
	} while (1);

	goto out;

error:
	res = -1;
out:
	if (ctx != NULL)
		tlm_shm_close(ctx);
	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
