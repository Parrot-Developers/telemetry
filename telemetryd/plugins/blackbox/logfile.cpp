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
 * @file logfile.cpp
 *
 */

#include "headers.hpp"

/**
 */
LogFile::LogFile(const std::string &path, bool compressed)
{
	/* Initialize parameters */
	mPath = path;
	mFile = NULL;
	mBlockBeginOffset = 0;
	mCompressed = compressed;
	memset(&mZStream, 0, sizeof(mZStream));
}

/**
 */
LogFile::~LogFile()
{
	assert(mFile == NULL);
}

/**
 */
int LogFile::open()
{
	int res = 0;
	assert(mFile == NULL);

	/* Open file for writing */
	ULOGI("Creating file '%s'", mPath.c_str());
	mFile = fopen(mPath.c_str(), "wb");
	if (mFile == NULL) {
		res = -errno;
		ULOGE("Failed to create '%s': err=%d(%s)", mPath.c_str(),
				errno, strerror(errno));
		goto error;
	}

	/* Setup zlib stream if required */
	if (mCompressed) {
		res = deflateInit2(&mZStream, Z_DEFAULT_COMPRESSION,
				Z_DEFLATED, 15, 8, Z_DEFAULT_STRATEGY);
		if (res != Z_OK) {
			res = -EIO;
			ULOGW("deflateInit2: err=%d(%s)", res, mZStream.msg);
			goto error;
		}
	}

	return 0;

	/* Cleanup in case of error */
error:
	close();
	return res;
}

/**
 */
int LogFile::close()
{
	int res = 0;
	if (mFile != NULL) {
		ULOGI("Closing file '%s'", mPath.c_str());

		/* Release zlib stream */
		res = deflateEnd(&mZStream);
		if (res != Z_OK)
			ULOGE("deflateEnd: err=%d(%s)", res, mZStream.msg);

		/* Close the file */
		if (fclose(mFile) < 0)
			ULOGE("fclose: err=%d(%s)", errno, strerror(errno));
	}

	/* Reset state */
	mFile = NULL;
	memset(&mZStream, 0, sizeof(mZStream));

	/* Force success */
	return 0;
}

/**
 */
int LogFile::writeData(const void *buf, size_t len)
{
	if (mCompressed)
		return writeDataCompressed(buf, len, false);
	else
		return writeDataRaw(buf, len);
}

/**
 */
int LogFile::writeDataRaw(const void *buf, size_t len)
{
	int res = 0;
	if (mFile == NULL)
		return -EINVAL;

	/* Write data, closing file in case of error */
	if (fwrite(buf, 1, len, mFile) != len) {
		res = -errno;
		ULOGE("fwrite: err=%d(%s)", errno, strerror(errno));
		close();
	}

	return res;
}

/**
 */
int LogFile::writeDataCompressed(const void *buf, size_t len, bool flush)
{
	int res = 0;
	uint8_t outbuf[4096];
	if (mFile == NULL)
		return -EINVAL;

	/* Prepare next input buffer */
	assert(mZStream.avail_in == 0);
	mZStream.next_in = (uint8_t *)buf;
	mZStream.avail_in = len;

	/* Call compression function while the output buffer has been
	 * completely used. The last call can generate Z_BUF_ERROR if all
	 * input was used and the last buffer was exactly the max size given */
	do {
		/* Prepare next output buffer */
		mZStream.next_out = outbuf;
		mZStream.avail_out = sizeof(outbuf);
		res = deflate(&mZStream, flush ? Z_FINISH : Z_NO_FLUSH);
		if (res != Z_OK && res != Z_BUF_ERROR && res != Z_STREAM_END) {
			res = -EIO;
			ULOGE("deflate: err=%d(%s)", res, mZStream.msg);
		}
		size_t outlen = sizeof(outbuf) - mZStream.avail_out;
		if (outlen > 0)
			res = writeDataRaw(outbuf, outlen);
	} while (res == 0 && mZStream.avail_out == 0);

	/* All input data shall have been processed if no error */
	assert(res != 0 || mZStream.avail_in == 0);
	return res;
}

/**
 */
int LogFile::getCurrentBlockSize(uint32_t *size)
{
	int res = 0;
	long pos = 0;

	/* Get current position in file */
	res = getPos(&pos);
	if (res < 0)
		return res;

	*size = pos - mBlockBeginOffset - sizeof(uint32_t);
	return 0;
}

/**
 */
int LogFile::beginBlock()
{
	int res = 0;

	/* Remember offset of start of block */
	res = getPos(&mBlockBeginOffset);
	if (res < 0)
		return res;

	/* Write a fake block size */
	uint32_t blockSize = 0xffffffff;
	return writeDataRaw(&blockSize, sizeof(uint32_t));
}

/**
 */
int LogFile::finishBlock()
{
	int res = 0;
	uint32_t blockSize = 0;

	/* Finish compression of current block if needed */
	if (mCompressed && mZStream.total_in != 0) {
		/* Flush zlib stream*/
		res = writeDataCompressed(NULL, 0, true);
		if (res < 0)
			goto out;

		/* Reset stream so we can start a new block */
		res = deflateReset(&mZStream);
		if (res != Z_OK) {
			res = -EIO;
			ULOGE("deflateReset: err=%d(%s)", res, mZStream.msg);
			goto out;
		}
	}

	/* Get block size */
	res = getCurrentBlockSize(&blockSize);
	if (res < 0)
		goto out;

	/* Go back to offset of start of block */
	res = seek(mBlockBeginOffset, SEEK_SET);
	if (res < 0)
		goto out;

	/* Nothing more to do if not data was actually written */
	if (blockSize == 0)
		goto out;

	/* Overwrite the size of the block */
	res = writeDataRaw(&blockSize, sizeof(uint32_t));
	if (res < 0)
		goto out;

	/* Go back to end of block */
	res = seek(blockSize, SEEK_CUR);
	if (res < 0)
		goto out;

	/* Flush to disk */
	res = flush();
	if (res < 0)
		goto out;

	ULOGD("Written new block of size %u", blockSize);
out:
	return 0;
}

/**
 */
int LogFile::finishFile()
{
	int res = 0;
	uint32_t eof = 0xffffffff;

	/* Finish current block if any */
	res = finishBlock();
	if (res < 0)
		goto out;

	/* Write an eof marker (same as marker for pending block) */
	res = writeDataRaw(&eof, sizeof(uint32_t));
	if (res < 0)
		goto out;

out:
	return 0;
}

/**
 */
int LogFile::flush()
{
	int res = 0;
	int fd = -1;
	std::string dirPath;
	if (mFile == NULL)
		return -EINVAL;

	/* From libc to kernel */
	if (fflush(mFile) < 0) {
		res = -errno;
		ULOGE("fflush: err=%d(%s)", errno, strerror(errno));
		close();
		goto out;
	}

	/* From kernel to disk */
	if (fsync(fileno(mFile)) < 0) {
		res = -errno;
		ULOGE("fsync: err=%d(%s)", errno, strerror(errno));
		close();
		goto out;
	}

	/* Also sync directory */
	dirPath = mPath.substr(0, mPath.find_last_of('/'));
	fd = ::open(dirPath.c_str(), O_RDONLY);
	if (fd < 0 || fsync(fd) < 0) {
		res = -errno;
		ULOGE("Unable to sync directory '%s': err=%d(%s)",
				dirPath.c_str(), errno, strerror(errno));
		goto out;
	}

out:
	if (fd >= 0)
		::close(fd);
	return res;
}

/**
 */
int LogFile::seek(long off, int whence)
{
	int res = 0;
	if (mFile == NULL)
		return -EINVAL;

	/* Seek in file, closing file in case of error */
	if (fseek(mFile, off, whence) < 0) {
		res = -errno;
		ULOGE("fseek: err=%d(%s)", errno, strerror(errno));
		close();
	}

	return res;
}

/**
 */
int LogFile::getPos(long *pos)
{
	int res = 0;
	if (mFile == NULL)
		return -EINVAL;

	/* Get file position, closing file in case of error */
	assert(pos != NULL);
	*pos = ftell(mFile);
	if (*pos < 0) {
		res = -errno;
		ULOGE("ftell: err=%d(%s)", errno, strerror(errno));
		close();
	}

	return res;
}
