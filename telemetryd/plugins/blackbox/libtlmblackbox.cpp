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
 * @file libtlmblackbox.cpp
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <map>
#include <vector>

#include <zlib.h>

#define ULOG_TAG tlmblackbox
#include "ulog.h"

#include "libtlmblackbox.hpp"
#include "tlmb.hpp"

ULOG_DECLARE_TAG(tlmblackbox);

namespace telemetry {
namespace internal {

/** */
class TlmbDataReader {
protected:
	virtual void doClear() = 0;
	virtual size_t doGetRemaining() const = 0;
	virtual int doReadData(void *data, size_t size) = 0;

public:
	inline TlmbDataReader() {}
	inline virtual ~TlmbDataReader() {}

	inline void clear()
	{
		doClear();
	}

	inline size_t getRemaining() const
	{
		return doGetRemaining();
	}

	inline int readData(void *data, size_t size)
	{
		int res = 0;
		res = doReadData(data, size);
		if (res < 0)
			ULOGE("Unable to read %zu bytes", size);
		return res;
	}

	inline int readU32(uint32_t *val)
	{
		return readData(val, sizeof(*val));
	}

	inline int readU8(uint8_t *val)
	{
		return readData(val, sizeof(*val));
	}

	inline int readStr(std::string *val)
	{
		int res = 0;
		char *str = NULL;
		uint8_t len = 0;

		/* String length (including final NUL) */
		res = readU8(&len);
		if (res < 0)
			goto out;
		if (len == 0) {
			res = -EIO;
			goto out;
		}

		/* Avoid allocating memory if the read will fail */
		if (len > getRemaining()) {
			res = -EIO;
			goto out;
		}

		str = (char *)calloc(len, 1);
		if (str == NULL) {
			res = -ENOMEM;
			goto out;
		}

		res = readData(str, len);
		if (res < 0)
			goto out;

		/* Check that we have a NUL byte */
		if (str[len - 1] != '\0') {
			res = -EIO;
			goto out;
		}

		val->assign(str);

	out:
		free(str);
		return res;
	}

	inline int readBuffer(void **outData, size_t size)
	{
		int res = 0;
		void *data = NULL;

		/* Avoid allocating memory if the read will fail */
		if (size > getRemaining())
			return -EIO;

		data = calloc(1, size);
		if (data == NULL) {
			res = -ENOMEM;
			goto error;
		}

		res = readData(data, size);
		if (res < 0)
			goto error;

		*outData = data;
		return 0;

	error:
		free(data);
		return res;
	}

	inline int readTimestamp(struct timespec *ts)
	{
		int res = 0;
		uint32_t sec = 0, nsec = 0;

		res = readU32(&sec);
		if (res < 0)
			goto out;

		res = readU32(&nsec);
		if (res < 0)
			goto out;

		ts->tv_sec = sec;
		ts->tv_nsec = nsec;

	out:
		return res;
	}
};

class TlmbDataReaderFile : public TlmbDataReader {
private:
	FILE    *mFile;
	size_t  mSize;

protected:
	inline virtual void doClear()
	{
		mFile = NULL;
		mSize = 0;
	}

	inline virtual size_t doGetRemaining() const
	{
		return mSize - ftell(mFile);
	}

	inline virtual int doReadData(void *data, size_t size)
	{
		return fread(data, 1, size, mFile) == size ? 0 : -EIO;
	}

public:
	inline TlmbDataReaderFile()
	{
		mFile = NULL;
		mSize = 0;
	}

	inline TlmbDataReaderFile(FILE *file)
	{
		mFile = file;
		long orig = ftell(mFile);
		mSize = fseek(mFile, 0, SEEK_END);
		fseek(mFile, orig, SEEK_SET);
	}
};

/** */
class TlmbDataReaderBuffer : public TlmbDataReader {
private:
	const void  *mBuffer;
	size_t      mSize;
	size_t      mReadOffset;

protected:
	inline virtual void doClear()
	{
		mBuffer = NULL;
		mSize = 0;
		mReadOffset = 0;
	}

	inline virtual size_t doGetRemaining() const
	{
		return mSize - mReadOffset;
	}

	inline virtual int doReadData(void *data, size_t size)
	{
		if (mSize - mReadOffset < size)
			return -EIO;
		memcpy(data, (const uint8_t *)mBuffer + mReadOffset, size);
		mReadOffset += size;
		return 0;
	}

public:
	inline TlmbDataReaderBuffer()
	{
		mBuffer = NULL;
		mSize = 0;
		mReadOffset = 0;
	}

	inline TlmbDataReaderBuffer(const void *buffer, size_t size)
	{
		mBuffer = buffer;
		mSize = size;
		mReadOffset = 0;
	}
};

/** */
class TlmbFile {
private:
	typedef std::vector<telemetry::VarDesc> VarDescVector;

	class SectionInfo {
	public:
		uint32_t       mSectionId;
		std::string    mSection;
		VarDescVector  mVarDescVector;
		inline SectionInfo() : mSectionId(0) {}
	};

	typedef std::map<uint32_t, SectionInfo> SectionInfoMap;
	typedef BlackboxReaderItfCb::PropertyMap PropertyMap;

private:
	TlmbDataReader       *mReader;
	BlackboxReaderItfCb  *mCb;
	uint32_t             mMagic;
	uint8_t              mVersion;
	bool                 mCompressed;
	PropertyMap          mPropertyMap;
	SectionInfoMap       mSectionInfoMap;

	struct {
		TlmbDataReaderBuffer  mReader;
		void                  *mData;
		uint32_t              mSize;
	} mCurrentBlock;

private:
	void clearCurrentBlock();
	int loadNextBlock();
	int readNextSampleInBlock();
	int uncompress(const void *inputData, size_t inputSize,
			void **outputData, size_t *outputSize);
	int processSectionAdded();
	int processSectionRemoved();
	int processSectionChanged();
	int processSample();

public:
	TlmbFile(TlmbDataReader *reader, BlackboxReaderItfCb *cb);
	~TlmbFile();

	int readHeader();
	int readNextSample();

	const std::string &getSection(uint32_t sectionId) const;
	uint32_t getVarCount(uint32_t sectionId) const;
	const VarDesc &getVarDesc(uint32_t sectionId, uint32_t varId) const;
};

/**
 */
TlmbFile::TlmbFile(TlmbDataReader *reader, BlackboxReaderItfCb *cb)
{
	mReader = reader;
	mCb = cb;
	mMagic = 0;
	mVersion = 0;
	mCompressed = false;
	mCurrentBlock.mData = NULL;
	mCurrentBlock.mSize = 0;
}

/**
 */
TlmbFile::~TlmbFile()
{
	clearCurrentBlock();
}

/**
 */
int TlmbFile::readHeader()
{
	int res = 0;
	uint32_t propertyCount = 0;
	std::string key, val;
	PropertyMap::const_iterator it;

	/* Read magic and version */
	res = mReader->readU32(&mMagic);
	if (res < 0)
		goto error;
	res = mReader->readU8(&mVersion);
	if (res < 0)
		goto error;

	/* Check validity */
	if (mMagic != TLMB_MAGIC) {
		res = -EINVAL;
		ULOGE("Bad magic: 0x%08x(0x%08x)", mMagic, TLMB_MAGIC);
		goto error;
	}
	if (mVersion != TLMB_VERSION) {
		res = -EINVAL;
		ULOGE("Bad version: %u", mVersion);
		goto error;
	}

	/* Get number of properties in header */
	res = mReader->readU32(&propertyCount);
	if (res < 0)
		goto error;

	/* Read properties */
	ULOGD("%u properties", propertyCount);
	for (uint32_t i = 0; i < propertyCount; i++) {
		/* Read key and value */
		res = mReader->readStr(&key);
		if (res < 0)
			goto error;
		res = mReader->readStr(&val);
		if (res < 0)
			goto error;

		mPropertyMap.insert(PropertyMap::value_type(key, val));
		ULOGD("  %s=%s", key.c_str(), val.c_str());
	}

	/* Check if compressed */
	it = mPropertyMap.find("compression");
	if (it != mPropertyMap.end())
		mCompressed = it->second == "1";
	ULOGD("compressed=%d", mCompressed);

	mCb->fileHeader(mPropertyMap);
	return 0;

error:
	return res;
}

/**
 */
int TlmbFile::readNextSample()
{
	int res = 0;

	/* Load next block if needed */
	if (mCurrentBlock.mData == NULL) {
		res = loadNextBlock();
		if (res < 0 && res != -ENOENT) {
			ULOGE("loadNextBlock: err=%d(%s)", res, strerror(-res));
			goto out;
		}
	}

	while (true) {
		res = readNextSampleInBlock();
		if (res == -ENOENT) {
			/* No more data in current block, load next one */
			clearCurrentBlock();
			res = loadNextBlock();
			if (res < 0)
				goto out;
		} else {
			/* Either error or found */
			if (res < 0)
				ULOGE("readNextSampleInBlock: err=%d(%s)", res, strerror(-res));
			goto out;
		}
	}

out:
	return res;
}

/**
 */
const std::string &TlmbFile::getSection(uint32_t sectionId) const
{
	SectionInfoMap::const_iterator it = mSectionInfoMap.find(sectionId);
	assert(it != mSectionInfoMap.end());
	return it->second.mSection;
}

/**
 */
uint32_t TlmbFile::getVarCount(uint32_t sectionId) const
{
	SectionInfoMap::const_iterator it = mSectionInfoMap.find(sectionId);
	assert(it != mSectionInfoMap.end());
	return it->second.mVarDescVector.size();
}

/**
 */
const VarDesc &TlmbFile::getVarDesc(uint32_t sectionId, uint32_t varId) const
{
	SectionInfoMap::const_iterator it = mSectionInfoMap.find(sectionId);
	assert(it != mSectionInfoMap.end());
	return it->second.mVarDescVector[varId];
}

/**
 */
void TlmbFile::clearCurrentBlock()
{
	free(mCurrentBlock.mData);
	mCurrentBlock.mData = NULL;
	mCurrentBlock.mSize = 0;
}

/**
 */
int TlmbFile::loadNextBlock()
{
	int res = 0;
	void *data = NULL;
	uint32_t dataSize = 0;

	/* Stop at end of reader */
	if (mReader->getRemaining() < 4)
		return -ENOENT;

	/* Read block size, check for end of file marker */
	res = mReader->readU32(&dataSize);
	if (res < 0)
		goto error;
	if (dataSize == 0xffffffff) {
		res = -ENOENT;
		ULOGD("Last block found");
		goto error;
	}

	ULOGD("Reading block of size %u", dataSize);
	res = mReader->readBuffer(&data, dataSize);
	if (res < 0)
		goto error;

	/* Uncompress if required */
	if (mCompressed) {
		void *uncompressedData = NULL;
		size_t uncompressedSize = 0;

		res = uncompress(data, dataSize, &uncompressedData, &uncompressedSize);
		if (res < 0)
			goto error;

		/* Free original data and replace by uncompressed one */
		free(data);
		data = uncompressedData;
		dataSize = uncompressedSize;
	}

	/* Save block context */
	assert(mCurrentBlock.mData == NULL);
	mCurrentBlock.mData = data;
	mCurrentBlock.mSize = dataSize;
	mCurrentBlock.mReader = TlmbDataReaderBuffer(data, dataSize);
	return 0;

error:
	free(data);
	return res;
}

/**
 */
int TlmbFile::readNextSampleInBlock()
{
	int res = 0;
	uint8_t tag = 0;

	while (true) {
		if (mCurrentBlock.mReader.getRemaining() < sizeof(uint8_t)) {
			res = -ENOENT;
			goto out;
		}

		res = mCurrentBlock.mReader.readU8(&tag);
		if (res < 0)
			goto out;

		switch ((TLMB_TAG)tag) {
		case TLMB_TAG_SECTION_ADDED:
			res = processSectionAdded();
			if (res < 0)
				goto out;
			break;

		case TLMB_TAG_SECTION_REMOVED:
			res = processSectionRemoved();
			if (res < 0)
				goto out;
			break;

		case TLMB_TAG_SECTION_CHANGED: {
			res = processSectionChanged();
			if (res < 0)
				goto out;
			break;
		}

		case TLMB_TAG_SAMPLE:
			res = processSample();
			goto out;

		default:
			ULOGE("Invalid tag %02X", tag);
			res = -EIO;
			goto out;
		}
	}

out:
	return res;
}

/**
 */
int TlmbFile::uncompress(const void *inputData, size_t inputSize,
		void **outputData, size_t *outputSize)
{
	/* Block size taken from Python doc */
	int res = 0;
	const size_t BLOCKSIZE = 16384;
	void *data = NULL;
	size_t dataSize = 0;
	z_stream stream;

	data = calloc(1, BLOCKSIZE);
	if (data == NULL)
		return -ENOMEM;
	dataSize = BLOCKSIZE;

	/* Init zlib */
	memset(&stream, 0, sizeof(stream));
	res = inflateInit(&stream);
	if (res != Z_OK) {
		ULOGE("inflateInit err=%d(%s)", res, stream.msg);
		res = -EIO;
		goto clear_data;
	}

	stream.next_in = (Bytef *)inputData;
	stream.avail_in = inputSize;
	stream.next_out = (Bytef *)data;
	stream.avail_out = dataSize;

	do {
		if (stream.avail_out == 0) {
			void *newData = NULL;

			newData = realloc(data, dataSize + BLOCKSIZE);
			if (newData == NULL) {
				res = -ENOMEM;
				goto clear_stream;
			}

			data = newData;
			stream.next_out = (Bytef *)data + dataSize;
			stream.avail_out = BLOCKSIZE;
			dataSize += BLOCKSIZE;
		}

		res = inflate(&stream, Z_NO_FLUSH);
		if (res != Z_OK && res != Z_STREAM_END) {
			ULOGE("inflate err=%d(%s)", res, stream.msg);
			res = -EIO;
			goto clear_stream;
		}
	} while (res != Z_STREAM_END);

	res = inflateEnd(&stream);
	if (res != Z_OK) {
		ULOGE("inflateEnd err=%d(%s)", res, stream.msg);
		res = -EIO;
		goto clear_data;
	}

	ULOGD("Uncompressed %zu bytes", dataSize);
	*outputData = data;
	*outputSize = dataSize - stream.avail_out;
	return 0;

clear_stream:
	inflateEnd(&stream);
clear_data:
	free(data);
	return res;
}

/**
 */
int TlmbFile::processSectionAdded()
{
	int res = 0;
	TlmbDataReader *reader = &mCurrentBlock.mReader;
	uint32_t sectionId = 0;
	std::string section;
	SectionInfo sectionInfo;

	res = reader->readU32(&sectionId);
	if (res < 0)
		goto out;
	res = reader->readStr(&section);
	if (res < 0)
		goto out;

	sectionInfo.mSectionId = sectionId;
	sectionInfo.mSection = section;
	mSectionInfoMap.insert(SectionInfoMap::value_type(sectionId, sectionInfo));
	mCb->sectionAdded(sectionId);

out:
	return res;
}

/**
 */
int TlmbFile::processSectionRemoved()
{
	int res = 0;
	TlmbDataReader *reader = &mCurrentBlock.mReader;
	uint32_t sectionId = 0;
	SectionInfoMap::iterator it;

	res = reader->readU32(&sectionId);
	if (res < 0)
		goto out;

	it = mSectionInfoMap.find(sectionId);
	if (it != mSectionInfoMap.end()) {
		mCb->sectionRemoved(sectionId);
		mSectionInfoMap.erase(it);
	}

out:
	return res;
}

/**
 */
int TlmbFile::processSectionChanged()
{
	int res = 0;
	TlmbDataReader *reader = &mCurrentBlock.mReader;
	uint32_t sectionId = 0;
	void *data = NULL;
	uint32_t dataSize = 0;
	SectionInfoMap::iterator it;

	res = reader->readU32(&sectionId);
	if (res < 0)
		goto out;

	res = reader->readU32(&dataSize);
	if (res < 0)
		goto out;

	res = reader->readBuffer(&data, dataSize);
	if (res < 0)
		goto out;

	it = mSectionInfoMap.find(sectionId);
	if (it != mSectionInfoMap.end()) {
		SectionInfo &sectionInfo = it->second;
		sectionInfo.mVarDescVector.clear();
		telemetry::VarDesc::readRecordArray(sectionInfo.mVarDescVector,
				sectionInfo.mSection, data, dataSize);
		mCb->sectionChanged(sectionId, data, dataSize);
	}

out:
	free(data);
	return res;
}

/**
 */
int TlmbFile::processSample()
{
	int res = 0;
	TlmbDataReader *reader = &mCurrentBlock.mReader;
	uint32_t sectionId = 0;
	struct timespec timestamp = {0, 0};
	void *data = NULL;
	uint32_t dataSize = 0;
	SectionInfoMap::iterator it;

	res = reader->readU32(&sectionId);
	if (res < 0)
		goto out;

	res = reader->readTimestamp(&timestamp);
	if (res < 0)
		goto out;

	res = reader->readU32(&dataSize);
	if (res < 0)
		goto out;

	res = reader->readBuffer(&data, dataSize);
	if (res < 0)
		goto out;

	it = mSectionInfoMap.find(sectionId);
	if (it != mSectionInfoMap.end()) {
		const uint8_t *varbuf = (const uint8_t *)data;
		size_t varoff = 0;
		SectionInfo &sectionInfo = it->second;
		VarDescVector &varDescVector = sectionInfo.mVarDescVector;
		mCb->sampleBegin(sectionId, &timestamp, data, dataSize);
		for (uint32_t varId = 0; varId < varDescVector.size(); varId++) {
			const telemetry::VarDesc &varDesc = varDescVector[varId];
			if (varoff + varDesc.getTotalSize() > dataSize) {
				ULOGW("Buffer too small");
				break;
			}
			mCb->sample(sectionId, &timestamp, varId,
					varDesc, varbuf,
					varDesc.getTotalSize());
			varbuf += varDesc.getTotalSize();
			varoff += varDesc.getTotalSize();
		}
		mCb->sampleEnd(sectionId);
	}

out:
	free(data);
	return res;
}

/**
 */
class BlackboxReaderImpl : public BlackboxReaderItf {
private:
	BlackboxReaderItfCb  *mCb;
	FILE                 *mFile;
	TlmbFile             *mTlmbFile;
	TlmbDataReaderFile   mReader;

public:
	BlackboxReaderImpl(BlackboxReaderItfCb *cb);
	virtual ~BlackboxReaderImpl();

	virtual int open(const char *path);
	virtual void close();
	virtual int readAll();
	virtual int readNext();
	virtual const std::string &getSection(uint32_t sectionId) const;
	virtual uint32_t getVarCount(uint32_t sectionId) const;
	virtual const VarDesc &getVarDesc(uint32_t sectionId, uint32_t varId) const;
};

/**
 */
BlackboxReaderImpl::BlackboxReaderImpl(BlackboxReaderItfCb *cb)
{
	mCb = cb;
	mFile = NULL;
	mTlmbFile = NULL;
}

/**
 */
BlackboxReaderImpl::~BlackboxReaderImpl()
{
	/* Close current file if any */
	if (mFile != NULL)
		close();
	assert(mFile == NULL);
}

/**
 */
int BlackboxReaderImpl::open(const char *path)
{
	int res = 0;

	/* Close current file if any */
	if (mFile != NULL)
		close();
	assert(mFile == NULL);

	/* Open given path */
	mFile = fopen(path, "rb");
	if (mFile == NULL) {
		res = -errno;
		ULOGE("Failed to open '%s': err=%d(%s)", path,
				errno, strerror(errno));
		return res;
	}

	mReader = TlmbDataReaderFile(mFile);
	mTlmbFile = new TlmbFile(&mReader, mCb);
	res = mTlmbFile->readHeader();
	if (res < 0)
		close();

	return res;
}

/**
 */
void BlackboxReaderImpl::close()
{
	mReader.clear();

	if (mTlmbFile != NULL) {
		delete mTlmbFile;
		mTlmbFile = NULL;
	}

	if (mFile != NULL) {
		fclose(mFile);
		mFile = NULL;
	}
}

/**
 */
int BlackboxReaderImpl::readAll()
{
	int res = 0;
	while ((res = readNext()) == 0)
		;
	return res == -ENOENT ? 0 : res;
}

/**
 */
int BlackboxReaderImpl::readNext()
{
	assert(mTlmbFile != NULL);
	return mTlmbFile->readNextSample();
}

/**
 */
const std::string &BlackboxReaderImpl::getSection(uint32_t sectionId) const
{
	assert(mTlmbFile != NULL);
	return mTlmbFile->getSection(sectionId);
}

/**
 */
uint32_t BlackboxReaderImpl::getVarCount(uint32_t sectionId) const
{
	assert(mTlmbFile != NULL);
	return mTlmbFile->getVarCount(sectionId);
}

/**
 */
const VarDesc &BlackboxReaderImpl::getVarDesc(uint32_t sectionId, uint32_t varId) const
{
	assert(mTlmbFile != NULL);
	return mTlmbFile->getVarDesc(sectionId, varId);
}

} /* namespace internal */

/**
 */
BlackboxReaderItf *BlackboxReaderItf::create(BlackboxReaderItfCb *cb)
{
	return new internal::BlackboxReaderImpl(cb);
}

} /* namespace telemetry */
