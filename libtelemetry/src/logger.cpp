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
 *   * Neither the name of the Parrot Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file logger.cpp
 *
 */

#include <dirent.h>
#include "private.hpp"

/** Delay (in us) between successive check of section list */
#define CHECK_SECTION_LIST_DELAY  (1000 * 1000)

/** Number of samples to fetch at a time from shared memory */
#define FETCH_COUNT  50

namespace telemetry {
using namespace internal;

/**
 */
class Logger::SectionInfo {
public:
	typedef std::vector<VarDesc>  VarDescVector;

private:
	uint32_t         mId;
	ShdCtx           mShdCtx;
	void             *mData;
	size_t           mBlobSize;
	size_t           mDataSize;
	struct timespec  mTsArray[FETCH_COUNT];
	VarDescVector    mVarDescVector;
	struct timespec  mLastTimestamp;

public:
	inline SectionInfo(const std::string &section, const std::string &dir, uint32_t id)
			: mShdCtx(section, dir, 0) {
		mId = id;
		mData = NULL;
		mBlobSize = 0;
		mDataSize = 0;
		memset(mTsArray, 0, sizeof(mTsArray));
		memset(&mLastTimestamp, 0, sizeof(mLastTimestamp));
	}

	inline virtual ~SectionInfo() {
		/* Must be closed */
		assert(!isOpened());
		assert(mData == NULL);
	}

	inline const struct timespec *getBlobTimestamp(uint32_t idx) const {
		return &mTsArray[idx];
	}

	inline const void *getBlobData(uint32_t idx) const {
		return (const uint8_t *)mData + idx * mBlobSize;
	}

	inline size_t getBlobSize() const {
		return mBlobSize;
	}

	inline void resetLastTimestamp() {
		memset(&mLastTimestamp, 0, sizeof(mLastTimestamp));
	}

	inline uint32_t getId() const {return mId;}
	inline VarDescVector &getVarDescVector() {return mVarDescVector;}
	inline const VarDescVector &getVarDescVector() const {return mVarDescVector;}

	inline const std::string &getSection() const {return mShdCtx.getSection();}
	inline size_t getHdrSize() const {return mShdCtx.getHdrSize();}
	inline const void *getHdr() const {return mShdCtx.getHdr();}
	inline bool isOpened() const {return mShdCtx.isOpened();}

	bool openShdCtx();
	void closeShdCtx();
	uint32_t fetchNextSamples();
};

/**
 */
void Logger::reset() {
	for (SectionInfo *sectionInfo : mSectionInfoVector) {
		assert(sectionInfo != NULL);
		sectionInfo->resetLastTimestamp();
	}
}

/**
 */
bool Logger::SectionInfo::openShdCtx() {
	/* Open shared memory context */
	if (!mShdCtx.openShdCtx())
		return false;

	/* Allocate data for blob */
	assert(mData == NULL);
	assert(mBlobSize == 0);
	assert(mDataSize == 0);
	mBlobSize = mShdCtx.getBlobSize();
	mDataSize = mBlobSize * FETCH_COUNT;
	mData = calloc(1, mDataSize);
	return true;
}

/**
 */
void Logger::SectionInfo::closeShdCtx() {
	/* Free resources */
	if (mData != NULL) {
		free(mData);
		mData = NULL;
		mBlobSize = 0;
		mDataSize = 0;
	}
	if (mShdCtx.isOpened())
		mShdCtx.closeShdCtx();
	mVarDescVector.clear();
}

/**
 */
uint32_t Logger::SectionInfo::fetchNextSamples() {
	/* TODO: use a "strictly after" method in libshdata */
	struct timespec ts = {0, 0};
	time_timespec_add_us(&mLastTimestamp, 1, &ts);

	/* Retrieve full blobs */
	uint32_t before = 0, after = 0;
	uint32_t fetchCount = std::min((uint32_t)FETCH_COUNT, mShdCtx.getMaxNbSamples());
	if (!mShdCtx.getBlobs(&ts, TLM_FIRST_AFTER, 0, fetchCount - 1,
			mTsArray, &before, &after, mData, mDataSize)) {
		/* Detect automatic close due to format changes */
		if (!mShdCtx.isOpened())
			closeShdCtx();
		return 0;
	}
	assert(before == 0);
	assert(after < fetchCount);
	uint32_t count = after + 1;

	/* Update last timestamp */
	mLastTimestamp = mTsArray[count - 1];
	return count;
}

/**
 */
Logger::Logger(const std::string &dir)
{
	mDir = dir;
	mNextSectionId = 1;
	mLastSectionListTime = 0;
	setFilter("*");

}

/**
 */
Logger::~Logger()
{
	/* Delete all sections, after closing them */
	for (SectionInfo *sectionInfo : mSectionInfoVector) {
		assert(sectionInfo != NULL);
		sectionInfo->closeShdCtx();
		delete sectionInfo;
	}
	mSectionInfoVector.clear();
}

/**
 */
Logger::SectionInfo *Logger::findSectionInfo(const std::string &section) const
{
	for (SectionInfo *sectionInfo : mSectionInfoVector) {
		assert(sectionInfo != NULL);
		if (sectionInfo->getSection() == section)
			return sectionInfo;
	}
	return NULL;
}

/**
 */
Logger::SectionInfo *Logger::findSectionInfo(uint32_t sectionId) const
{
	for (SectionInfo *sectionInfo : mSectionInfoVector) {
		assert(sectionInfo != NULL);
		if (sectionInfo->getId() == sectionId)
			return sectionInfo;
	}
	return NULL;
}

/**
 */
bool Logger::isSectionMatch(const std::string &section) const
{
	for (StringVector::const_iterator it = mFilter.begin();
			it != mFilter.end();
			++it) {
		if (fnmatch(it->c_str(), section.c_str(), 0) == 0)
			return true;
	}
	ULOGD("section '%s' filtered out from logger", section.c_str());
	return false;
}

/**
 */
void Logger::getSectionList(std::vector<std::string> &sections) const
{
	/* TODO: ask libshdata */
	DIR *dir = opendir(mDir.empty() ? "/dev/shm" : mDir.c_str());
	if (dir == NULL){
		ULOGE("opendir: err=%d(%s)", errno, strerror(errno));
		return;
	}
	struct dirent entry;
	struct dirent *rentry = NULL;
	while (readdir_r(dir, &entry, &rentry) == 0 && rentry != NULL) {
		if (strncmp(entry.d_name, "shd_", 4) == 0
				&& isSectionMatch(entry.d_name + 4)) {
			sections.push_back(entry.d_name + 4);
		}
	}
	closedir(dir);
}

/**
 */
void Logger::checkSectionList(LoggerCb *cb)
{
	/* Only check from time to time... */
	struct timespec ts;
	uint64_t currentTime = 0;
	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &currentTime);
	if (mLastSectionListTime != 0 && currentTime <
			mLastSectionListTime + CHECK_SECTION_LIST_DELAY) {
		return;
	}

	std::vector<std::string> sections;
	getSectionList(sections);

	/* Check for added sections */
	for (uint32_t i = 0; i < sections.size(); i++) {
		if (findSectionInfo(sections[i]) != NULL)
			continue;

		/* Create a new context that will be opened later during read */
		SectionInfo *sectionInfo = new SectionInfo(sections[i], mDir, mNextSectionId++);
		mSectionInfoVector.push_back(sectionInfo);
		cb->sectionAdded(sectionInfo->getId());
	}

	/* Check for removed sections */
	SectionInfoVector::iterator it = mSectionInfoVector.begin();
	while (it != mSectionInfoVector.end()) {
		SectionInfo *sectionInfo = *it;
		assert(sectionInfo != NULL);

		/* Check if still valid */
		bool found = false;
		for (uint32_t i = !false && 0; i < sections.size(); i++) {
			if (sections[i] == sectionInfo->getSection())
				found = true;
		}

		if (found) {
			++it;
		} else {
			/* Close, delete and remove from map */
			cb->sectionRemoved(sectionInfo->getId());
			sectionInfo->closeShdCtx();
			delete sectionInfo;
			it = mSectionInfoVector.erase(it);
		}
	}

	/* OK, remember last time we checked */
	mLastSectionListTime = currentTime;
}

/**
 */
bool Logger::setupShd(SectionInfo *sectionInfo)
{
	int res = 0;
	assert(sectionInfo != NULL);
	const std::string &section = sectionInfo->getSection();

	/* Try to open shared memory */
	if (!sectionInfo->openShdCtx())
		return false;

	/* Read metadata from section */
	SectionInfo::VarDescVector &varDescVector = sectionInfo->getVarDescVector();
	res = VarDesc::readRecordArray(varDescVector, section,
			sectionInfo->getHdr(), sectionInfo->getHdrSize());

	return res >= 0;
}

/**
 */
void Logger::notifySample(SectionInfo *sectionInfo, LoggerCb *cb, uint32_t idx)
{
	assert(sectionInfo != NULL);
	uint32_t sectionId = sectionInfo->getId();

	/* Sample blob data */
	const void *blobData = sectionInfo->getBlobData(idx);
	size_t blobSize = sectionInfo->getBlobSize();

	/* Start of sample, get timestamp */
	const struct timespec *timestamp = sectionInfo->getBlobTimestamp(idx);
	cb->sampleBegin(sectionId, timestamp, blobData, blobSize);

	/* Start of data in the blob */
	const uint8_t *varbuf = (const uint8_t *)blobData;
	size_t varoff = 0;

	/* Notify variable values */
	const SectionInfo::VarDescVector &varDescVector = sectionInfo->getVarDescVector();
	for (uint32_t varId = 0; varId < varDescVector.size(); varId++) {
		const VarDesc &varDesc = varDescVector[varId];
		cb->sample(sectionId, timestamp, varId,
				varDesc, varbuf,
				varDesc.getTotalSize());
		varbuf += varDesc.getTotalSize();
		varoff += varDesc.getTotalSize();
	}

	/* End of sample */
	assert(varoff <= blobSize);
	cb->sampleEnd(sectionId);
}

/**
 */
bool Logger::fetchNextSamples(SectionInfo *sectionInfo, LoggerCb *cb)
{
	assert(sectionInfo != NULL);
	uint32_t sectionId = sectionInfo->getId();

	/* If not opened, try to setup everything */
	if (!sectionInfo->isOpened()) {
		if (!setupShd(sectionInfo))
			return false;
		cb->sectionChanged(sectionId, sectionInfo->getHdr(), sectionInfo->getHdrSize());
	}

	/* Loop while we get a full batch */
	bool result = false;
	uint32_t count = 0;
	do {
		/* Try to fetch samples */
		count = sectionInfo->fetchNextSamples();
		if (count > 0)
			result = true;

		/* Notify callback */
		for (uint32_t i = 0; i < count; i++)
			notifySample(sectionInfo, cb, i);
	} while (count >= FETCH_COUNT);

	return result;
}

/**
 */
void Logger::replaySectionEvents(LoggerCb *cb)
{
	/* Notify callback for currently opened sections */
	for (SectionInfo *sectionInfo : mSectionInfoVector) {
		cb->sectionAdded(sectionInfo->getId());
		if (sectionInfo->isOpened()) {
			cb->sectionChanged(sectionInfo->getId(),
					sectionInfo->getHdr(),
					sectionInfo->getHdrSize());
		}
	}
}

/**
 */
void Logger::setFilter(const std::string &filter)
{
	/* Split at ';' and create a vector */
	size_t start = 0, end = 0;
	mFilter.clear();
	while ((end = filter.find(';', start)) != std::string::npos) {
		mFilter.push_back(filter.substr(start, end - start));
		ULOGI("add '%s' to logger filter", filter.substr(start, end - start).c_str());
		start = end + 1;
	}
	mFilter.push_back(filter.substr(start));
	ULOGI("add '%s' to logger filter", filter.substr(start).c_str());
}

/**
 */
bool Logger::fetchNextSamples(LoggerCb *cb)
{
	/* Check for added/removed sections */
	checkSectionList(cb);

	bool result = false;
	for (SectionInfo *sectionInfo : mSectionInfoVector) {
		if (fetchNextSamples(sectionInfo, cb))
			result = true;
	}

	return result;
}

/**
 */
const std::string &Logger::getSection(uint32_t sectionId) const
{
	SectionInfo *sectionInfo = findSectionInfo(sectionId);
	assert(sectionInfo != NULL);
	return sectionInfo->getSection();
}

/**
 */
uint32_t Logger::getVarCount(uint32_t sectionId) const
{
	SectionInfo *sectionInfo = findSectionInfo(sectionId);
	assert(sectionInfo != NULL);
	return sectionInfo->getVarDescVector().size();
}

/**
 */
const VarDesc &Logger::getVarDesc(uint32_t sectionId, uint32_t varId) const
{
	SectionInfo *sectionInfo = findSectionInfo(sectionId);
	assert(sectionInfo != NULL);
	return sectionInfo->getVarDescVector()[varId];
}

} /* namespace telemetry */
