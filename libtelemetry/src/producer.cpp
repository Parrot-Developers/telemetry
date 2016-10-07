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
 * @file producer.cpp
 *
 */

#include "private.hpp"

namespace telemetry {
using namespace internal;

/**
 */
Producer::Producer(const std::string &section, const std::string &dir,
		uint32_t maxSamples, uint32_t rate)
{
	/* Save parameters */
	mSection = section;
	mDir = dir;
	mData = NULL;
	mDataSize = 0;
	mShdCtx = NULL;
	mMaxSamples = maxSamples;
	mRate = rate;
}

/**
 */
Producer::~Producer()
{
	/* Free resources */
	if (mData != NULL)
		free(mData);
	if (mShdCtx != NULL)
		shd_close(mShdCtx, NULL);
}

/**
 */
void Producer::setupVarInfoMap()
{
	/* Determine total size required for storage */
	mDataSize = 0;
	for (const auto &it : mVarDescMap) {
		const VarDesc &varDesc = it.second;
		mDataSize += varDesc.getTotalSize();
	}

	/* Allocate data for storage */
	mData = calloc(mDataSize, sizeof(uint8_t));
	assert(mData != NULL);

	/* Setup local data pointers. We can also setup pointers to variable
	 * description now that maps will not change anymore */
	size_t off = 0;
	for (const auto &it : mVarDescMap) {
		const VarDesc &varDesc = it.second;
		VarInfo &varInfo = mVarInfoMap.find(varDesc.getFullName())->second;
		varInfo.mPtrLocal = ((uint8_t *)mData) + off;
		varInfo.mVarDesc = &varDesc;
		off += varDesc.getTotalSize();
	}
	assert(off <= mDataSize);
}

/**
 */
void Producer::setupShd()
{
	/* Determine total size required for shared memory header */
	size_t hdrSize = 2 * sizeof(uint32_t);
	for (const auto &it : mVarDescMap) {
		const VarDesc &varDesc = it.second;
		size_t reclen = varDesc.getRecordLength();
		hdrSize += reclen;
	}

	/* Allocate storage for metadata header */
	void *hdr = malloc(hdrSize);
	assert(hdr != NULL);
	uint8_t *ptr = (uint8_t *)hdr;
	size_t off = 0;

	/* Magic */
	assert(off + sizeof(uint32_t) <= hdrSize);
	*((uint32_t *)ptr) = TLM_SHM_HDR_MAGIC;
	ptr += sizeof(uint32_t);
	off += sizeof(uint32_t);

	/* Number of variables */
	assert(off + sizeof(uint32_t) <= hdrSize);
	*((uint32_t *)ptr) = mVarDescMap.size();
	ptr += sizeof(uint32_t);
	off += sizeof(uint32_t);

	/* Variables */
	for (const auto &it : mVarDescMap) {
		const VarDesc &varDesc = it.second;
		size_t reclen = varDesc.getRecordLength();
		assert(off + reclen <= hdrSize);
		varDesc.writeRecord(ptr, reclen);
		ptr += reclen;
		off += reclen;
	}
	assert(off <= hdrSize);

	/* Setup shared memory info */
	struct shd_hdr_user_info info;
	memset(&info, 0, sizeof(info));
	info.blob_size = mDataSize;
	info.max_nb_samples = mMaxSamples;
	info.rate = mRate;
	info.blob_metadata_hdr_size = hdrSize;

	/* Create shared memory */
	mShdCtx = shd_create(mSection.c_str(), mDir.empty() ? NULL : mDir.c_str(), &info, hdr);
	if (mShdCtx == NULL) {
		ULOGE("Failed to create shared memory '%s'", mSection.c_str());
	}

	/* Free local header */
	free(hdr);
}

/**
 */
void Producer::regComplete()
{
	/* Nothing to do if already completed */
	if (mCompleted)
		return;

	setupVarInfoMap();
	setupShd();

	/* Registration completed, it is not possible to add more variables */
	mCompleted = true;
}

void Producer::reset()
{
	ULOGN("Reseting producer %s...", mSection.c_str());

	if (mData != NULL) {
		free(mData);
		mData = NULL;
	}
	if (mShdCtx != NULL) {
		shd_close(mShdCtx, NULL);
		mShdCtx = NULL;
	}

	setupVarInfoMap();
	setupShd();
}

/**
 */
void Producer::putSample(const struct timespec *timestamp)
{
	int res = 0;
	assert(mCompleted);

	/* Get original data and put it in our local storage */
	for (auto &it : mVarInfoMap) {
		VarInfo &varInfo = it.second;
		memcpy(varInfo.mPtrLocal, varInfo.mPtrOrig, varInfo.mVarDesc->getTotalSize());
	}

	/* Setup new sample */
	struct shd_sample_metadata metadata;
	memset(&metadata, 0, sizeof(metadata));
	if (timestamp == NULL)
		clock_gettime(CLOCK_MONOTONIC, &metadata.ts);
	else
		metadata.ts = *timestamp;

	/* Write new sample, if section was not successfully opened an error has
	 * already been reported, simply skip sharing data... */
	if (mShdCtx != NULL) {
		res = shd_write_new_blob(mShdCtx, mData, mDataSize, &metadata);
		if (res < 0) {
			ULOGE("shd_write_new_blob: err=%d(%s)", -res, strerror(-res));
		}
	}
}

} /* namespace telemetry */
