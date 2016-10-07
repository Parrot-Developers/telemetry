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
 * @file consumer.cpp
 *
 */

#include "private.hpp"

namespace telemetry {
using namespace internal;

/**
 */
Consumer::Consumer(const std::string &dir)
{
	/* Save parameters */
	mDir = dir;
}

/**
 */
Consumer::~Consumer()
{
	/* Delete all contexts, after closing them */
	for (const auto &it : mShdCtxPtrMap) {
		ShdCtx *shdCtx = it.second;
		shdCtx->closeShdCtx();
		delete shdCtx;
	}
	mShdCtxPtrMap.clear();
}

/**
 */
void Consumer::regComplete()
{
	/* Nothing to do if already completed */
	if (mCompleted)
		return;

	/* Setup variable info */
	for (const auto &it : mVarDescMap) {
		const VarDesc &varDesc = it.second;

		/* Setup variable description in variable info */
		VarInfo &varInfo = mVarInfoMap.find(varDesc.getFullName())->second;
		varInfo.mVarDesc = &varDesc;

		/* Add variable info in map by section, the first variable of a
		 * section will insert a new entry in the map */
		VarInfoPtrVector &varInfoPtrVector = mVarInfoMapBySection[varDesc.getSection()];
		varInfoPtrVector.push_back(&varInfo);
	}

	/* Setup shared memory context info, it will be opened later */
	for (const auto &it : mVarInfoMapBySection) {
		const std::string section = it.first;
		const VarInfoPtrVector &varInfoPtrVector = it.second;

		ShdCtx *shdCtx = new ShdCtx(section, mDir, varInfoPtrVector.size());
		assert(shdCtx != NULL);
		mShdCtxPtrMap.insert(ShdCtxPtrMap::value_type(section, shdCtx));
	}

	/* TODO: check that a variable was not registered twice with the same
	 * destination pointer (to avoid hard to find bugs in user code) */

	/* Registration completed, it is not possible to add more variables */
	mCompleted = true;
}

/**
 */
bool Consumer::setupShdCtx(ShdCtx *shdCtx)
{
	int res = 0;
	assert(shdCtx != NULL);
	const std::string &section = shdCtx->getSection();

	/* Try to open shared memory */
	if (!shdCtx->openShdCtx())
		return false;

	/* Read metadata from section */
	std::vector<VarDesc> varDescVector;
	res = VarDesc::readRecordArray(varDescVector, section,
			shdCtx->getHdr(), shdCtx->getHdrSize());
	if (res < 0)
		return false;

	/* Process all descriptions */
	size_t qtyOffset = 0;
	for (uint32_t i = 0; i < varDescVector.size(); i++) {
		const VarDesc &varDescRead = varDescVector[i];

		/* If this is a variable we are interested in */
		auto it = mVarDescMap.find(varDescRead.getFullName());
		auto it2 = mVarInfoMap.find(varDescRead.getFullName());
		if (it != mVarDescMap.end() && it2 != mVarInfoMap.end()) {
			VarDesc &varDesc = it->second;
			VarInfo &varInfo = it2->second;

			/* Check that variable description is ok */
			bool ok = (varDesc.getSection() == varDescRead.getSection() &&
					varDesc.getName() == varDescRead.getName() &&
					varDesc.getType() == varDescRead.getType() &&
					varDesc.getSize() == varDescRead.getSize() &&
					varDesc.getCount() == varDescRead.getCount());
			if (!ok) {
				/* Skip this variable but try other in the section */
				ULOGW("Variable description mismatch for '%s'",
						varDesc.getFullName().c_str());
				break;
			}

			/* Update other fields from read data */
			varDesc = varDescRead;

			/* Setup quantity parameters */
			shdCtx->addQty(varInfo.mPtrOrig, qtyOffset,
					varDesc.getTotalSize());
		}

		/* Shall be done for ALL variable descriptions read from header */
		qtyOffset += varDescRead.getTotalSize();
	}

	/* Check that all the variables we were supposed to find in this
	 * section were indeed found */
	const VarInfoPtrVector &varInfoPtrVector = mVarInfoMapBySection[section];
	for (const VarInfo *varInfo : varInfoPtrVector) {
		bool foundInSection = false;

		/* Check the pointer of destination to determine if variable was found */
		for (uint32_t i = 0; i < shdCtx->getQtyCount(); i++) {
			if (varInfo->mPtrOrig == shdCtx->getQtySample(i)->ptr) {
				foundInSection = true;
				break;
			}
		}

		if (!foundInSection) {
			ULOGW("Could not find var '%s' in section '%s'",
					varInfo->mVarDesc->getName().c_str(),
					section.c_str());
		} else {
			ULOGD("Successfully found var '%s' in section '%s'",
					varInfo->mVarDesc->getName().c_str(),
					section.c_str());
		}
	}

	return true;
}

/**
 */
bool Consumer::getSample(ShdCtx *shdCtx, const struct timespec *timestamp, Method method)
{
	assert(shdCtx != NULL);
	const std::string &section = shdCtx->getSection();

	/* If not opened, try to setup everything */
	if (!shdCtx->isOpened() && !setupShdCtx(shdCtx))
		return false;

	/* If no variable in this section still do a check in case a new format
	 * of data becomes available */
	if (shdCtx->getQtyCount() == 0)
		return shdCtx->checkRevision();

	/* Try to get the data */
	if (!shdCtx->getSample(timestamp, method))
		return false;

	/* Get read timestamp (assume all read quantities share the same) */
	const struct shd_sample_metadata *metadata = shdCtx->getQtySampleMetadata(0);

	/* Set it to read variables */
	VarInfoPtrVector &varInfoPtrVector = mVarInfoMapBySection[section];
	for (VarInfo *varInfo : varInfoPtrVector) {
		if (varInfo->mTimestamp != NULL)
			*varInfo->mTimestamp = metadata->ts;
	}

	return true;
}

/**
 */
bool Consumer::getSample(const struct timespec *timestamp, Method method)
{
	bool result = false;
	for (const auto &it : mShdCtxPtrMap) {
		ShdCtx *shdCtx = it.second;
		if (getSample(shdCtx, timestamp, method))
			result = true;
	}
	return result;
}

/**
 */
bool Consumer::getPrevSample(const struct timespec *timestamp)
{
	if (mShdCtxPtrMap.size() != 1) {
		ULOGW("getPrevSample works only for single section");
		return false;
	}

	/* TODO: use a "strictly before" method in libshdata */
	struct timespec ts = {0, 0};
	time_timespec_add_us(timestamp, -1, &ts);
	return getSample(&ts, TLM_FIRST_BEFORE);
}

/**
 */
bool Consumer::getNextSample(const struct timespec *timestamp, uint32_t timeout)
{
	if (mShdCtxPtrMap.size() != 1) {
		ULOGW("getNextSample works only for single section");
		return false;
	}

	/* TODO: use a "strictly after" method in libshdata */
	struct timespec ts = {0, 0};
	time_timespec_add_us(timestamp, +1, &ts);
	if (getSample(&ts, TLM_FIRST_AFTER))
		return true;
	else if (timeout == 0)
		return false;

	/* Get the start time of waiting */
	struct timespec startwait;
	struct timespec waittime;
	uint64_t waited = 0;
	clock_gettime(CLOCK_MONOTONIC, &startwait);

	/* Get producer rate in us */
	ShdCtx *shdCtx = mShdCtxPtrMap.begin()->second;
	assert(shdCtx != NULL);
	uint32_t rate = shdCtx->getRate();

	/* Waiting loop */
	do {
		usleep(rate);
		if (getSample(&ts, TLM_FIRST_AFTER))
			return true;
		time_timespec_diff_now(&startwait, &waittime);
		time_timespec_to_us(&waittime, &waited);
	} while (waited < (uint64_t)timeout * 1000);

	return false;
}

} /* namespace telemetry */
