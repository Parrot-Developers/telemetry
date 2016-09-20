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
 * @file shdctx.cpp
 *
 */

#include "private.hpp"

/** Delay (in us) between successive attempt to open shared memory */
#define OPEN_ATTEMPT_DELAY  (1000 * 1000)

namespace telemetry {
namespace internal {

/**
 */
ShdCtx::ShdCtx(const std::string &section, const std::string &dir, uint32_t qtyMaxCount)
		: mSection(section), mDir(dir)
{
	/* Initialize parameters */
	mShdCtx = NULL;
	mShdRev = NULL;
	mLastOpenFailureTime = 0;
	memset(&mHdrInfo, 0, sizeof(mHdrInfo));
	mHdr = NULL;
	mLastHdrCheckValid = true;

	/* Allocate internal structure for queries */
	mQuantities = (struct shd_quantity *)calloc(qtyMaxCount,
			sizeof(struct shd_quantity));
	mQtySamples = (struct shd_quantity_sample *)calloc(qtyMaxCount,
			sizeof(struct shd_quantity_sample));
	assert(mQuantities != NULL);
	assert(mQtySamples != NULL);
	mQtyCount = 0;
	mQtyMaxCount = qtyMaxCount;
}

/**
 */
ShdCtx::~ShdCtx()
{
	/* Must be closed */
	assert(mShdCtx == NULL);
	free(mQuantities);
	free(mQtySamples);
	mQuantities = NULL;
	mQtySamples = NULL;
}

/**
 */
bool ShdCtx::openShdCtx()
{
	/* Must not be opened */
	assert(mShdCtx == NULL);

	/* Avoid too many failures in case the process with the producer is
	 * not yet started */
	struct timespec ts;
	uint64_t currentTime = 0;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	time_timespec_to_us(&ts, &currentTime);
	if (mLastOpenFailureTime != 0 && currentTime < mLastOpenFailureTime + OPEN_ATTEMPT_DELAY) {
		/* Try again later... */
		return false;
	}

	/* If a previous open was successful but header was invalid, to not
	 * try to open it again */
	/* TODO: should we try to open it again ? */
	if (!mLastHdrCheckValid)
		return false;

	/* Try to open section */
	mShdCtx = shd_open(mSection.c_str(), mDir.empty() ? NULL : mDir.c_str(), &mShdRev);
	if (mShdCtx == NULL) {
		mLastOpenFailureTime = currentTime;
		return false;
	}

	/* Clear failure time so new open attempt will be done without delay */
	mLastOpenFailureTime = 0;

	/* Now try to read header */
	if (readHdr() < 0) {
		/* Means that format has changed during the read header...
		 * ... close it now, we will open it at next read */
		closeShdCtx();
		return false;
	}

	/* Check that header is valid */
	if (!checkHdr()) {
		/* Avoid opening too soon for nothing */
		mLastOpenFailureTime = currentTime;
		mLastHdrCheckValid = false;
		closeShdCtx();
		return false;
	}

	/* OK ! */
	return true;
}

/**
 * Close shared memory context. This will clear everything related to it
 * (header and quantities)
 */
void ShdCtx::closeShdCtx()
{
	if (mShdCtx != NULL) {
		shd_close(mShdCtx, mShdRev);
		mShdCtx = NULL;
		mShdRev = NULL;

		memset(&mHdrInfo, 0, sizeof(mHdrInfo));
		free(mHdr);
		mHdr = NULL;

		memset(mQuantities, 0, mQtyMaxCount * sizeof(struct shd_quantity));
		memset(mQtySamples, 0, mQtyMaxCount * sizeof(struct shd_quantity_sample));
		mQtyCount = 0;
	}
}

/**
 */
void ShdCtx::setupSearch(struct shd_sample_search *search,
		const struct timespec *timestamp, Method method)
{
	memset(search, 0, sizeof(*search));
	if (timestamp != NULL)
		search->date = *timestamp;
	switch (method) {
	case TLM_LATEST: search->method = SHD_LATEST; break;
	case TLM_CLOSEST: search->method = SHD_CLOSEST; break;
	case TLM_FIRST_AFTER: search->method = SHD_FIRST_AFTER; break;
	case TLM_FIRST_BEFORE: search->method = SHD_FIRST_BEFORE; break;
	default: assert(0); break;
	}
}

/**
 */
bool ShdCtx::getSample(const struct timespec *timestamp, Method method)
{
	int res = 0;
	assert(mShdCtx != NULL);

	/* Setup search */
	struct shd_sample_search search;
	setupSearch(&search, timestamp, method);

	/* Read single sample for given quantities */
	res = shd_read_from_sample(mShdCtx, mQtyCount, &search, mQuantities, mQtySamples);
	if (res < 0) {
		/* No log for expected errors */
		if (res == -EAGAIN || res == -ENOENT)
			return false;
		ULOGE("shd_read_from_sample: err=%d(%s)", res, strerror(-res));
		return false;
	}

	/* End the read operation, this will check validity of read sample */
	res = shd_end_read(mShdCtx, mShdRev);
	if (res < 0) {
		if (res == -ENODEV) {
			/* Format has changed, need to reopen the shd */
			closeShdCtx();
		} else {
			ULOGE("shd_end_read: err=%d(%s)", res, strerror(-res));
		}
		return false;
	}

	return true;
}

/**
 */
bool ShdCtx::getBlobs(const struct timespec *timestamp, Method method,
		uint32_t before, uint32_t after,
		struct timespec *rtsarray,
		uint32_t *rbefore, uint32_t *rafter,
		void *dst, size_t dstsize)
{
	int res = 0;
	assert(mShdCtx != NULL);

	/* Initialize result */
	assert(rtsarray != NULL && rbefore != NULL && rafter != NULL);
	memset(rtsarray, 0, (before + after + 1) * sizeof(struct timespec));
	*rbefore = 0;
	*rafter = 0;

	/* Setup search */
	struct shd_sample_search search;
	setupSearch(&search, timestamp, method);
	search.nb_values_before_date = before;
	search.nb_values_after_date = after;

	/* Select samples */
	struct shd_sample_metadata *metadata = NULL;
	struct shd_search_result sres;
	res = shd_select_samples(mShdCtx, &search, &metadata, &sres);
	if (res < 0) {
		/* No log for expected errors */
		if (res == -EAGAIN || res == -ENOENT)
			return false;
		ULOGE("shd_select_samples: err=%d(%s)", res, strerror(-res));
		return false;
	}

	/* Fill timestamp found */
	assert(sres.nb_matches > 0);
	assert(sres.nb_matches <= (int)(before + after +1));
	assert(sres.r_sample_idx < sres.nb_matches);
	*rbefore = sres.r_sample_idx;
	*rafter = sres.nb_matches - sres.r_sample_idx - 1;
	for (int i = 0; i < sres.nb_matches; i++)
		rtsarray[i] = metadata[i].ts;

	/* Get data (full blob) */
	res = shd_read_quantity(mShdCtx, NULL, dst, dstsize);
	if (res < 0)
		ULOGE("shd_read_quantity: err=%d(%s)", res, strerror(-res));

	/* End the read operation, this will check validity of read sample */
	res = shd_end_read(mShdCtx, mShdRev);
	if (res < 0) {
		if (res == -ENODEV) {
			/* Format has changed, need to reopen the shd */
			closeShdCtx();
		} else {
			ULOGE("shd_end_read: err=%d(%s)", res, strerror(-res));
		}
		return false;
	}

	return true;
}

/**
 */
bool ShdCtx::checkRevision()
{
	assert(mShdCtx != NULL);

	/* Re-reading the header will trigger an internal check */
	struct shd_hdr_user_info hdrInfo;
	memset(&hdrInfo, 0, sizeof(hdrInfo));
	if (shd_read_section_hdr(mShdCtx, &hdrInfo, mShdRev) < 0) {
		ULOGI("Revision of section '%s' has changed, closing ...",
				mSection.c_str());
		closeShdCtx();
		return false;
	}
	return true;
}

/**
 */
int ShdCtx::readHdr()
{
	int res = 0;
	size_t hdrSize = 0;
	assert(mShdCtx != NULL);

	/* Read section header */
	res = shd_read_section_hdr(mShdCtx, &mHdrInfo, mShdRev);
	if (res < 0) {
		ULOGE("shd_read_section_hdr: err=%d(%s)", res, strerror(-res));
		goto error;
	}

	/* Read section header, allocate memory to read metadata */
	hdrSize = mHdrInfo.blob_metadata_hdr_size;
	mHdr = calloc(1, hdrSize);
	if (mHdr == NULL) {
		res = -ENOMEM;
		goto error;
	}

	/* Read metadata header */
	res = shd_read_blob_metadata_hdr(mShdCtx, mHdr, hdrSize, mShdRev);
	if (res < 0) {
		ULOGE("shd_read_blob_metadata_hdr: err=%d(%s)", res, strerror(-res));
		goto error;
	}

	return 0;

	/* Cleanup in case of error */
error:
	if (mHdr != NULL) {
		free(mHdr);
		mHdr = NULL;
	}
	return res;
}

/**
 */
bool ShdCtx::checkHdr()
{
	assert(mShdCtx != NULL);

	if (mHdrInfo.blob_metadata_hdr_size < sizeof(uint32_t)) {
		ULOGW("'%s': Bad header size: %zu",
				mSection.c_str(),
				mHdrInfo.blob_metadata_hdr_size);
		return false;
	}

	/* Check Magic */
	if (*((uint32_t *)mHdr) != TLM_SHM_HDR_MAGIC) {
		ULOGW("'%s': Bad magic: 0x%08x",
				mSection.c_str(),
				*((uint32_t *)mHdr));
		return false;
	}

	return true;
}

} /* namespace internal */
} /* namespace telemetry */
