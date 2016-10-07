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
 * @file shdctx.hpp
 *
 */

#ifndef _SHD_INCLUDED_HPP
#define _SHD_INCLUDED_HPP

namespace telemetry {
namespace internal {

/**
 */
class ShdCtx {
private:
	std::string                 mSection;
	std::string                 mDir;
	struct shd_ctx              *mShdCtx;
	struct shd_revision         *mShdRev;
	struct shd_hdr_user_info    mHdrInfo;
	bool                        mLastHdrCheckValid;

	void                        *mHdr;
	uint64_t                    mLastOpenFailureTime;

	struct shd_quantity         *mQuantities;
	struct shd_quantity_sample  *mQtySamples;
	uint32_t                    mQtyCount;
	uint32_t                    mQtyMaxCount;

	ShdCtx(const ShdCtx &);
	ShdCtx &operator =(const ShdCtx &);

private:
	int readHdr();
	bool checkHdr();
	void setupSearch(struct shd_sample_search *search,
			const struct timespec *timestamp, Method method);

public:
	ShdCtx(const std::string &section, const std::string &dir, uint32_t qtyMaxCount);
	~ShdCtx();

	inline const std::string &getSection() const {return mSection;}
	inline uint32_t getMaxNbSamples() const {return mHdrInfo.max_nb_samples;}
	inline uint32_t getRate() const {return mHdrInfo.rate;}
	inline size_t getBlobSize() const {return mHdrInfo.blob_size;}
	inline bool isOpened() const {return mShdCtx != NULL;}

	/* Return header without magic */
	inline size_t getHdrSize() const {
		size_t s = mHdrInfo.blob_metadata_hdr_size;
		return mHdr == NULL || s < sizeof(uint32_t) ? 0 :
				s - sizeof(uint32_t);
	}
	inline const void *getHdr() const {
		size_t s = mHdrInfo.blob_metadata_hdr_size;
		return mHdr == NULL || s < sizeof(uint32_t) ? NULL:
				(const uint8_t *)mHdr + sizeof(uint32_t);
	}

	inline uint32_t getQtyCount() const {
		return mQtyCount;
	}

	inline const struct shd_quantity_sample *getQtySample(uint32_t idx) {
		return &mQtySamples[idx];
	}

	inline void addQty(void *ptr, size_t off, size_t size) {
		assert(mQtyCount < mQtyMaxCount);
		mQuantities[mQtyCount].quantity_offset = off;
		mQuantities[mQtyCount].quantity_size = size;
		mQtySamples[mQtyCount].ptr = ptr;
		mQtySamples[mQtyCount].size = size;
		mQtyCount++;
	}

	inline const struct shd_sample_metadata *getQtySampleMetadata(size_t idx) const {
		assert(idx < mQtyCount);
		return &mQtySamples[idx].meta;
	}

	bool openShdCtx();
	void closeShdCtx();
	bool getSample(const struct timespec *timestamp, Method method);

	bool getBlobs(const struct timespec *timestamp, Method method,
			uint32_t before, uint32_t after,
			struct timespec *rtsarray,
			uint32_t *rbefore, uint32_t *rafter,
			void *dst, size_t dstsize);

	bool checkRevision();
};

} /* namespace internal */
} /* namespace telemetry */

#endif /* !_SHD_INCLUDED_HPP */
