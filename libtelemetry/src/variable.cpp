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
 * @file variable.cpp
 *
 */

#include "private.hpp"

namespace telemetry {
using namespace internal;

/**
 */
const char *getVarTypeStr(VarType type)
{
	switch (type) {
	case TLM_TYPE_INVALID: return "INVALID";
	case TLM_TYPE_BOOL:    return "BOOL";
	case TLM_TYPE_UINT8:   return "UINT8";
	case TLM_TYPE_INT8:    return "INT8";
	case TLM_TYPE_UINT16:  return "UINT16";
	case TLM_TYPE_INT16:   return "INT16";
	case TLM_TYPE_UINT32:  return "UINT32";
	case TLM_TYPE_INT32:   return "INT32";
	case TLM_TYPE_UINT64:  return "UINT64";
	case TLM_TYPE_INT64:   return "INT64";
	case TLM_TYPE_FLOAT32: return "FLOAT32";
	case TLM_TYPE_FLOAT64: return "FLOAT64";
	case TLM_TYPE_STRING:  return "STRING";
	case TLM_TYPE_BINARY:  return "BINARY";
	default: return "UNKNOWN";
	}
}

/**
 */
size_t VarDesc::getRecordLength() const
{
	return sizeof(VarDescRecord) + mName.length() + 1;
}

/**
 */
int VarDesc::writeRecord(void *dst, size_t maxdst) const
{
	VarDescRecord rec;
	size_t reclen = getRecordLength();
	assert(dst != NULL);
	assert(reclen <= maxdst);

	memset(&rec, 0, sizeof(rec));
	rec.reclen = reclen;
	rec.namelen = mName.length();
	rec.type = mType;
	rec.size = mSize;
	rec.count = mCount;
	rec.flags = mFlags;
	memcpy(dst, &rec, sizeof(rec));
	memcpy((uint8_t *)dst + sizeof(rec), mName.c_str(), rec.namelen);
	*((uint8_t *)dst + sizeof(rec) + rec.namelen) = '\0';

	return (int)rec.reclen;
}

/**
 */
int VarDesc::readRecord(const void *src, size_t maxsrc)
{
	VarDescRecord rec;

	/* Check validity of buffer */
	if (maxsrc < sizeof(VarDescRecord)) {
		ULOGW("Buffer too small: %zu (%zu)", maxsrc, sizeof(VarDescRecord));
		return -EINVAL;
	}
	memcpy(&rec, src, sizeof(rec));
	if (maxsrc < rec.reclen) {
		ULOGW("Buffer too small: %zu (%u)", maxsrc, rec.reclen);
		return -EINVAL;
	}
	if (maxsrc < sizeof(VarDescRecord) + rec.namelen + 1) {
		ULOGW("Buffer too small: %zu (%zu)", maxsrc,
				sizeof(VarDescRecord) + rec.namelen + 1);
		return -EINVAL;
	}
	if (*((const uint8_t *)src + sizeof(rec) + rec.namelen) != '\0') {
		ULOGW("String not null terminated");
		return -EINVAL;
	}

	/* Read data, additional coherence checks will be done later in consumer */
	mName = (const char *)((const uint8_t *)src + sizeof(rec));
	mType = (VarType)rec.type;
	mSize = rec.size;
	mCount = rec.count;
	mFlags = rec.flags;
	mFullName = mSection + "." + mName;

	return (int)rec.reclen;
}

/**
 */
int VarDesc::readRecordArray(std::vector<VarDesc> &varDescVector,
		const std::string &section, const void *src, size_t maxsrc)
{
	int res = 0;
	const uint8_t *ptr = (const uint8_t *)src;
	size_t off = 0;
	uint32_t varDescCount = 0;

	if (maxsrc < sizeof(uint32_t)) {
		ULOGE("Header too small: %zu (%zu)", maxsrc, sizeof(uint32_t));
		return -EINVAL;
	}

	/* How many variables are in the section ? */
	memcpy(&varDescCount, ptr, sizeof(uint32_t));
	ptr += sizeof(uint32_t);
	off += sizeof(uint32_t);
	varDescVector.reserve(varDescCount);

	/* Read all descriptions */
	for (uint32_t i = 0; i < varDescCount; i++) {
		/* Read description in a local variable */
		VarDesc varDescRead(section);
		res = varDescRead.readRecord(ptr, maxsrc - off);
		if (res < 0)
			break;
		ptr += res;
		off += res;
		varDescVector.push_back(varDescRead);
	}

	return 0;
}

} /* namespace telemetry */
