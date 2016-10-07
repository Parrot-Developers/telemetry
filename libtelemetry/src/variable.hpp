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
 * @file variable.hpp
 *
 */

#ifndef _VARIABLE_HPP_
#define _VARIABLE_HPP_

namespace telemetry {
namespace internal {

/**
 * Variable description record stored in shared memory.
 * Name follows this structure.
 */
struct VarDescRecord {
	uint32_t  reclen;   /**< Size of record (this structure + name + '\0' + padding */
	uint32_t  namelen;  /**< Size of name (not including '\0' */
	uint32_t  type;     /**< Type of variable */
	uint32_t  size;     /**< Size of variable */
	uint32_t  count;    /**< Number of elements for arrays */
	uint32_t  flags;    /**< Additional flags */
};

/**
 * Variable information used internally.
 */
class VarInfo {
private:
	friend class telemetry::Producer;
	friend class telemetry::Consumer;
	friend class telemetry::Logger;

private:
	void             *mPtrOrig;    /**< Pointer to original data */
	void             *mPtrLocal;   /**< Pointer to local internal data */
	struct timespec  *mTimestamp;  /**< Where to store back timestamp */
	const VarDesc    *mVarDesc;    /**< Public variable description */

public:
	inline VarInfo(void *ptr, struct timespec *timestamp) :
		mPtrOrig(ptr),
		mPtrLocal(NULL),
		mTimestamp(timestamp),
		mVarDesc(NULL) {
	}
};

} /* namespace internal */
} /* namespace telemetry */

#endif /* !_VARIABLE_HPP_ */
