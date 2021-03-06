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
 * @file registrator.cpp
 *
 */

#include "private.hpp"
ULOG_DECLARE_TAG(libtelemetry);

namespace telemetry {
using namespace internal;

namespace internal {

/**
 */
Registrator::Registrator()
{
	mCompleted = false;
}

/**
 */
Registrator::~Registrator()
{
}

/**
 */
void Registrator::regInternal(void *ptr,
		const std::string &section,
		const std::string &name,
		VarType type,
		size_t size,
		size_t count,
		uint32_t flags,
		struct timespec *timestamp)
{
	/* Not possible to add a new variable if already completed */
	if (mCompleted) {
		ULOGW("Failed to register var '%s': registration completed",
				name.c_str());
		return;
	}

	/* Refuse variables whose type could not be detected */
	if (type == TLM_TYPE_INVALID) {
		ULOGW("Failed to register var '%s': invalid type",
				name.c_str());
		return;
	}

	/* Extract section from name if required (consumer size) */
	std::string realSection(section), realName(name);
	if (section.empty()) {
		realSection = getVarSection(name);
		realName = getVarName(name);
	}

	/* TODO: check that a variable is not registered twice with the same name
	 * We allow the same pointer though (at least in producer, consumer
	 * will refuse this in its regComplete checks) */

	/* Register in tables */
	VarDesc varDesc(realSection, realName, type, size, count, flags);
	VarInfo varInfo(ptr, timestamp);
	mVarDescMap.insert(VarDescMap::value_type(varDesc.getFullName(), varDesc));
	mVarInfoMap.insert(VarInfoMap::value_type(varDesc.getFullName(), varInfo));

	ULOGD("Finished registering var '%s' in section '%s'",
			realName.c_str(), realSection.c_str());
}

/**
 */
std::string Registrator::getVarSection(const std::string varFullName) const
{
	/* Split at first '.' */
	size_t pos = varFullName.find('.');
	if (pos == std::string::npos)
		return "main";
	else
		return varFullName.substr(0, pos);
}

/**
 */
std::string Registrator::getVarName(const std::string varFullName) const
{
	/* Split at first '.' */
	size_t pos = varFullName.find('.');
	if (pos == std::string::npos)
		return varFullName;
	else
		return varFullName.substr(pos + 1);
}

} /* namespace internal */

} /* namespace telemetry */
