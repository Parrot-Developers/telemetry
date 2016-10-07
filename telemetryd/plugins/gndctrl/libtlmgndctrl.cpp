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
 * @file libtlmgndctrl.cpp
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>

#include <map>
#include <vector>

#define POMP_ENABLE_ADVANCED_API
#include "libpomp.hpp"

#define ULOG_TAG tlmgndctrl
#include "ulog.h"

#include "libtlmgndctrl.hpp"
#include "protocol.hpp"

ULOG_DECLARE_TAG(tlmgndctrl);

namespace telemetry {
namespace internal {

/**
 */
class GndCtrlImpl : public GndCtrlItf {
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

private:
	GndCtrlItfCb          *mCb;
	std::string           mName;
	std::string           mCtrlAddr;
	uint32_t              mDataPort;
	uint32_t              mSampleRate;
	uint32_t              mMsgRate;
	struct pomp_ctx       *mCtrlCtx;
	struct pomp_ctx       *mDataCtx;
	SectionInfoMap        mSectionInfoMap;
	bool                  mConnected;

private:
	static void pompCtrlEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
			struct pomp_conn *conn, const struct pomp_msg *msg,
			void *userdata);
	static void pompDataEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
			struct pomp_conn *conn, const struct pomp_msg *msg,
			void *userdata);

	void recvCtrlMsg(const struct pomp_msg *msg);
	void recvDataMsg(const struct pomp_msg *msg);
	void recvConnResp(const struct pomp_msg *msg);
	void recvSample(uint32_t sectionId, const struct timespec *timestamp,
			const void *buf, size_t len);

public:
	GndCtrlImpl(GndCtrlItfCb *cb, struct pomp_loop *loop,
			const std::string &name,
			const std::string &ctrlAddr, uint32_t dataPort,
			uint32_t sampleRate, uint32_t msgRate);
	virtual ~GndCtrlImpl();

	virtual int start();
	virtual int stop();
	virtual const std::string &getSection(uint32_t sectionId) const;
	virtual uint32_t getVarCount(uint32_t sectionId) const;
	virtual const VarDesc &getVarDesc(uint32_t sectionId, uint32_t idx) const;
};

/**
 */
GndCtrlImpl::GndCtrlImpl(GndCtrlItfCb *cb, struct pomp_loop *loop,
		const std::string &name,
		const std::string &ctrlAddr, uint32_t dataPort, uint32_t sampleRate,
		uint32_t msgRate)
{
	mCb = cb;
	mName = name;
	mCtrlAddr = ctrlAddr;
	mDataPort = dataPort;
	mSampleRate = sampleRate;
	mMsgRate = msgRate;
	mCtrlCtx = pomp_ctx_new_with_loop(&GndCtrlImpl::pompCtrlEvtCb, this, loop);
	if (mDataPort > 0)
		mDataCtx = pomp_ctx_new_with_loop(&GndCtrlImpl::pompDataEvtCb, this, loop);
	else
		mDataCtx = NULL;
	mConnected = false;
}

/**
 */
GndCtrlImpl::~GndCtrlImpl()
{
	/* Free resources */
	pomp_ctx_destroy(mCtrlCtx);
	mCtrlCtx = NULL;
	if (mDataCtx != NULL) {
		pomp_ctx_destroy(mDataCtx);
		mDataCtx = NULL;
	}
	mCb = NULL;
}

/**
 */
void GndCtrlImpl::pompCtrlEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
		struct pomp_conn *conn, const struct pomp_msg *msg,
		void *userdata)
{
	GndCtrlImpl *self = reinterpret_cast<GndCtrlImpl *>(userdata);
	switch (evt) {
	case POMP_EVENT_CONNECTED:
		/* Send connection request */
		pomp_conn_send(conn, GNDCTRL_MSG_CONN_REQ, "%u%s%u%u%u",
				GNDCTRL_PROTOCOL_VERSION,
				self->mName.c_str(),
				self->mDataPort,
				self->mSampleRate,
				self->mMsgRate);
		break;

	case POMP_EVENT_DISCONNECTED:
		/* Clear internal state */
		if (self->mConnected)
			self->mCb->disconnected();
		self->mSectionInfoMap.clear();
		self->mConnected = false;
		break;

	case POMP_EVENT_MSG:
		self->recvCtrlMsg(msg);
		break;
	}
}

/**
 */
void GndCtrlImpl::pompDataEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
		struct pomp_conn *conn, const struct pomp_msg *msg,
		void *userdata)
{
	GndCtrlImpl *self = reinterpret_cast<GndCtrlImpl *>(userdata);
	if (evt == POMP_EVENT_MSG)
		self->recvDataMsg(msg);
}

/**
 */
void GndCtrlImpl::recvCtrlMsg(const struct pomp_msg *msg)
{
	uint32_t sectionId = 0;
	char *section = NULL;
	void *buf = NULL;
	uint32_t len = 0;
	uint32_t ts_sec = 0, ts_nsec = 0;
	struct timespec ts;

	switch (pomp_msg_get_id(msg)) {
	case GNDCTRL_MSG_CONN_RESP:
		recvConnResp(msg);
		break;

	case GNDCTRL_MSG_SECTION_ADDED:
		if (pomp_msg_read(msg, "%u%ms", &sectionId, &section) == 0) {
			SectionInfo sectionInfo;
			sectionInfo.mSectionId = sectionId;
			sectionInfo.mSection = section;
			mSectionInfoMap.insert(SectionInfoMap::value_type(sectionId, sectionInfo));
			mCb->sectionAdded(sectionId);
		}
		break;

	case GNDCTRL_MSG_SECTION_REMOVED:
		if (pomp_msg_read(msg, "%u", &sectionId) == 0) {
			SectionInfoMap::iterator it = mSectionInfoMap.find(sectionId);
			if (it != mSectionInfoMap.end()) {
				mCb->sectionRemoved(sectionId);
				mSectionInfoMap.erase(it);
			}
		}
		break;

	case GNDCTRL_MSG_SECTION_CHANGED:
		if (pomp_msg_read(msg, "%u%p%u", &sectionId, &buf, &len) == 0) {
			SectionInfoMap::iterator it = mSectionInfoMap.find(sectionId);
			if (it != mSectionInfoMap.end()) {
				SectionInfo &sectionInfo = it->second;
				sectionInfo.mVarDescVector.clear();
				telemetry::VarDesc::readRecordArray(sectionInfo.mVarDescVector,
						sectionInfo.mSection, buf, len);
				mCb->sectionChanged(sectionId, buf, len);
			}
		}
		break;

	/* Only if client is configured to receive samples on the control channel */
	case GNDCTRL_MSG_SECTION_SAMPLE:
		if (pomp_msg_read(msg, "%u%u%u%p%u", &sectionId,
				&ts_sec, &ts_nsec, &buf, &len) == 0) {
			ts.tv_sec = ts_sec;
			ts.tv_nsec = ts_nsec;
			recvSample(sectionId, &ts, buf, len);
		}
	}

	free(section);
}

/**
 */
void GndCtrlImpl::recvDataMsg(const struct pomp_msg *msg)
{
	uint32_t sectionId = 0;
	uint32_t ts_sec = 0, ts_nsec = 0;
	void *buf = NULL;
	uint32_t len = 0;
	struct timespec ts;

	switch (pomp_msg_get_id(msg)) {
	case GNDCTRL_MSG_SECTION_SAMPLE:
		if (pomp_msg_read(msg, "%u%u%u%p%u", &sectionId,
				&ts_sec, &ts_nsec, &buf, &len) == 0) {
			ts.tv_sec = ts_sec;
			ts.tv_nsec = ts_nsec;
			recvSample(sectionId, &ts, buf, len);
		}
	break;
	}
}

/**
 */
void GndCtrlImpl::recvConnResp(const struct pomp_msg *msg)
{
	int res = 0;
	uint32_t status = 0, count = 0;
	struct pomp_decoder *dec = NULL;
	GndCtrlItfCb::PropertyMap properties;

	/* Create decoder and initialize it */
	dec = pomp_decoder_new();
	if (dec == NULL)
		goto out;
	res = pomp_decoder_init(dec, msg);
	if (res < 0)
		goto out;

	/* Read status and number of properties */
	res = pomp_decoder_read_u32(dec, &status);
	if (res < 0)
		goto out;
	res = pomp_decoder_read_u32(dec, &count);
	if (res < 0)
		goto out;

	/* Read properties */
	for (uint32_t i = 0; i < count; i++) {
		const char *key = NULL;
		const char *val = NULL;
		res = pomp_decoder_read_cstr(dec, &key);
		if (res < 0)
			goto out;
		res = pomp_decoder_read_cstr(dec, &val);
		if (res < 0)
			goto out;
		properties[key] = val;
	}

	/* Notify connection */
	if (status == 1) {
		mConnected = true;
		mCb->connected(properties);
	}

	/* Cleanup */
out:
	if (dec != NULL)
		pomp_decoder_destroy(dec);
}

/**
 */
void GndCtrlImpl::recvSample(uint32_t sectionId, const struct timespec *timestamp,
		const void *buf, size_t len)
{
	const uint8_t *varbuf = (const uint8_t *)buf;
	size_t varoff = 0;

	SectionInfoMap::iterator it = mSectionInfoMap.find(sectionId);
	if (it == mSectionInfoMap.end())
		return;
	SectionInfo &sectionInfo = it->second;
	VarDescVector &varDescVector = sectionInfo.mVarDescVector;

	mCb->sampleBegin(sectionId, timestamp, buf, len);
	for (uint32_t varId = 0; varId < varDescVector.size(); varId++) {
		const telemetry::VarDesc &varDesc = varDescVector[varId];
		if (varoff + varDesc.getTotalSize() > len) {
			ULOGW("Buffer too small");
			break;
		}
		mCb->sample(sectionId, timestamp, varId,
				varDesc, varbuf,
				varDesc.getTotalSize());
		varbuf += varDesc.getTotalSize();
		varoff += varDesc.getTotalSize();
	}

	mCb->sampleEnd(sectionId);
}

/**
 */
int GndCtrlImpl::start()
{
	int res = 0;
	struct sockaddr_storage addr;
	uint32_t addrlen = 0;
	char dataAddr[256] = "";

	/* Get address for control channel*/
	addrlen = sizeof(addr);
	res = pomp_addr_parse(mCtrlAddr.c_str(), (struct sockaddr *)&addr, &addrlen);
	if (res < 0) {
		ULOGE("pomp_addr_parse(%s): err=%d(%s)",
				mCtrlAddr.c_str(), res, strerror(-res));
		goto error;
	}

	/* Start context for control channel */
	res = pomp_ctx_connect(mCtrlCtx, (struct sockaddr *)&addr, addrlen);
	if (res < 0) {
		ULOGE("pomp_ctx_connect: err=%d(%s)", res, strerror(-res));
		goto error;
	}

	if (mDataPort > 0) {
		/* Get address for data channel*/
		snprintf(dataAddr, sizeof(dataAddr), "inet:0.0.0.0:%u", mDataPort);
		addrlen = sizeof(addr);
		res = pomp_addr_parse(dataAddr, (struct sockaddr *)&addr, &addrlen);
		if (res < 0) {
			ULOGE("pomp_addr_parse(%s): err=%d(%s)",
					dataAddr, res, strerror(-res));
			goto error;
		}

		/* Start context for data channel */
		res = pomp_ctx_bind(mDataCtx, (struct sockaddr *)&addr, addrlen);
		if (res < 0) {
			ULOGE("pomp_ctx_bind: err=%d(%s)", res, strerror(-res));
			goto error;
		}
	}

	/* Success */
	return 0;

	/* Cleanup in case of error */
error:
	pomp_ctx_stop(mCtrlCtx);
	if (mDataCtx != NULL)
		pomp_ctx_stop(mDataCtx);
	return res;
}

/**
 */
int GndCtrlImpl::stop()
{
	int res = 0;

	/* Stop contexts */
	res = pomp_ctx_stop(mCtrlCtx);
	if (res < 0)
		ULOGE("pomp_ctx_stop: err=%d(%s)", res, strerror(-res));
	if (mDataCtx != NULL) {
		res = pomp_ctx_stop(mDataCtx);
		if (res < 0)
			ULOGE("pomp_ctx_stop: err=%d(%s)", res, strerror(-res));
	}

	return 0;
}

/**
 */
const std::string &GndCtrlImpl::getSection(uint32_t sectionId) const
{
	SectionInfoMap::const_iterator it = mSectionInfoMap.find(sectionId);
	assert(it != mSectionInfoMap.end());
	return it->second.mSection;
}

/**
 */
uint32_t GndCtrlImpl::getVarCount(uint32_t sectionId) const
{
	SectionInfoMap::const_iterator it = mSectionInfoMap.find(sectionId);
	assert(it != mSectionInfoMap.end());
	return it->second.mVarDescVector.size();
}

/**
 */
const VarDesc &GndCtrlImpl::getVarDesc(uint32_t sectionId, uint32_t varId) const
{
	SectionInfoMap::const_iterator it = mSectionInfoMap.find(sectionId);
	assert(it != mSectionInfoMap.end());
	return it->second.mVarDescVector[varId];
}

} /* namespace internal */

/**
 */
GndCtrlItf *GndCtrlItf::create(GndCtrlItfCb *cb, struct pomp_loop *loop,
		const std::string &name,
		const std::string &ctrlAddr, uint32_t dataPort,
		uint32_t sampleRate, uint32_t msgRate)
{
	return new internal::GndCtrlImpl(cb, loop, name, ctrlAddr, dataPort,
									sampleRate, msgRate);
}

} /* namespace telemetry */
