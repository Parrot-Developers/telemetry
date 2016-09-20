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
 * @file gndctrl.c
 *
 */

#include <memory>

#include "headers.hpp"
#include "futils/timetools.h"

#ifdef BUILD_LIBPUTILS
#  include "putils/properties.h"
#endif /* BUILD_LIBPUTILS */

ULOG_DECLARE_TAG(tlmgndctrl);

#define CTRL_ADDR  "inet:0.0.0.0:9060"
#define DATA_ADDR  "inet:0.0.0.0:9061"

/**
 * Ground control client
 */
class GndCtrlClient {
private:
	friend class GndCtrlLoggerCb;

private:
	struct pomp_conn    *mConn;         /**< Pomp connection */
	std::string         mName;          /**< Client name */
	uint32_t            mSampleRate;    /**< Sampling rate (in microseconds) */
	uint32_t            mMsgRate;       /**< Message rate (in microseconds) */
	struct sockaddr_in  mDataAddr;      /**< UDP address for data */
	bool                mIsValid;       /**< Connection is valid */
	bool                mUseTcp;        /**< Use TCP protocol to send data */

private:
	int recvConnReq(const struct pomp_msg *msg);
	int sendConnResp(bool ok);

public:
	GndCtrlClient(struct pomp_conn *conn);
	~GndCtrlClient();

	int recvCtrlMsg(const struct pomp_msg *msg);
	int sendCtrlMsg(const struct pomp_msg *msg);
	int sendDataMsg(struct pomp_ctx *ctx, const struct pomp_msg *msg);
};

/**
 * Ground control logger callback interface.
 */
class GndCtrlLoggerCb : public telemetry::LoggerCb {
private:
	typedef std::map<struct pomp_conn *, GndCtrlClient *> ClientMap;
	struct MsgData {
		struct timespec mTimestamp;
		pomp_msg *mMsg;

		inline MsgData(uint32_t sectionId, const struct timespec *timestamp, const void *buf, size_t len) {
			mMsg = pomp_msg_new();

			/* Setup message */
			pomp_msg_write(mMsg, GNDCTRL_MSG_SECTION_SAMPLE, "%u%u%u%p%u",
					sectionId,
					(uint32_t)timestamp->tv_sec,
					(uint32_t)timestamp->tv_nsec,
					buf, (uint32_t)len);

			mTimestamp = *timestamp;
		}

		inline ~MsgData() {
			pomp_msg_destroy(mMsg);
		}
	};
	typedef std::vector<std::shared_ptr<MsgData> > MsgVector;
	struct SectionData {
		struct timespec mTimestamp;
		MsgVector mMsgVector;

		inline SectionData(const struct timespec &timestamp) {
			mTimestamp = timestamp;
		}
	};
	typedef std::map<uint32_t, SectionData> SectionMap;
	typedef std::map<GndCtrlClient *, SectionMap> ClientSectionMap;

	telemetry::Logger  *mLogger;            /**< Associated logger */
	struct pomp_loop   *mLoop;              /**< Associated loop */
	std::string        mInstance;           /**< Logger instance */
	struct pomp_ctx    *mCtrlCtx;           /**< Pomp context for control */
	struct pomp_ctx    *mDataCtx;           /**< Pomp context for data */
	ClientMap          mClientMap;          /**< Connected client */
	ClientSectionMap   mClientSectionMap;   /**< Client related section map */

	/** Client being connected, used for replaying logger events */
	GndCtrlClient      *mClientConnecting;

private:
	static void pompCtrlEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
			struct pomp_conn *conn, const struct pomp_msg *msg,
			void *userdata);
	static void pompDataEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
			struct pomp_conn *conn, const struct pomp_msg *msg,
			void *userdata);

public:
	GndCtrlLoggerCb(telemetry::Logger *logger, struct pomp_loop *loop,
			const std::string &instance);
	virtual ~GndCtrlLoggerCb();

	int start();
	int stop();

public:
	virtual void sectionAdded(uint32_t sectionId);
	virtual void sectionRemoved(uint32_t sectionId);
	virtual void sectionChanged(uint32_t sectionId,
			const void *buf, size_t len);

	virtual void sampleBegin(uint32_t sectionId,
			const struct timespec *timestamp,
			const void *buf, size_t len);

	virtual void sampleEnd(uint32_t sectionId);

	virtual void sample(uint32_t sectionId,
			const struct timespec *timestamp,
			uint32_t varId,
			const telemetry::VarDesc &varDesc,
			const void *buf, size_t len);
};

/**
 */
GndCtrlClient::GndCtrlClient(struct pomp_conn *conn)
{
	mConn = conn;
	memset(&mDataAddr, 0, sizeof(mDataAddr));
	mIsValid = false;
	mSampleRate = 0;
	mMsgRate = 0;
	mUseTcp = false;
}

/**
 */
GndCtrlClient::~GndCtrlClient()
{
	if (mIsValid)
		ULOGI("DiscReq: name='%s'", mName.c_str());
	mConn = NULL;
}

/**
 */
int GndCtrlClient::recvCtrlMsg(const struct pomp_msg *msg)
{
	uint32_t id = pomp_msg_get_id(msg);

	switch (id) {
	case GNDCTRL_MSG_CONN_REQ:
		return recvConnReq(msg);

	case GNDCTRL_MSG_SUBSCRIBE_REQ:
		/* TODO */
		break;

	case GNDCTRL_MSG_UNSUBSCRIBE_REQ:
		/* TODO */
		break;
	}

	/* Not a fatal error to receive unknown messages */
	return 0;
}

/**
 */
int GndCtrlClient::sendCtrlMsg(const struct pomp_msg *msg)
{
	return pomp_conn_send_msg(mConn, msg);
}

/**
 */
int GndCtrlClient::sendDataMsg(struct pomp_ctx *ctx, const struct pomp_msg *msg)
{
	if (!mIsValid)
		return -ENOTCONN;
	if (mUseTcp)
		return sendCtrlMsg(msg);
	return pomp_ctx_send_msg_to(ctx, msg, (const sockaddr *)&mDataAddr,
			sizeof(mDataAddr));
}

/**
 */
int GndCtrlClient::recvConnReq(const struct pomp_msg *msg)
{
	int res = 0;
	uint32_t version = 0;
	char *name = NULL;
	uint32_t dataport = 0;
	uint32_t sampleRate = 0;
	uint32_t msgRate = 0;
	bool ok = false;

	/* Decode message */
	res = pomp_msg_read(msg, "%u%ms%u%u%u", &version, &name, &dataport,
						&sampleRate, &msgRate);
	if (res < 0) {
		ULOGE("ConnReq: err=%d(%s)", res, strerror(-res));
		goto out;
	}

	/* Check protocol version */
	if (version != GNDCTRL_PROTOCOL_VERSION) {
		ULOGE("ConnReq: bad version(%u)", version);
		goto out;
	}

	/* Save name */
	mName = name;

	/* Save sample rate */
	mSampleRate = sampleRate;

	/* Save message rate */
	mMsgRate = msgRate;

	if (dataport > 0) {
		const struct sockaddr *addr = NULL;
		uint32_t addrlen = 0;
		char buf[256] = "";

		mUseTcp = false;

		/* Construct address for data channel */
		addr = pomp_conn_get_peer_addr(mConn, &addrlen);
		if (addr == NULL) {
			ULOGE("ConnReq: bad version(%u)", version);
			goto out;
		}
		if (addr->sa_family != AF_INET || addrlen != sizeof(mDataAddr)) {
			ULOGE("ConnReq: bad address (family=%u len=%u)",
					addr->sa_family, addrlen);
			goto out;
		}
		memcpy(&mDataAddr, addr, sizeof(mDataAddr));
		mDataAddr.sin_port = htons(dataport);

		pomp_addr_format(buf, sizeof(buf), addr, addrlen);
		ULOGI("ConnReq: name='%s' addr='%s' dataport=%u", name, buf, dataport);
	} else {
		mUseTcp = true;
	}

	ok = true;

	/* Free resources at exit */
out:
	free(name);

	/* Send response, final status */
	mIsValid = ok;
	res = sendConnResp(ok);
	return ok && res == 0 ? 0 : -EINVAL;
}

/**
 */
int GndCtrlClient::sendConnResp(bool ok)
{
	int res = 0;
	struct pomp_msg *msg = NULL;
	struct pomp_encoder *enc = NULL;

	/* Setup a map of key/value */
	typedef std::map<std::string, std::string> PropertyMap;
	PropertyMap properties;

	/* Extract system properties */
#ifdef BUILD_LIBPUTILS
	static const char * const sysprops[] = {
		"ro.hardware",
		"ro.build.date",
		"ro.parrot.build.group",
		"ro.parrot.build.product",
		"ro.parrot.build.project",
		"ro.parrot.build.region",
		"ro.parrot.build.uid",
		"ro.parrot.build.variant",
		"ro.parrot.build.version",
		"ro.revision",
		"ro.serialno",
	};
	for (size_t i = 0; i < sizeof(sysprops) / sizeof(sysprops[0]); i++) {
		char value[SYS_PROP_VALUE_MAX] = "";
		sys_prop_get(sysprops[i], value, "");
		properties[sysprops[i]] = value;
	}
#endif /* BUILD_LIBPUTILS */

	/* Create message/encoder */
	msg = pomp_msg_new();
	enc = pomp_encoder_new();
	if (msg == NULL || enc == NULL) {
		res = -ENOMEM;
		goto out;
	}

	/* Initialize message with id */
	res = pomp_msg_init(msg, GNDCTRL_MSG_CONN_RESP);
	if (res < 0)
		goto out;
	res = pomp_encoder_init(enc, msg);
	if (res < 0)
		goto out;

	/* Response status */
	res = pomp_encoder_write_u32(enc, ok ? 1 : 0);
	if (res < 0)
		goto out;

	/* System properties */
	res = pomp_encoder_write_u32(enc, ok ? properties.size() : 0);
	if (res < 0)
		goto out;
	for (PropertyMap::const_iterator it = properties.begin();
			ok && it != properties.end();
			++it) {
		const std::string &key = it->first;
		const std::string &val = it->second;
		res = pomp_encoder_write_str(enc, key.c_str());
		if (res < 0)
			goto out;
		res = pomp_encoder_write_str(enc, val.c_str());
		if (res < 0)
			goto out;
	}

	/* Finish message and send it */
	res = pomp_msg_finish(msg);
	if (res < 0)
		goto out;
	res = sendCtrlMsg(msg);
	if (res < 0)
		goto out;

	/* Cleanup */
out:
	if (enc != NULL)
		pomp_encoder_destroy(enc);
	if (msg != NULL)
		pomp_msg_destroy(msg);
	return res;
}

/**
 */
GndCtrlLoggerCb::GndCtrlLoggerCb(telemetry::Logger *logger, struct pomp_loop *loop,
		const std::string &instance)
{
	mLogger = logger;
	mLoop = loop;
	mInstance = instance;
	mCtrlCtx = pomp_ctx_new_with_loop(&GndCtrlLoggerCb::pompCtrlEvtCb, this, loop);
	mDataCtx = pomp_ctx_new_with_loop(&GndCtrlLoggerCb::pompDataEvtCb, this, loop);
	mClientConnecting = NULL;
}

/**
 */
GndCtrlLoggerCb::~GndCtrlLoggerCb()
{
	/* Should not be running */
	if (!mClientMap.empty())
		ULOGW("Client map not empty");

	/* Free resources */
	pomp_ctx_destroy(mCtrlCtx);
	pomp_ctx_destroy(mDataCtx);
	mLogger = NULL;
	mLoop = NULL;
	mCtrlCtx = NULL;
	mDataCtx = NULL;
}

/**
 */
void GndCtrlLoggerCb::pompCtrlEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
		struct pomp_conn *conn, const struct pomp_msg *msg,
		void *userdata)
{
	GndCtrlLoggerCb *self = reinterpret_cast<GndCtrlLoggerCb *>(userdata);
	GndCtrlClient *client = NULL;
	ClientMap::iterator it;

	switch (evt) {
	case POMP_EVENT_CONNECTED:
		client = new GndCtrlClient(conn);
		self->mClientMap.insert(ClientMap::value_type(conn, client));
		break;

	case POMP_EVENT_DISCONNECTED:
		it = self->mClientMap.find(conn);
		if (it == self->mClientMap.end()) {
			ULOGW("Connection %p not found in client map", conn);
		} else {
			delete it->second;
			self->mClientMap.erase(it);
		}
		break;

	case POMP_EVENT_MSG:
		it = self->mClientMap.find(conn);
		if (it == self->mClientMap.end()) {
			ULOGW("Connection %p not found in client map", conn);
		} else if (it->second->recvCtrlMsg(msg) < 0) {
			/* This will send a disconnected event that
			 *  will remove the connection from map */
			pomp_conn_disconnect(conn);
		} else if (pomp_msg_get_id(msg) == GNDCTRL_MSG_CONN_REQ) {
			/* At connection, replay section events for new client */
			self->mClientConnecting = it->second;
			self->mLogger->replaySectionEvents(self);
			self->mClientConnecting = NULL;
		}
		break;
	}
}

/**
 */
void GndCtrlLoggerCb::pompDataEvtCb(struct pomp_ctx *ctx, enum pomp_event evt,
		struct pomp_conn *conn, const struct pomp_msg *msg,
		void *userdata)
{
	/* Ignore everything received on data channel */
}

/**
 */
int GndCtrlLoggerCb::start()
{
	int res = 0;
	struct sockaddr_storage addr;
	uint32_t addrlen = 0;

	/* Get address for control channel*/
	addrlen = sizeof(addr);
	res = pomp_addr_parse(CTRL_ADDR, (struct sockaddr *)&addr, &addrlen);
	if (res < 0) {
		ULOGE("pomp_addr_parse(%s): err=%d(%s)",
				CTRL_ADDR, res, strerror(-res));
		goto error;
	}

	/* Start context for control channel */
	res = pomp_ctx_listen(mCtrlCtx, (struct sockaddr *)&addr, addrlen);
	if (res < 0) {
		ULOGE("pomp_ctx_listen: err=%d(%s)", res, strerror(-res));
		goto error;
	}

	/* Get address for data channel*/
	addrlen = sizeof(addr);
	res = pomp_addr_parse(DATA_ADDR, (struct sockaddr *)&addr, &addrlen);
	if (res < 0) {
		ULOGE("pomp_addr_parse(%s): err=%d(%s)",
				DATA_ADDR, res, strerror(-res));
		goto error;
	}

	/* Start context for data channel */
	res = pomp_ctx_bind(mDataCtx, (struct sockaddr *)&addr, addrlen);
	if (res < 0) {
		ULOGE("pomp_ctx_bind: err=%d(%s)", res, strerror(-res));
		goto error;
	}

	/* Success */
	return 0;

	/* Cleanup in case of error */
error:
	pomp_ctx_stop(mCtrlCtx);
	pomp_ctx_stop(mDataCtx);
	return res;
}

/**
 */
int GndCtrlLoggerCb::stop()
{
	int res = 0;

	/* Stop contexts, this should disconnect all clients */
	res = pomp_ctx_stop(mCtrlCtx);
	if (res < 0)
		ULOGE("pomp_ctx_stop: err=%d(%s)", res, strerror(-res));
	res = pomp_ctx_stop(mDataCtx);
	if (res < 0)
		ULOGE("pomp_ctx_stop: err=%d(%s)", res, strerror(-res));

	return 0;
}

/**
 */
void GndCtrlLoggerCb::sectionAdded(uint32_t sectionId)
{
	/* Setup message */
	pomp_msg *msg = pomp_msg_new();
	pomp_msg_write(msg, GNDCTRL_MSG_SECTION_ADDED, "%u%s",
			sectionId, mLogger->getSection(sectionId).c_str());

	/* Either send to connection client if it is a replay or all clients */
	if (mClientConnecting != NULL) {
		mClientConnecting->sendCtrlMsg(msg);
	} else {
		for (ClientMap::const_iterator it = mClientMap.begin();
				it != mClientMap.end();
				++it) {
			GndCtrlClient *client = it->second;
			client->sendCtrlMsg(msg);
		}
	}

	pomp_msg_destroy(msg);

	for (ClientMap::const_iterator it = mClientMap.begin();
			it != mClientMap.end();
			++it) {
		SectionMap &sectionMap = mClientSectionMap[it->second];

		/* Add data initialized with last timestamp for this section */
		struct timespec ts = {0, 0};
		sectionMap.insert(SectionMap::value_type(sectionId, {ts}));
	}
}

/**
 */
void GndCtrlLoggerCb::sectionRemoved(uint32_t sectionId)
{
	/* Setup message */
	pomp_msg *msg = pomp_msg_new();
	pomp_msg_write(msg, GNDCTRL_MSG_SECTION_REMOVED, "%u", sectionId);

	/* Either send to connection client if it is a replay or all clients */
	if (mClientConnecting != NULL) {
		mClientConnecting->sendCtrlMsg(msg);
	} else {
		for (ClientMap::const_iterator it = mClientMap.begin();
				it != mClientMap.end();
				++it) {
			GndCtrlClient *client = it->second;
			client->sendCtrlMsg(msg);
		}
	}

	pomp_msg_destroy(msg);

	for (ClientMap::const_iterator it = mClientMap.begin();
			it != mClientMap.end();
			++it) {
		SectionMap &sectionMap = mClientSectionMap[it->second];

		/* Remove data for this section */
		sectionMap.erase(sectionId);
	}
}

/**
 */
void GndCtrlLoggerCb::sectionChanged(uint32_t sectionId,
		const void *buf, size_t len)
{
	/* Setup message */
	pomp_msg *msg = pomp_msg_new();
	pomp_msg_write(msg, GNDCTRL_MSG_SECTION_CHANGED, "%u%p%u",
			sectionId, buf, (uint32_t)len);

	/* Either send to connection client if it is a replay or all clients */
	if (mClientConnecting != NULL) {
		mClientConnecting->sendCtrlMsg(msg);
	} else {
		for (ClientMap::const_iterator it = mClientMap.begin();
				it != mClientMap.end();
				++it) {
			GndCtrlClient *client = it->second;
			client->sendCtrlMsg(msg);
		}
	}

	pomp_msg_destroy(msg);
}

/**
 */
void GndCtrlLoggerCb::sampleBegin(uint32_t sectionId,
		const struct timespec *timestamp,
		const void *buf, size_t len)
{
	std::shared_ptr<MsgData> msgData {std::make_shared<MsgData>(sectionId, timestamp, buf, len)};

	/* Send message to all clients on the data channel */
	for (ClientMap::const_iterator it = mClientMap.begin();
			it != mClientMap.end();
			++it) {
		GndCtrlClient *client = it->second;

		SectionMap &sectionMap = mClientSectionMap[client];

		SectionMap::iterator it2 = sectionMap.find(sectionId);
		if (it2 == sectionMap.end())
			continue;

		SectionData &sectionData = it2->second;

		/* Store message data in client vector */
		sectionData.mMsgVector.push_back(msgData);

		if (time_timespec_diff_in_range(timestamp, &sectionData.mTimestamp, client->mMsgRate, NULL))
			continue;

		struct timespec &lastTimestamp = sectionData.mTimestamp;

		for (auto &msgData : sectionData.mMsgVector) {
			if (time_timespec_diff_in_range(&lastTimestamp, &msgData->mTimestamp, client->mSampleRate, NULL))
				continue;

			client->sendDataMsg(mDataCtx, msgData->mMsg);

			lastTimestamp = msgData->mTimestamp;
		}

		/* Clear list of messages */
		sectionData.mMsgVector.clear();

		/* Update last timestamp for this section */
		sectionData.mTimestamp = *timestamp;
	}
}

/**
 */
void GndCtrlLoggerCb::sampleEnd(uint32_t sectionId)
{
}

/**
 */
void GndCtrlLoggerCb::sample(uint32_t sectionId,
		const struct timespec *timestamp,
		uint32_t varId,
		const telemetry::VarDesc &varDesc,
		const void *buf, size_t len)
{
	/* Nothing to do, everything was done in sampleBegin for ALL variables */
}

/**
 */
extern "C" int tlm_register_logger(telemetry::Logger *logger,
		struct pomp_loop *loop,
		const std::string &instance,
		telemetry::LoggerCb **cb)
{
	/* Create logger cb */
	GndCtrlLoggerCb *gndCtrlLoggerCb = new GndCtrlLoggerCb(logger, loop, instance);

	/* Start it */
	gndCtrlLoggerCb->start();

	/* Set callback to caller */
	*cb = gndCtrlLoggerCb;
	return 0;
}

/**
 */
extern "C" int tlm_unregister_logger(telemetry::Logger *logger,
		struct pomp_loop *loop,
		const std::string &instance,
		telemetry::LoggerCb *cb)
{
	GndCtrlLoggerCb *gndCtrlLoggerCb = static_cast<GndCtrlLoggerCb *>(cb);
	if (gndCtrlLoggerCb != NULL) {
		gndCtrlLoggerCb->stop();
		delete gndCtrlLoggerCb;
	}
	return 0;
}
