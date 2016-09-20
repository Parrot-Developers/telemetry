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
 */

#ifndef __STDC_FORMAT_MACROS
#  define __STDC_FORMAT_MACROS
#endif /* !__STDC_FORMAT_MACROS */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netcdf>

#include <map>

#define ULOG_TAG tlmgroundctrl
#include "ulog.h"

#define POMP_ENABLE_ADVANCED_API
#include "libpomp.h"

#include "futils/timetools.h"

#include "libtlmgndctrl.hpp"

#define REC_VAR_NAME "timestamp"
#define REC_DIM_NAME "Time"

ULOG_DECLARE_TAG(tlmgroundctrl);

/**
 */
struct LogVar {
private:
	netCDF::NcVar mVar;

public:
	inline LogVar(netCDF::NcVar var)
	{
		mVar = var;
	}

	inline ~LogVar()
	{
	}

	void putVar(uint32_t start, uint32_t end, const telemetry::VarDesc &varDesc, const std::vector<char> &buf)
	{
		std::vector<size_t> startp, countp;
		startp.push_back(start);
		countp.push_back(end);
		if (varDesc.isArray()) {
			startp.push_back(0);
			countp.push_back(varDesc.getSize());
		}
		mVar.putVar(startp, countp, (const void *)buf.data());
	}

	inline netCDF::NcVar& get()
	{
		return mVar;
	}
};

/**
 */
class LogDim {
private:
	netCDF::NcDim mDim;

public:
	inline LogDim(netCDF::NcDim dim)
	{
		mDim = dim;
	}

	inline ~LogDim()
	{
	}

	inline netCDF::NcDim& get()
	{
		return mDim;
	}
};

/**
 */
class LogSection {
private:
	typedef std::map<uint32_t, LogVar> VarMap;
	typedef std::vector<LogDim> DimVector;
	typedef std::map<uint32_t, LogSection> SectionMap;

protected:
	netCDF::NcGroup *mGroup;
	VarMap mVarMap;
	DimVector mDimVector;
	uint32_t mRecCount;
	SectionMap mSectionMap;
	netCDF::NcVar mRecVar;

public:
	inline LogSection(netCDF::NcGroup *group) : mGroup(group), mRecCount(0)
	{
		if (mGroup != NULL)
			addRec();
	}

	inline virtual ~LogSection()
	{
		mVarMap.clear();
		mDimVector.clear();
		mSectionMap.clear();

		if (mGroup != NULL) {
			delete mGroup;
			mGroup = NULL;
		}
	}

	inline bool isNull() const
	{
		if (mGroup != NULL)
			return mGroup->isNull();
		return true;
	}

	inline void addSection(uint32_t sectionId, const std::string &sectionName)
	{
		mSectionMap.emplace(sectionId, new netCDF::NcGroup(mGroup->addGroup(sectionName)));
	}

	inline void removeSection(uint32_t sectionId)
	{
		mSectionMap.erase(sectionId);
	}

	inline LogSection* getSection(uint32_t sectionId)
	{
		auto section = mSectionMap.find(sectionId);
		if (section == mSectionMap.end())
		{
			ULOGE("Failed to get section '%d'", sectionId);
			return NULL;
		}
		return &section->second;
	}

	inline LogDim& addDim(const std::string &name)
	{
		mDimVector.push_back(LogDim(mGroup->addDim(name)));
		return mDimVector.back();
	}

	inline LogDim& addDim(const std::string &name, uint32_t dimSize)
	{
		mDimVector.push_back(LogDim(mGroup->addDim(name, dimSize)));
		return mDimVector.back();
	}

	inline void addVar(uint32_t varId, const telemetry::VarDesc &varDesc)
	{
		netCDF::NcType::ncType type;
		switch (varDesc.getType()) {
		case TLM_TYPE_BOOL:
			type = netCDF::NcType::nc_UBYTE;
			break;

		case TLM_TYPE_UINT8:
			type = netCDF::NcType::nc_UBYTE;
			break;

		case TLM_TYPE_INT8:
			type = netCDF::NcType::nc_BYTE;
			break;

		case TLM_TYPE_UINT16:
			type = netCDF::NcType::nc_USHORT;
			break;

		case TLM_TYPE_INT16:
			type = netCDF::NcType::nc_SHORT;
			break;

		case TLM_TYPE_UINT32:
			type = netCDF::NcType::nc_UINT;
			break;

		case TLM_TYPE_INT32:
			type = netCDF::NcType::nc_INT;
			break;

		case TLM_TYPE_UINT64:
			type = netCDF::NcType::nc_UINT64;
			break;

		case TLM_TYPE_INT64:
			type = netCDF::NcType::nc_INT64;
			break;

		case TLM_TYPE_FLOAT32:
			type = netCDF::NcType::nc_FLOAT;
			break;

		case TLM_TYPE_FLOAT64:
			type = netCDF::NcType::nc_DOUBLE;
			break;

		case TLM_TYPE_STRING:
			type = netCDF::NcType::nc_STRING;
			break;

		case TLM_TYPE_BINARY: /* NO BREAK */
			// TODO Add support for binary type
			ULOGW("Unsupported variable type.");
			return;

		case TLM_TYPE_INVALID: /* NO BREAK */
		default:
			return;
		}
		if (varDesc.isArray()) {
			LogDim &varDim = this->addDim(varDesc.getName(), varDesc.getCount());
			std::vector<netCDF::NcDim> dimVector;
			dimVector.push_back(mDimVector[0].get());
			dimVector.push_back(varDim.get());
			mVarMap.emplace(varId, mGroup->addVar(varDesc.getName(), type, dimVector));
		} else {
			mVarMap.emplace(varId, mGroup->addVar(varDesc.getName(), type, mDimVector[0].get()));
		}
	}

	inline LogVar* getVar(uint32_t varId)
	{
		auto var = mVarMap.find(varId);
		if (var == mVarMap.end())
		{
			ULOGE("Failed to get variable '%d'", varId);
			return NULL;
		}
		return &var->second;
	}

	inline void putRec(const std::vector<uint64_t> &recVector)
	{
		std::vector<size_t> startp, countp;
		startp.push_back(mRecCount);
		countp.push_back(recVector.size());
		mRecVar.putVar(startp, countp, recVector.data());
		mRecCount += recVector.size();
	}

	inline uint32_t getRecCount() const
	{
		return mRecCount;
	}

	inline void changeVar(uint32_t varId, const telemetry::VarDesc &varDesc)

	{
		addVar(varId, varDesc);
	}

	inline netCDF::NcGroup& get()
	{
		return *mGroup;
	}

	inline void putProperty(const std::string &name, const std::string &value)
	{
		mGroup->putAtt(name, value);
	}

protected:
	inline void addRec()
	{
		LogDim &dim = addDim(REC_DIM_NAME);  // Adds an unlimited dimension for time
		mRecVar = mGroup->addVar(REC_VAR_NAME, netCDF::NcType::nc_UINT64, dim.get());
	}

};

/**
 */
class LogFile : public LogSection {
public:
	inline LogFile() : LogSection(NULL)
	{
	}

	inline ~LogFile()
	{
	}

	inline void open(const char *path)
	{
		if (path == NULL)
			return;
		try {
			mGroup = new netCDF::NcFile(path, netCDF::NcFile::replace);
		} catch (netCDF::exceptions::NcException &e) {
			ULOGE("Failed to open filename '%s': %s", path, e.what());
		}
	}

	inline void close()
	{
		mRecCount = 0;
		mVarMap.clear();
		mDimVector.clear();
		mSectionMap.clear();

		if (mGroup != NULL) {
			delete mGroup;
			mGroup = NULL;
		}
	}

	inline void sync()
	{
		reinterpret_cast<netCDF::NcFile *>(mGroup)->sync();
	}

	inline netCDF::NcFile& get()
	{
		return *reinterpret_cast<netCDF::NcFile *>(mGroup);
	}
};

/**
 */
class GndCtrlApp : public telemetry::GndCtrlItfCb {
private:
	typedef std::map<uint32_t, std::vector<char> > VarMap;
	typedef std::vector<uint64_t> RecVector;
	struct SectionData {
		uint32_t mMsgCount;
		uint64_t mSavedTimestamp;
		RecVector mRecVector;
		VarMap mVarMap;

		inline SectionData() {
			mMsgCount = 0;
			mSavedTimestamp = 0;
		}
	};
	typedef std::map<uint32_t, SectionData> SectionDataMap;

private:
	struct pomp_loop       *mLoop;
	telemetry::GndCtrlItf  *mGndCtrlItf;
	volatile bool          mQuit;
	LogFile                mLogFile;
	const char            *mPath;
	SectionDataMap         mSectionDataMap;
	uint32_t               mNextMsgCount;
	uint32_t               mMsgRate;

public:
	GndCtrlApp(const std::string &name, const std::string &ctrlAddr, uint32_t dataPort, uint32_t sampleRate, uint32_t msgRate, const char *path);
	~GndCtrlApp();

	int start();
	int stop();
	void run();
	void quit();

public:
	virtual void sectionAdded(uint32_t sectionId);
	virtual void sectionRemoved(uint32_t sectionId);
	virtual void sectionChanged(uint32_t sectionId, const void *buf, size_t len);
	virtual void sampleBegin(uint32_t sectionId,
			const struct timespec *timestamp,
			const void *buf, size_t len);
	void sampleEnd(uint32_t sectionId);
	void sample(uint32_t sectionId,
			const struct timespec *timestamp,
			uint32_t varId,
			const telemetry::VarDesc &varDesc,
			const void *buf, size_t len);

	virtual void connected(const telemetry::GndCtrlItfCb::PropertyMap &properties);
	virtual void disconnected();
};

/**
 */
GndCtrlApp::GndCtrlApp(const std::string &name, const std::string &ctrlAddr,
		uint32_t dataPort, uint32_t sampleRate, uint32_t msgRate, const char *path)
{
	mLoop = pomp_loop_new();
	mGndCtrlItf = telemetry::GndCtrlItf::create(this, mLoop, name, ctrlAddr, dataPort, sampleRate, msgRate);
	mQuit = false;
	mPath = path;
	mNextMsgCount = 1;
	mMsgRate = msgRate;
}

/**
 */
GndCtrlApp::~GndCtrlApp()
{
	/* Free resources */
	delete mGndCtrlItf;
	pomp_loop_destroy(mLoop);
	mLoop = NULL;
	mGndCtrlItf = NULL;
}

/**
 */
void GndCtrlApp::sectionAdded(uint32_t sectionId)
{
	ULOGI("sectionAdded: %s(%d)",
			mGndCtrlItf->getSection(sectionId).c_str(), sectionId);

	if (mLogFile.isNull())
		return;

	mLogFile.addSection(sectionId, mGndCtrlItf->getSection(sectionId));
	mSectionDataMap.insert(SectionDataMap::value_type(sectionId, SectionData()));
}

/**
 */
void GndCtrlApp::sectionRemoved(uint32_t sectionId)
{
	ULOGI("sectionRemoved: %s(%d)",
			mGndCtrlItf->getSection(sectionId).c_str(), sectionId);

	if (mLogFile.isNull())
		return;

	mLogFile.removeSection(sectionId);
}

/**
 */
void GndCtrlApp::sectionChanged(uint32_t sectionId, const void *buf, size_t len)
{
	ULOGI("sectionChanged: %s(%d)",
			mGndCtrlItf->getSection(sectionId).c_str(), sectionId);

	if (mLogFile.isNull())
		return;

	LogSection *section = mLogFile.getSection(sectionId);
	if (section != NULL) {
		uint32_t varCount = mGndCtrlItf->getVarCount(sectionId);
		for (uint32_t i = 0; i < varCount; i++) {
			const telemetry::VarDesc &varDesc = mGndCtrlItf->getVarDesc(sectionId, i);
			section->changeVar(i, varDesc);
		}
	}
}

/**
 */
void GndCtrlApp::sampleBegin(uint32_t sectionId,
		const struct timespec *timestamp,
		const void *buf, size_t len)
{
	ULOGD("sampleBegin: %s(%d) %ld.%06ld",
			mGndCtrlItf->getSection(sectionId).c_str(), sectionId,
			timestamp->tv_sec, timestamp->tv_nsec / 1000);

	if (mGndCtrlItf->getVarCount(sectionId) == 0)
		return;

	if (mLogFile.isNull())
		return;

	// Add timestamp in record container
	SectionData &sd = mSectionDataMap[sectionId];
	uint64_t us;
	time_timespec_to_us(timestamp, &us);
	sd.mRecVector.push_back(us);
}

/**
 */
void GndCtrlApp::sampleEnd(uint32_t sectionId)
{
	if (mLogFile.isNull())
		return;

	SectionData &sd = mSectionDataMap[sectionId];

	uint64_t &savedTimestamp = sd.mSavedTimestamp;
	uint64_t currentTimestamp = sd.mRecVector.back();
	if (savedTimestamp + mMsgRate <= currentTimestamp) {
		sd.mMsgCount++;
		savedTimestamp = currentTimestamp;

		LogSection *section = mLogFile.getSection(sectionId);
		if (section != NULL) {
			uint32_t start = section->getRecCount();

			// Update netCDF list of records for this section
			section->putRec(sd.mRecVector);

			// Update netCDF list of samples for this section
			for (auto &sample : sd.mVarMap) {
				LogVar *var = section->getVar(sample.first);
				if (var != NULL) {
					const telemetry::VarDesc &varDesc = mGndCtrlItf->getVarDesc(sectionId, sample.first);
					var->putVar(start, sd.mRecVector.size(), varDesc, sample.second);
				}
			}
		}

		// Clear temporary containers
		sd.mVarMap.clear();
		sd.mRecVector.clear();
	}

	for (auto &sectionData : mSectionDataMap)
		if (sectionData.second.mMsgCount < mNextMsgCount)
			return;

	// Sync netCDF file once all sections have received a new message
	mLogFile.sync();
	mNextMsgCount++;
}


/**
 */
template<typename T>
static T getVar(const void *buf, size_t len)
{
	assert(len >= sizeof(T));
	T val;
	memcpy(&val, buf, sizeof(T));
	return val;
}
/**
 */
void GndCtrlApp::sample(uint32_t sectionId,
		const struct timespec * /*timestamp*/,
		uint32_t varId,
		const telemetry::VarDesc &varDesc,
		const void *buf, size_t len)
{
	if (mLogFile.isNull())
		return;

	// Add sample in variable container
	SectionData &sd = mSectionDataMap[sectionId];
	std::vector<char> &varVec = sd.mVarMap[varId];
	varVec.insert(varVec.end(), (const uint8_t *)buf, ((const uint8_t *)buf) + len);
}

/**
 */
void GndCtrlApp::connected(const telemetry::GndCtrlItfCb::PropertyMap &properties)
{
	telemetry::GndCtrlItfCb::PropertyMap::const_iterator it;
	ULOGI("Connected");
	for (it = properties.begin(); it != properties.end(); ++it) {
		ULOGI("%s='%s'", it->first.c_str(), it->second.c_str());
	}
	mLogFile.open(mPath);

	if (mLogFile.isNull())
		return;

	for (it = properties.begin(); it != properties.end(); ++it) {
		mLogFile.putProperty(it->first, it->second);
	}
}

/**
 */
void GndCtrlApp::disconnected()
{
	ULOGI("Disconnected");
	mLogFile.close();
}

/**
 */
int GndCtrlApp::start()
{
	return mGndCtrlItf->start();
}

/**
 */
int GndCtrlApp::stop()
{
	return mGndCtrlItf->stop();
}

/**
 */
void GndCtrlApp::run()
{
	ULOGI("Entering loop");

	/* Run loop until asked to quit */
	while (!mQuit)
		pomp_loop_wait_and_process(mLoop, -1);

	ULOGI("Exiting loop");
}

/**
 */
void GndCtrlApp::quit()
{
	/* Set quit flag, wakeup loop */
	mQuit = true;
	pomp_loop_wakeup(mLoop);
}

/**
 * Global application object, just so signal handler can access it.
 */
static GndCtrlApp *sApp;

/**
 */
static void sighandler(int signo)
{
	/* Ask daemon to quit */
	ULOGI("sighandler: signo=%d(%s)", signo, strsignal(signo));
	sApp->quit();
}

/**
 */
static void usage(const char *progname)
{
	fprintf(stderr, "usage: %s [<options>] <ctrladdr> <dataport>\n", progname);
}

/**
 */
int main(int argc, char *argv[])
{
	int argidx = 0;
	int status = EXIT_SUCCESS;
	std::string ctrlAddr;
	int dataPort = 0;
	int sampleRate = 200 * 1000;
	int msgRate = 200 * 1000;
	const char *path = NULL;

	/* Parse options */
	for (argidx = 1; argidx < argc; argidx++) {
		if (argv[argidx][0] != '-') {
			/* End of options */
			break;
		} else if (strcmp(argv[argidx], "-h") == 0
				|| strcmp(argv[argidx], "--help") == 0) {
			/* Help */
			usage(argv[0]);
			goto out;
		} else if (strcmp(argv[argidx], "-s") == 0
				&& argidx + 1 < argc) {
			/* Sample rate */
			sampleRate = atoi(argv[++argidx]);
		} else if (strcmp(argv[argidx], "-m") == 0
				&& argidx + 1 < argc) {
			/* Message rate */
			msgRate = atoi(argv[++argidx]);
		} else if (strcmp(argv[argidx], "-o") == 0
				&& argidx + 1 < argc) {
			/* Path */
			path = argv[++argidx];
		} else {
			ULOGE("Unknown option: '%s'", argv[argidx]);
			goto error;
		}
	}

	/* Get control address */
	if (argc - argidx >= 1) {
		ctrlAddr = argv[argidx++];
	} else {
		ULOGE("Missing address");
		goto error;
	}

	/* Get data port */
	if (argc - argidx >= 1) {
		dataPort = atoi(argv[argidx++]);
	} else {
		ULOGI("No data port specified, tell server to use TCP protocol to send samples.");
	}

	/* Create application object, setup signal handler */
	sApp = new GndCtrlApp("gndctrl-client-netcdf", ctrlAddr, dataPort, sampleRate, msgRate, path);
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);

	/* Run the application */
	sApp->start();
	sApp->run();
	sApp->stop();
	status = EXIT_SUCCESS;
	goto out;

error:
	status = EXIT_FAILURE;
out:
	/* Cleanup */
	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	if (sApp != NULL) {
		delete sApp;
		sApp = NULL;
	}
	return status;
}
