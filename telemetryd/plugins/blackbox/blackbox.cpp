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
 * @file blackbox.c
 *
 */

#include "headers.hpp"
#include "tlmb.hpp"

ULOG_DECLARE_TAG(tlmblackbox);

#define BBX_SETTING_ROOT               "tlm-blackbox"

#define BBX_SETTING_NAME_ENABLED       BBX_SETTING_ROOT ".enabled"
#define BBX_SETTING_NAME_COMPRESSED    BBX_SETTING_ROOT ".compressed"
#define BBX_SETTING_NAME_LOGDIR        BBX_SETTING_ROOT ".logdir"
#define BBX_SETTING_NAME_LOGNAME       BBX_SETTING_ROOT ".logname"
#define BBX_SETTING_NAME_LOGCOUNT      BBX_SETTING_ROOT ".logcount"
#define BBX_SETTING_NAME_FILTER        BBX_SETTING_ROOT ".filter"
#define BBX_SETTING_NAME_FLUSHPERIOD   BBX_SETTING_ROOT ".flushperiod"
#define BBX_SETTING_NAME_MAXSIZE       BBX_SETTING_ROOT ".maxsize"

#ifdef BUILD_LIBSHS
#  define BBX_SETTING_DEF_ENABLED      true
#  define BBX_SETTING_DEF_COMPRESSED   true
#  define BBX_SETTING_DEF_LOGDIR       "/var/lib/tlm-blackbox"
#  define BBX_SETTING_DEF_LOGNAME      "log.tlmb"
#  define BBX_SETTING_DEF_LOGCOUNT     4
#  define BBX_SETTING_DEF_FILTER       "*"
#  define BBX_SETTING_DEF_FLUSHPERIOD  10000
#  define BBX_SETTING_DEF_MAXSIZE      -1
#else /* !BUILD_LIBSHS */
#  define BBX_SETTING_DEF_ENABLED      true
#  define BBX_SETTING_DEF_COMPRESSED   true
#  define BBX_SETTING_DEF_LOGDIR       "."
#  define BBX_SETTING_DEF_LOGNAME      "log.tlmb"
#  define BBX_SETTING_DEF_LOGCOUNT     4
#  define BBX_SETTING_DEF_FILTER       "*"
#  define BBX_SETTING_DEF_FLUSHPERIOD  10000
#  define BBX_SETTING_DEF_MAXSIZE      -1
#endif /* !BUILD_LIBSHS */

/** */
#ifdef BUILD_LIBSHS
template<typename T>
class ShsHelper {
private:
	typedef void (T::*SetBool)(bool val);
	typedef void (T::*SetInt)(int val);
	typedef void (T::*SetDouble)(double val);
	typedef void (T::*SetString)(const char *val);

	union Setter {
		void       *ptr;
		SetBool    setBool;
		SetInt     setInt;
		SetDouble  setDouble;
		SetString  setString;
		inline Setter() : ptr(NULL) {}
		inline Setter(SetBool _setBool) : setBool(_setBool) {}
		inline Setter(SetInt _setInt) : setInt(_setInt) {}
		inline Setter(SetDouble _setDouble) : setDouble(_setDouble) {}
		inline Setter(SetString _setString) : setString(_setString) {}
	};

	typedef std::map<std::string, Setter> SettingMap;
	SettingMap      mSettingMap;
	T               *mParent;
	struct shs_ctx  *mShsCtx;

private:
	inline static void shsChangeCb(
			struct shs_ctx *ctx,
			enum shs_evt evt,
			const struct shs_entry *old_entry,
			const struct shs_entry *new_entry,
			size_t count,
			void *userdata) {
		ShsHelper *self = reinterpret_cast<ShsHelper *>(userdata);
		const Setter &setter = self->mSettingMap[new_entry->name];

		switch (new_entry->value.type) {
		case SHS_TYPE_BOOLEAN:
			(self->mParent->*(setter.setBool))(new_entry->value.val._boolean);
			break;

		case SHS_TYPE_INT:
			(self->mParent->*(setter.setInt))(new_entry->value.val._int);
			break;

		case SHS_TYPE_DOUBLE:
			(self->mParent->*(setter.setDouble))(new_entry->value.val._double);
			break;

		case SHS_TYPE_STRING:
			(self->mParent->*(setter.setString))(new_entry->value.val._cstring);
			break;

		default:
			assert(false);
			break;
		}
	}

	inline void regInternal(const char *name, Setter setter) {
		mSettingMap.insert(typename SettingMap::value_type(name, setter));
	}

public:
	inline ShsHelper(T *parent, struct shs_ctx *shsCtx) {
		mParent = parent;
		mShsCtx = shsCtx;
	}

	inline ~ShsHelper() {}

	inline int reg(const char *name, bool def,
			uint32_t flags, SetBool setBool) {
		int res = shs_ctx_reg_bool(mShsCtx, name, def, flags,
				&ShsHelper::shsChangeCb, this);
		if (res == 0)
			regInternal(name, setBool);
		return res;
	}

	inline int reg(const char *name, int def,
			uint32_t flags, SetInt setInt) {
		int res = shs_ctx_reg_int(mShsCtx, name, def, flags,
				&ShsHelper::shsChangeCb, this);
		if (res == 0)
			regInternal(name, setInt);
		return res;
	}

	inline int reg(const char *name, double def,
			uint32_t flags, SetDouble setDouble) {
		int res = shs_ctx_reg_double(mShsCtx, name, def, flags,
				&ShsHelper::shsChangeCb, this);
		if (res == 0)
			regInternal(name, setDouble);
		return res;
	}

	inline int reg(const char *name, const char *def,
			uint32_t flags, SetString setString) {
		int res = shs_ctx_reg_string(mShsCtx, name, def, flags,
				&ShsHelper::shsChangeCb, this);
		if (res == 0)
			regInternal(name, setString);
		return res;
	}
};
#endif /* BUILD_LIBSHS */

/**
 * Blackbox logger callback interface.
 */
class BbxLoggerCb : public telemetry::LoggerCb {
private:
	telemetry::Logger  *mLogger;   /**< Associated logger */
	struct pomp_loop   *mLoop;     /**< Associated loop */
	struct pomp_timer  *mTimer;    /**< Flush timer */
	std::string        mInstance;  /**< Logger instance */
	LogFile            *mLogFile;  /**< Output log file */

	/** Settings */
#ifdef BUILD_LIBSHS
	struct shs_ctx     *mShsCtx;   /**< Shared settings context */
	ShsHelper<BbxLoggerCb>  *mShsHelper;
#endif /* BUILD_LIBSHS */
	struct {
		bool         mEnabled;     /**< Enabled/Disabled */
		bool         mCompressed;  /**< Compression flag */
		std::string  mLogDir;      /**< Output directory */
		std::string  mLogName;     /**< Name of log file */
		int          mLogCount;    /**< Rotation depth */
		std::string  mFilter;      /**< Filter */
		bool         mLoaded;      /**< Initial settings loaded */
		int          mFlushPeriod; /**< Period of data flushing to disk */
		int          mMaxSize;     /**< Maximum size in bytes of a log file (-1 = no limit)*/
	} mSettings;

private:
	int writeLogFileHeader();
	void checkEndOfBlock();

	void rotateLogs() const;
	std::string getLogFilePath() const;
	void setEnabled(bool val);
	void setCompressed(bool val);
	void setLogDir(const char *val);
	void setLogName(const char *val);
	void setLogCount(int val);
	void setFilter(const char *val);
	void setFlushPeriod(int val);
	void setMaxSize(int val);

	static void timerCb(struct pomp_timer *timer, void *userdata);

	template <typename T> T convertInto(const char *in, T defaultval)
	{
		T rv;
		if (in) {
			std::istringstream stream(in);
			if (stream >> rv)
				return rv;
		}
		return defaultval;
	}
	/* cannot use the template function for const char* type */
	const char* convertInto(const char *in, const char* defaultval)
	{
		if (in)
			return in;
		return defaultval;
	}

public:
	BbxLoggerCb(telemetry::Logger *logger, struct pomp_loop *loop,
			const std::string &instance);
	virtual ~BbxLoggerCb();

	inline bool isEnabled() const {return mSettings.mEnabled;}
	int openLogFile();
	int closeLogFile();

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
	virtual void reset();
};

/**
 */
BbxLoggerCb::BbxLoggerCb(telemetry::Logger *logger, struct pomp_loop *loop,
		const std::string &instance)
{
	/* Initialize parameters */
	mLogger = logger;
	mLoop = loop;
	mInstance = instance;
	mLogFile = NULL;
	mTimer = pomp_timer_new(mLoop, &BbxLoggerCb::timerCb, this);

	/* Make sure no action are taken until ALL settings are loaded */
	mSettings.mLoaded = false;

#ifdef BUILD_LIBSHS
	/* Create the shared settings context */
	uint32_t flags = SHS_FLAG_WRITABLE | SHS_FLAG_PERSISTENT | SHS_FLAG_PUBLIC;
	mShsCtx = shs_ctx_new_server(BBX_SETTING_ROOT, NULL, NULL);
	mShsHelper = new ShsHelper<BbxLoggerCb>(this, mShsCtx);
	mShsHelper->reg(BBX_SETTING_NAME_ENABLED,
			BBX_SETTING_DEF_ENABLED, flags,
			&BbxLoggerCb::setEnabled);
	mShsHelper->reg(BBX_SETTING_NAME_COMPRESSED,
			BBX_SETTING_DEF_COMPRESSED, flags,
			&BbxLoggerCb::setCompressed);
	mShsHelper->reg(BBX_SETTING_NAME_LOGDIR,
			BBX_SETTING_DEF_LOGDIR, flags,
			&BbxLoggerCb::setLogDir);
	mShsHelper->reg(BBX_SETTING_NAME_LOGNAME,
			BBX_SETTING_DEF_LOGNAME, flags,
			&BbxLoggerCb::setLogName);
	mShsHelper->reg(BBX_SETTING_NAME_LOGCOUNT,
			BBX_SETTING_DEF_LOGCOUNT, flags,
			&BbxLoggerCb::setLogCount);
	mShsHelper->reg(BBX_SETTING_NAME_FILTER,
			BBX_SETTING_DEF_FILTER, flags,
			&BbxLoggerCb::setFilter);
	mShsHelper->reg(BBX_SETTING_NAME_FLUSHPERIOD,
			BBX_SETTING_DEF_FLUSHPERIOD, flags,
			&BbxLoggerCb::setFlushPeriod);
	mShsHelper->reg(BBX_SETTING_NAME_MAXSIZE,
			BBX_SETTING_DEF_MAXSIZE, flags,
			&BbxLoggerCb::setMaxSize);

	/* Start context and add in loop */
	shs_ctx_start(mShsCtx);
	shs_ctx_pomp_loop_register(mShsCtx, mLoop);
#else /* !BUILD_LIBSHS */
	/* Directly set default values, either from defines or,
	    if available, from environment variables */
	setEnabled(convertInto(getenv("BBX_SETTING_DEF_ENABLED"), BBX_SETTING_DEF_ENABLED) );
	setCompressed(convertInto(getenv("BBX_SETTING_DEF_COMPRESSED"),	BBX_SETTING_DEF_COMPRESSED) );
	setLogDir(convertInto(getenv("BBX_SETTING_DEF_LOGDIR"),	BBX_SETTING_DEF_LOGDIR) );
	setLogName(convertInto(getenv("BBX_SETTING_DEF_LOGNAME"), BBX_SETTING_DEF_LOGNAME) );
	setLogCount(convertInto(getenv("BBX_SETTING_DEF_LOGCOUNT"),	BBX_SETTING_DEF_LOGCOUNT) );
	setFilter(convertInto(getenv("BBX_SETTING_DEF_FILTER"),	BBX_SETTING_DEF_FILTER) );
	setFlushPeriod(convertInto(getenv("BBX_SETTING_DEF_FLUSHPERIOD"), BBX_SETTING_DEF_FLUSHPERIOD) );
	setMaxSize(convertInto(getenv("BBX_SETTING_DEF_MAXSIZE"), BBX_SETTING_DEF_MAXSIZE) );
#endif /* !BUILD_LIBSHS */

	/* Settings are now fully loaded */
	mSettings.mLoaded = true;
}

/**
 */
BbxLoggerCb::~BbxLoggerCb()
{
#ifdef BUILD_LIBSHS
	/* Stop and destroy shared settings context */
	shs_ctx_pomp_loop_unregister(mShsCtx, mLoop);
	shs_ctx_stop(mShsCtx);
	delete mShsHelper;
	shs_ctx_destroy(mShsCtx);
	mShsCtx = NULL;
	mShsHelper = NULL;
#endif /* BUILD_LIBSHS */

	/* Free timer */
	pomp_timer_destroy(mTimer);

	/* Clear pointers */
	mLogger = NULL;
	mLoop = NULL;
	assert(mLogFile == NULL);
	mTimer = NULL;
}

/**
 */
int BbxLoggerCb::openLogFile()
{
	int res = 0;
	std::string path;
	assert(mLogFile == NULL);

	/* Create log directory (not recursive) */
	if (mkdir(mSettings.mLogDir.c_str(), 0755) < 0 && errno != EEXIST) {
		res = -errno;
		ULOGE("Failed to create directory '%s': err=%d(%s)",
				mSettings.mLogDir.c_str(),
				errno, strerror(errno));
		goto error;
	}

	/* Do a log rotation */
	rotateLogs();

	/* Open log file */
	path = getLogFilePath();
	mLogFile = new LogFile(path, mSettings.mCompressed);
	res = mLogFile->open();
	if (res < 0)
		goto error;

	/* Write header */
	res = writeLogFileHeader();
	if (res < 0)
		goto error;

	/* Begin a new block */
	res = mLogFile->beginBlock();
	if (res < 0)
		goto error;

	/* Ask logger to replay sections events that we could have missed */
	mLogger->replaySectionEvents(this);

	/* Start flushing timer */
	pomp_timer_set_periodic(mTimer, mSettings.mFlushPeriod, mSettings.mFlushPeriod);

	/* OK ! */
	return 0;

	/* Cleanup in case of error */
error:
	closeLogFile();
	return res;
}

/**
 */
int BbxLoggerCb::closeLogFile()
{
	if (mLogFile != NULL) {
		/* Clear flushing timer */
		pomp_timer_clear(mTimer);

		/* Finish file */
		mLogFile->finishFile();

		/* Free resources */
		mLogFile->close();
		delete mLogFile;
		mLogFile = NULL;
	}
	return 0;
}

/**
 */
void BbxLoggerCb::reset()
{
	/* properly close current log file */
	closeLogFile();

	/* and, if needed, open a new one */
	if (isEnabled())
		openLogFile();
}

/**
 */
int BbxLoggerCb::writeLogFileHeader()
{
	int res = 0;
	if (mLogFile == NULL)
		return -EINVAL;

	/* Write file magic and version */
	uint32_t magic = TLMB_MAGIC;
	mLogFile->writeDataRaw(&magic, sizeof(uint32_t));
	uint8_t version = TLMB_VERSION;
	mLogFile->writeDataRaw(&version, sizeof(uint8_t));

	/* Date of log */
	struct timespec ts = {0, 0};
	clock_gettime(CLOCK_REALTIME, &ts);
	struct tm tm;
	memset(&tm, 0, sizeof(tm));
	localtime_r(&ts.tv_sec, &tm);
	char date[128] = "";
	strftime(date, sizeof(date), "%Y%m%d-%H%M%S", &tm);

	/* Setup a map of key/value */
	typedef std::map<std::string, std::string> PropertyMap;
	PropertyMap properties;
	properties["compression"] = mLogFile->isCompressed() ? "1" : "0";
	properties["date"] = date;

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
		"ro.note",
	};
	for (size_t i = 0; i < sizeof(sysprops) / sizeof(sysprops[0]); i++) {
		char value[SYS_PROP_VALUE_MAX] = "";
		sys_prop_get(sysprops[i], value, "");
		properties[sysprops[i]] = value;
	}
#endif /* BUILD_LIBPUTILS */

	/* Property used to write some annotation into the log file */
	if (properties.find("ro.note") == properties.end()) {
		const char *note = getenv("BBX_SETTING_DEF_NOTE");
		if (note)
			properties["ro.note"] = note;
		else
			properties["ro.note"] = "";
	}

	/* Write properties */
	uint8_t strsize = 0;
	uint32_t count = properties.size();
	mLogFile->writeDataRaw(&count, sizeof(uint32_t));
	for (PropertyMap::const_iterator it = properties.begin();
			it != properties.end();
			++it) {
		const std::string &key = it->first;
		const std::string &val = it->second;

		/* Key */
		strsize = (uint8_t)(key.length() + 1);
		mLogFile->writeDataRaw(&strsize, sizeof(uint8_t));
		mLogFile->writeDataRaw(key.c_str(), strsize);

		/* Value */
		strsize = (uint8_t)(val.length() + 1);
		mLogFile->writeDataRaw(&strsize, sizeof(uint8_t));
		mLogFile->writeDataRaw(val.c_str(), strsize);
	}

	return res;
}

/**
 */
void BbxLoggerCb::checkEndOfBlock()
{
	if (mLogFile == NULL)
		return;

	/* Create a new block every 1MB of data */
	uint32_t blockSize = 0;
	mLogFile->getCurrentBlockSize(&blockSize);
	if (blockSize >= 1024 * 1024) {
		mLogFile->finishBlock();
		mLogFile->beginBlock();
	}
}

/**
 */
void BbxLoggerCb::rotateLogs() const
{
	char num[16] = "";
	std::string file0, file1;
	std::string logPath = getLogFilePath();

	for (int i = mSettings.mLogCount; i > 0; i--) {
		snprintf(num, sizeof(num), "%d", i);
		file1 = logPath + "." + num;

		if (i == 1) {
			file0 = logPath;
		} else {
			snprintf(num, sizeof(num), "%d", i - 1);
			file0 = logPath + "." + num;
		}

		ULOGI("rotate logs: '%s' -> '%s'", file0.c_str(), file1.c_str());
		if (rename(file0.c_str(), file1.c_str()) < 0 && errno != ENOENT) {
			ULOGE("Failed to rename '%s' in '%s': err=%d(%s)",
					file0.c_str(), file1.c_str(),
					errno, strerror(errno));
		}
	}
}

/**
 */
std::string BbxLoggerCb::getLogFilePath() const
{
	if (mInstance.empty() || mInstance == "default")
		return mSettings.mLogDir + "/" + mSettings.mLogName;
	else
		return mSettings.mLogDir + "/" + mInstance + "-" + mSettings.mLogName;
}

/**
 */
void BbxLoggerCb::setEnabled(bool val)
{
	/* Save value, update state if all settings have been loaded */
	ULOGI("setEnabled %d", val);
	mSettings.mEnabled = val;
	if (mSettings.mLoaded) {
		if (mSettings.mEnabled && mLogFile == NULL)
			openLogFile();
		else if (!mSettings.mEnabled && mLogFile != NULL)
			closeLogFile();
	}
}

/**
 */
void BbxLoggerCb::setCompressed(bool val)
{
	/* Simply save value */
	ULOGI("setCompressed %d", val);
	mSettings.mCompressed = val;
}

/**
 */
void BbxLoggerCb::setLogDir(const char *val)
{
	/* Simply save value */
	ULOGI("setLogDir '%s'", val);
	mSettings.mLogDir = val;
}

/**
 */
void BbxLoggerCb::setLogName(const char *val)
{
	/* Simply save value */
	ULOGI("setLogName '%s'", val);
	mSettings.mLogName = val;
}

/**
 */
void BbxLoggerCb::setLogCount(int val)
{
	/* Simply save value */
	ULOGI("setLogCount %d", val);
	mSettings.mLogCount = val;
}

/**
 */
void BbxLoggerCb::setFilter(const char *val)
{
	/* Setup logger filter
	 * NOTE: it will apply the filter for other plugins as well */
	ULOGI("setFilter %s", val);
	mSettings.mFilter = val;
	mLogger->setFilter(mSettings.mFilter);
}

/**
 */
void BbxLoggerCb::setFlushPeriod(int val)
{
	ULOGI("setFlushPeriod %d ms", val);
	mSettings.mFlushPeriod = val;
	/* Don't start timer if log file is not opened */
	if(mLogFile != NULL)
		pomp_timer_set_periodic(mTimer, mSettings.mFlushPeriod, mSettings.mFlushPeriod);
}

/**
 */
void BbxLoggerCb::setMaxSize(int val)
{
	ULOGI("setMaxSize %d bytes", val);
	mSettings.mMaxSize = val;
}

/**
 */
void BbxLoggerCb::timerCb(struct pomp_timer *timer, void *userdata)
{
	BbxLoggerCb *self = reinterpret_cast<BbxLoggerCb *>(userdata);
	long position;

	ULOGD("flushing blackbox");
	/* Flush block and start a new one */
	self->mLogFile->finishBlock();

	/* if max size is reached, then perform a "reset" to rotate the logs */
	if ((self->mSettings.mMaxSize != -1) &&
			(self->mLogFile->getPos(&position) >= 0) &&
			(position > self->mSettings.mMaxSize)) {
		self->reset();
        }
	self->mLogFile->beginBlock();
}

/**
 */
void BbxLoggerCb::sectionAdded(uint32_t sectionId)
{
	if (mLogFile == NULL)
		return;

	uint8_t tag = TLMB_TAG_SECTION_ADDED;
	mLogFile->writeData(&tag, sizeof(uint8_t));
	mLogFile->writeData(&sectionId, sizeof(uint32_t));

	/* Set the name of the section, the contents of metadata will be
	 * written in sectionChanged */
	const std::string &section = mLogger->getSection(sectionId);
	uint8_t strsize = (uint8_t)(section.length() + 1);
	mLogFile->writeData(&strsize, sizeof(uint8_t));
	mLogFile->writeData(section.c_str(), strsize);
}

/**
 */
void BbxLoggerCb::sectionRemoved(uint32_t sectionId)
{
	if (mLogFile == NULL)
		return;

	uint8_t tag = TLMB_TAG_SECTION_REMOVED;
	mLogFile->writeData(&tag, sizeof(uint8_t));
	mLogFile->writeData(&sectionId, sizeof(uint32_t));
}

/**
 */
void BbxLoggerCb::sectionChanged(uint32_t sectionId,
		const void *buf, size_t len)
{
	if (mLogFile == NULL)
		return;

	uint8_t tag = TLMB_TAG_SECTION_CHANGED;
	mLogFile->writeData(&tag, sizeof(uint8_t));
	mLogFile->writeData(&sectionId, sizeof(uint32_t));

	/* Directly write the contents of the metadata header */
	uint32_t datasize = len;
	mLogFile->writeData(&datasize, sizeof(uint32_t));
	mLogFile->writeData(buf, datasize);
}

/**
 */
void BbxLoggerCb::sampleBegin(uint32_t sectionId,
		const struct timespec *timestamp,
		const void *buf, size_t len)
{
	if (mLogFile == NULL)
		return;

	uint8_t tag = TLMB_TAG_SAMPLE;
	mLogFile->writeData(&tag, sizeof(uint8_t));
	mLogFile->writeData(&sectionId, sizeof(uint32_t));

	/* Write the timestamp */
	uint32_t sec = timestamp->tv_sec;
	uint32_t nsec = timestamp->tv_nsec;
	mLogFile->writeData(&sec, sizeof(uint32_t));
	mLogFile->writeData(&nsec, sizeof(uint32_t));

	/* Directly write the contents of variable data */
	uint32_t datasize = len;
	mLogFile->writeData(&datasize, sizeof(uint32_t));
	mLogFile->writeData(buf, datasize);
}

/**
 */
void BbxLoggerCb::sampleEnd(uint32_t sectionId)
{
	/* Check if we should finish the current block */
	checkEndOfBlock();
}

/**
 */
void BbxLoggerCb::sample(uint32_t sectionId,
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
	BbxLoggerCb *bbxLoggerCb = new BbxLoggerCb(logger, loop, instance);

	/* Try to open log if needed, do NOT fail, let settings to be updated */
	if (bbxLoggerCb->isEnabled())
		bbxLoggerCb->openLogFile();

	/* Set callback to caller */
	*cb = bbxLoggerCb;
	return 0;
}

/**
 */
extern "C" int tlm_unregister_logger(telemetry::Logger *logger,
		struct pomp_loop *loop,
		const std::string &instance,
		telemetry::LoggerCb *cb)
{
	BbxLoggerCb *bbxLoggerCb = static_cast<BbxLoggerCb *>(cb);
	if (bbxLoggerCb != NULL) {
		bbxLoggerCb->closeLogFile();
		delete bbxLoggerCb;
	}
	return 0;
}
