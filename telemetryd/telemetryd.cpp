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
 * @file telemetryd.c
 *
 * @brief Telemetry daemon.
 *
 * It loads plugins found in /usr/lib/tlm-plugins and notify them when there
 * is new samples found in shared memory. Each plugins is responsible for
 * managing the sample in the ways it sees fit (saving in flash, sending over
 * the network...)
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <dlfcn.h>
#include <signal.h>
#include <unistd.h>

#include <string>
#include <map>
#include <vector>

#define ULOG_TAG telemetryd
#include "ulog.h"
ULOG_DECLARE_TAG(telemetryd);

#include "libtelemetry.hpp"
#include "libpomp.hpp"

/** Directory when plugins will be loaded */
#define TLM_PLUGINS_DIR  "/usr/lib/tlm-plugins"

/** Default refresh rate in ms */
#define TLM_DEFAULT_RATE  1000

typedef std::vector<std::string> InstanceDirVector;

/**
 * Plugin class.
 */
class TlmPlugin {
private:
	/* Function prototypes found in plugins */
	typedef int (*TlmPluginRegisterLoggerFunc)(telemetry::Logger *logger,
			struct pomp_loop *loop,
			const std::string &instance,
			telemetry::LoggerCb **cb);
	typedef int (*TlmPluginUnregisterLoggerFunc)(telemetry::Logger *logger,
			struct pomp_loop *loop,
			const std::string &instance,
			telemetry::LoggerCb *cb);

	std::string                    mPath;
	void                           *mHandle;
	TlmPluginRegisterLoggerFunc    mRegisterLoggerFunc;
	TlmPluginUnregisterLoggerFunc  mUnregisterLoggerFunc;

public:
	TlmPlugin(const std::string &path);
	~TlmPlugin();

	int load();
	void unload();

	const std::string getPath() const {return mPath;}

	inline int registerLogger(telemetry::Logger *logger,
			struct pomp_loop *loop,
			const std::string &instance,
			telemetry::LoggerCb **cb) {
		return (*mRegisterLoggerFunc)(logger, loop, instance, cb);
	}

	inline int unregisterLogger(telemetry::Logger *logger,
			struct pomp_loop *loop,
			const std::string &instance,
			telemetry::LoggerCb *cb) {
		return (*mUnregisterLoggerFunc)(logger, loop, instance, cb);
	}
};

/**
 * Daemon class.
 */
class TlmDaemon : public telemetry::LoggerCb {
private:
	struct PluginLoggerCb {
		TlmPlugin            *mPlugin;
		telemetry::Logger    *mLogger;
		std::string          mInstance;
		telemetry::LoggerCb  *mCb;

		inline PluginLoggerCb() {
			mPlugin = NULL;
			mLogger = NULL;
			mCb = NULL;
		}
	};

	typedef std::map<std::string, telemetry::Logger *>  LoggerMap;
	typedef std::map<std::string, TlmPlugin *>          PluginMap;
	typedef std::vector<PluginLoggerCb>                 PluginLoggerCbVector;

	LoggerMap             mLoggerMap;
	PluginMap             mPluginMap;
	PluginLoggerCbVector  mPluginLoggerCbVector;
	uint32_t              mRate;
	volatile bool         mQuit;
	struct pomp_loop      *mLoop;
	struct pomp_timer     *mTimer;

private:
	static void timerCb(struct pomp_timer *timer, void *userdata);
	void registerPluginLoggers(TlmPlugin *plugin);
	void unregisterPluginLoggers(TlmPlugin *plugin);

public:
	TlmDaemon(uint32_t rate, const InstanceDirVector &instanceDirVector);
	virtual ~TlmDaemon();

	int loadPlugins(const std::string &dirpath);
	void unloadPlugins();
	void run();
	void quit();
	void reset();

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
TlmPlugin::TlmPlugin(const std::string &path)
{
	/* Initialize parameters */
	mPath = path;
	mHandle = NULL;
	mRegisterLoggerFunc = NULL;
	mUnregisterLoggerFunc = NULL;
}

/**
 */
TlmPlugin::~TlmPlugin()
{
}

/**
 */
int TlmPlugin::load()
{
	int res = 0;
	ULOGI("Loading '%s'", mPath.c_str());

	/* Do NOT put static as it reference members of this object */
	const struct {
		const char *name;
		void **ptr;
	} functions[] = {
		{"tlm_register_logger", (void **)&mRegisterLoggerFunc},
		{"tlm_unregister_logger", (void **)&mUnregisterLoggerFunc},
	};

	/* Load library */
	mHandle = dlopen(mPath.c_str(), RTLD_NOW);
	if (mHandle == NULL) {
		res = -EINVAL;
		ULOGE("%s", dlerror());
		goto error;
	}

	/* Get function pointers */
	for (size_t i = 0; i < sizeof(functions) / sizeof(functions[0]); i++) {
		*functions[i].ptr = dlsym(mHandle, functions[i].name);
		if (*functions[i].ptr == NULL) {
			res = -EINVAL;
			ULOGE("%s", dlerror());
			goto error;
		}
	}

	return 0;

	/* Cleanup in case of error */
error:
	if (mHandle != NULL)
		dlclose(mHandle);
	mHandle = NULL;
	mRegisterLoggerFunc = NULL;
	mUnregisterLoggerFunc = NULL;
	return res;
}

/**
 */
void TlmPlugin::unload()
{
	if (mHandle == NULL)
		return;
	ULOGI("Unloading '%s'", mPath.c_str());

	/* Unload library */
	dlclose(mHandle);
	mHandle = NULL;
	mRegisterLoggerFunc = NULL;
	mUnregisterLoggerFunc = NULL;
}

/**
 */
TlmDaemon::TlmDaemon(uint32_t rate, const InstanceDirVector &instanceDirVector)
{
	mRate = rate;
	mQuit = false;
	ULOGI("Creating daemon with rate %d ms", rate);

	/* Setup loggers */
	for (InstanceDirVector::const_iterator it = instanceDirVector.begin();
			it != instanceDirVector.end();
			++it) {
		/* Split instance:dir */
		const std::string &instanceDir = *it;
		std::string instance, dir;
		size_t pos = instanceDir.find(':');
		if (pos == std::string::npos) {
			instance = "default";
			dir = instanceDir;
			if (dir == "default")
				dir = "";
		} else {
			instance = instanceDir.substr(0, pos);
			dir = instanceDir.substr(pos + 1);
		}

		/* Create logger and add it in map */
		telemetry::Logger *logger = telemetry::Logger::create(dir);
		mLoggerMap.insert(LoggerMap::value_type(instance, logger));
	}

	/* Create loop and timer */
	mLoop = pomp_loop_new();
	mTimer = pomp_timer_new(mLoop, &TlmDaemon::timerCb, this);
}

/**
 */
TlmDaemon::~TlmDaemon()
{
	int res = 0;

	/* Cleanup timer */
	res = pomp_timer_destroy(mTimer);
	if (res < 0)
		ULOGE("pomp_timer_destroy: err=%d(%s)", res, strerror(-res));
	mTimer = NULL;

	/* Cleanup loop */
	res = pomp_loop_destroy(mLoop);
	if (res < 0)
		ULOGE("pomp_loop_destroy: err=%d(%s)", res, strerror(-res));
	mLoop = NULL;

	/* Cleanup loggers */
	for (LoggerMap::const_iterator it = mLoggerMap.begin();
			it != mLoggerMap.end();
			++it) {
		telemetry::Logger *logger = it->second;
		telemetry::Logger::release(logger);
	}
	mLoggerMap.clear();
}

/**
 */
int TlmDaemon::loadPlugins(const std::string &dirpath)
{
	int res = 0;
	DIR *dir = NULL;
	struct dirent entry;
	struct dirent *rentry = NULL;
	std::string path;
	TlmPlugin *plugin = NULL;
	ULOGI("Loading plugins from '%s'", dirpath.c_str());

	/* Open plugins directory */
	dir = opendir(dirpath.c_str());
	if (dir == NULL) {
		res = -errno;
		ULOGE("opendir '%s': err=%d(%s)", dirpath.c_str(), errno,
				strerror(errno));
		goto out;
	}

	/* Load plugins */
	while (readdir_r(dir, &entry, &rentry) == 0 && rentry != NULL) {
		if (strcmp(entry.d_name, ".") == 0)
			continue;
		if (strcmp(entry.d_name, "..") == 0)
			continue;

		/* Load plugin */
		path = dirpath + "/" + entry.d_name;
		plugin = new TlmPlugin(path);
		if (plugin->load() < 0) {
			delete plugin;
		} else {
			mPluginMap.insert(PluginMap::value_type(path, plugin));
			registerPluginLoggers(plugin);
		}
	}

	if (mPluginMap.size() == 0)
		ULOGW("No plugins found");

	/* Cleanup before exiting */
out:
	if (dir != NULL)
		closedir(dir);
	return res;
}

/**
 */
void TlmDaemon::unloadPlugins()
{
	PluginMap::iterator it = mPluginMap.begin();
	while (it != mPluginMap.end()) {
		TlmPlugin *plugin = it->second;
		unregisterPluginLoggers(plugin);
		plugin->unload();
		delete plugin;
		it = mPluginMap.erase(it);
	}
}

/**
 */
void TlmDaemon::registerPluginLoggers(TlmPlugin *plugin)
{
	int res = 0;

	/* Register loggers and get callbacks */
	for (LoggerMap::const_iterator it = mLoggerMap.begin();
			it != mLoggerMap.end();
			++it) {
		const std::string &instance = it->first;
		telemetry::Logger *logger = it->second;
		telemetry::LoggerCb *cb = NULL;
		res = plugin->registerLogger(logger, mLoop, instance, &cb);
		if (res < 0) {
			/* Not a fatal error for this plugin */
			ULOGW("Failed to register logger '%s' in plugin '%s'",
					instance.c_str(),
					plugin->getPath().c_str());
		} else {
			/* Add it table */
			PluginLoggerCb plcb;
			plcb.mPlugin = plugin;
			plcb.mLogger = logger;
			plcb.mInstance = instance;
			plcb.mCb = cb;
			mPluginLoggerCbVector.push_back(plcb);
		}
	}
}

/**
 */
void TlmDaemon::unregisterPluginLoggers(TlmPlugin *plugin)
{
	/* Register loggers of this plugin */
	PluginLoggerCbVector::iterator it = mPluginLoggerCbVector.begin();
	while (it != mPluginLoggerCbVector.end()) {
		const PluginLoggerCb &plcb = *it;
		if (plcb.mPlugin == plugin) {
			plugin->unregisterLogger(plcb.mLogger, mLoop,
					plcb.mInstance, plcb.mCb);
			it = mPluginLoggerCbVector.erase(it);
		} else {
			++it;
		}
	}
}

/**
 */
void TlmDaemon::run()
{
	ULOGI("Entering loop");

	/* Setup timer */
	pomp_timer_set_periodic(mTimer, mRate, mRate);

	/* Run loop until asked to quit */
	while (!mQuit) {
		pomp_loop_wait_and_process(mLoop, -1);
	}

	/* Get last batch of data */
	TlmDaemon::timerCb(NULL, this);

	/* Stop timer */
	pomp_timer_clear(mTimer);

	ULOGI("Exiting loop");
}

/**
 */
void TlmDaemon::quit()
{
	/* Set quit flag, wakeup loop */
	mQuit = true;
	pomp_loop_wakeup(mLoop);
}

/**
 */
void TlmDaemon::reset()
{
	/* reset stored timestamps in logger(s) */
	for (LoggerMap::const_iterator it = mLoggerMap.begin();
			it != mLoggerMap.end();
			++it) {
		it->second->reset();
	}

	/* Forward to plugins callbacks */
	for (PluginLoggerCbVector::const_iterator it = mPluginLoggerCbVector.begin();
			it != mPluginLoggerCbVector.end();
			++it) {
		it->mCb->reset();
	}
}

/**
 */
void TlmDaemon::timerCb(struct pomp_timer *timer, void *userdata)
{
	/* Read samples and process them in callbacks */
	TlmDaemon *self = reinterpret_cast<TlmDaemon *>(userdata);
	for (LoggerMap::const_iterator it = self->mLoggerMap.begin();
			it != self->mLoggerMap.end();
			++it) {
		telemetry::Logger *logger = it->second;
		logger->fetchNextSamples(self);
	}
}

/**
 */
void TlmDaemon::sectionAdded(uint32_t sectionId)
{
	/* Forward to plugins callbacks */
	for (PluginLoggerCbVector::const_iterator it = mPluginLoggerCbVector.begin();
			it != mPluginLoggerCbVector.end();
			++it) {
		it->mCb->sectionAdded(sectionId);
	}
}

/**
 */
void TlmDaemon::sectionRemoved(uint32_t sectionId)
{
	/* Forward to plugins callbacks */
	for (PluginLoggerCbVector::const_iterator it = mPluginLoggerCbVector.begin();
			it != mPluginLoggerCbVector.end();
			++it) {
		it->mCb->sectionRemoved(sectionId);
	}
}

/**
 */
void TlmDaemon::sectionChanged(uint32_t sectionId, const void *buf, size_t len)
{
	/* Forward to plugins callbacks */
	for (PluginLoggerCbVector::const_iterator it = mPluginLoggerCbVector.begin();
			it != mPluginLoggerCbVector.end();
			++it) {
		it->mCb->sectionChanged(sectionId, buf, len);
	}
}

/**
 */
void TlmDaemon::sampleBegin(uint32_t sectionId,
		const struct timespec *timestamp,
		const void *buf, size_t len)
{
	/* Forward to plugins callbacks */
	for (PluginLoggerCbVector::const_iterator it = mPluginLoggerCbVector.begin();
			it != mPluginLoggerCbVector.end();
			++it) {
		it->mCb->sampleBegin(sectionId, timestamp, buf, len);
	}
}

/**
 */
void TlmDaemon::sampleEnd(uint32_t sectionId)
{
	/* Forward to plugins callbacks */
	for (PluginLoggerCbVector::const_iterator it = mPluginLoggerCbVector.begin();
			it != mPluginLoggerCbVector.end();
			++it) {
		it->mCb->sampleEnd(sectionId);
	}
}

/**
 */
void TlmDaemon::sample(uint32_t sectionId,
		const struct timespec *timestamp,
		uint32_t varId,
		const telemetry::VarDesc &varDesc,
		const void *buf, size_t len)
{
	/* Forward to plugins callbacks */
	for (PluginLoggerCbVector::const_iterator it = mPluginLoggerCbVector.begin();
			it != mPluginLoggerCbVector.end();
			++it) {
		it->mCb->sample(sectionId, timestamp, varId, varDesc, buf, len);
	}
}

/**
 * Global daemon object, just so signal handler can access it.
 */
static TlmDaemon *sDaemon;

/**
 */
static void sighandler(int signo)
{
	/* Ask daemon to quit */
	ULOGI("%s: signo=%d(%s)", __func__, signo, strsignal(signo));
	sDaemon->quit();
}

/**
 */
static void sigreset(int signo)
{
	/* Ask daemon to reset its loggers */
	ULOGI("%s: signo=%d(%s)", __func__, signo, strsignal(signo));
	sDaemon->reset();
}

/**
 */
static void usage(const char *progname)
{
	fprintf(stderr, "usage: %s [<options>] [<instance>:<dir>...]\n",
			progname);
	fprintf(stderr, "Telemetry daemon. Continuously read the contents of\n");
	fprintf(stderr, "shared memory sections and forward them to loaded\n");
	fprintf(stderr, "plugins.\n");
	fprintf(stderr, "<instance>:<dir>: list of directories to scan for sections.\n");
	fprintf(stderr, "If none given, /dev/shm will be scanned.\n");
	fprintf(stderr, "<instance> will be used to identify the instance in output logs\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "  -h --help : print this help message and exit\n");
	fprintf(stderr, "  -r --rate : read rate in ms.\n");
	fprintf(stderr, "              (default: %d)\n", TLM_DEFAULT_RATE);
	fprintf(stderr, "  -p --plugins-dir : directory where plugins are.\n");
	fprintf(stderr, "                     (default: %s)\n", TLM_PLUGINS_DIR);
	fprintf(stderr, "  -o --output-dir : output directory.\n");
	fprintf(stderr, "                    (default: current directory)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Note: without libshsettings, plugins will use the current\n");
	fprintf(stderr, "directory for output unless -o (--output-dir) option.\n");
	fprintf(stderr, "is given\n");
}

/**
 */
int main(int argc, char *argv[])
{
	int argidx = 0;
	int status = EXIT_SUCCESS;
	uint32_t rate = TLM_DEFAULT_RATE;
	std::string pluginsDir = TLM_PLUGINS_DIR;
	InstanceDirVector instanceDirVector;
	std::string outputDir = ".";

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
		} else if (strcmp(argv[argidx], "-r") == 0
				|| strcmp(argv[argidx], "--rate") == 0) {
			if (++argidx >= argc) {
				ULOGE("Missing rate");
				goto error;
			}
			rate = atoi(argv[argidx]);
		} else if (strcmp(argv[argidx], "-p") == 0
				|| strcmp(argv[argidx], "--plugins-dir") == 0) {
			if (++argidx >= argc) {
				ULOGE("Missing plugins directory");
				goto error;
			}
			pluginsDir = argv[argidx];
		} else if (strcmp(argv[argidx], "-o") == 0
				|| strcmp(argv[argidx], "--output-dir") == 0) {
			if (++argidx >= argc) {
				ULOGE("Missing output directory");
				goto error;
			}
			outputDir = argv[argidx];
		} else {
			ULOGE("Unknown option: '%s'", argv[argidx]);
			goto error;
		}
	}

	/* Remaining arguments are instance:dir specification */
	while (argidx < argc) {
		instanceDirVector.push_back(argv[argidx]);
		argidx++;
	}

	/* If nothing was given use a default specification */
	if (instanceDirVector.empty())
		instanceDirVector.push_back("default");

	/* Change current directory to output directory if needed */
	if (outputDir != "." && chdir(outputDir.c_str()) < 0) {
		ULOGW("Unable to chdir to '%s': err=%d(%s)", outputDir.c_str(),
				errno, strerror(errno));
	}

	/* Create daemon object, setup signal handler */
	sDaemon = new TlmDaemon(rate, instanceDirVector);
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGUSR1, &sigreset);
	signal(SIGPIPE, SIG_IGN);

	/* Run the daemon with plugins */
	sDaemon->loadPlugins(pluginsDir);
	sDaemon->run();
	sDaemon->unloadPlugins();
	status = EXIT_SUCCESS;
	goto out;

error:
	status = EXIT_FAILURE;
out:
	/* Cleanup */
	signal(SIGINT, SIG_DFL);
	signal(SIGTERM, SIG_DFL);
	signal(SIGPIPE, SIG_DFL);
	if (sDaemon != NULL) {
		delete sDaemon;
		sDaemon = NULL;
	}
	return status;
}
