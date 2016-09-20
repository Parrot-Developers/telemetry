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
 * @file gndctrl_client.cpp
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

#include <map>

#define ULOG_TAG tlmgndctrl
#include "ulog.h"

#define POMP_ENABLE_ADVANCED_API
#include "libpomp.h"

#include "libtlmgndctrl.hpp"

ULOG_DECLARE_TAG(tlmgndctrl);

#define display(format, ...) \
{ \
	if(mDisplayToStdout) \
	fprintf(stdout, format,  ##__VA_ARGS__); \
	else \
	ULOGI(format,  ##__VA_ARGS__); \
} \


/**
 */
class GndCtrlApp : public telemetry::GndCtrlItfCb {
public:
	GndCtrlApp(const std::string &name, const std::string &ctrlAddr,
			uint32_t dataPort, uint32_t sampleRate,
			uint32_t msgRate, bool stdoutDisp);
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
	virtual void sampleEnd(uint32_t sectionId);
	virtual void sample(uint32_t sectionId,
			const struct timespec *timestamp,
			uint32_t varId,
			const telemetry::VarDesc &varDesc,
			const void *buf, size_t len);

	virtual void connected(const telemetry::GndCtrlItfCb::PropertyMap &properties);
	virtual void disconnected();

private:
	struct pomp_loop       *mLoop;
	telemetry::GndCtrlItf  *mGndCtrlItf;
	volatile bool          mQuit;
	bool                   mDisplayToStdout;
};

/**
 */
GndCtrlApp::GndCtrlApp(const std::string &name, const std::string &ctrlAddr,
		uint32_t dataPort, uint32_t sampleRate, uint32_t msgRate, bool stdoutDisp) :
	mQuit(false),
	mDisplayToStdout(stdoutDisp)

{
	mLoop = pomp_loop_new();
	mGndCtrlItf = telemetry::GndCtrlItf::create(this, mLoop, name, ctrlAddr, dataPort,
			sampleRate, msgRate);
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
}

/**
 */
void GndCtrlApp::sectionRemoved(uint32_t sectionId)
{
	display( "Removed: %s",
			mGndCtrlItf->getSection(sectionId).c_str());
}

/**
 */
void GndCtrlApp::sectionChanged(uint32_t sectionId, const void *buf, size_t len)
{
	display( "\e[33m" "section '%s':\n" "\e[0m",
			mGndCtrlItf->getSection(sectionId).c_str());
	for (unsigned int i=0; i < mGndCtrlItf->getVarCount(sectionId); i++)
	{
		const telemetry::VarDesc var = mGndCtrlItf->getVarDesc(sectionId, i);
		display( "   + %s\n", var.getName().c_str());
	}
	display( "\n");
}

/**
 */
void GndCtrlApp::sampleBegin(uint32_t sectionId,
		const struct timespec *timestamp,
		const void *buf, size_t len)
{
	display( "%s.timestamp: %ld.%06ld\n",
			mGndCtrlItf->getSection(sectionId).c_str(),
			timestamp->tv_sec, timestamp->tv_nsec / 1000);
}

/**
 */
void GndCtrlApp::sampleEnd(uint32_t sectionId)
{
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
		const struct timespec *timestamp,
		uint32_t varId,
		const telemetry::VarDesc &varDesc,
		const void *buf, size_t len)
{
	const uint8_t *ptr = (const uint8_t *)buf;
	size_t off = 0;
	for (uint32_t i = 0; i < varDesc.getCount(); i++) {
		switch (varDesc.getType()) {
		case TLM_TYPE_BOOL:
			display("%s: %u\n", varDesc.getFullName().c_str(), getVar<uint8_t>(ptr, len - off));
			break;

		case TLM_TYPE_UINT8:
			display("%s: %u\n", varDesc.getFullName().c_str(), getVar<uint8_t>(ptr, len - off));
			break;

		case TLM_TYPE_INT8:
			display("%s: %d\n", varDesc.getFullName().c_str(), getVar<int8_t>(ptr, len - off));
			break;

		case TLM_TYPE_UINT16:
			display("%s: %u\n", varDesc.getFullName().c_str(), getVar<uint16_t>(ptr, len - off));
			break;

		case TLM_TYPE_INT16:
			display("%s: %d\n", varDesc.getFullName().c_str(), getVar<int16_t>(ptr, len - off));
			break;

		case TLM_TYPE_UINT32:
			display("%s: %u\n", varDesc.getFullName().c_str(), getVar<uint32_t>(ptr, len - off));
			break;

		case TLM_TYPE_INT32:
			display("%s: %d\n", varDesc.getFullName().c_str(), getVar<uint32_t>(ptr, len - off));
			break;

		case TLM_TYPE_UINT64:
			display("%s: %" PRIu64 "\n", varDesc.getFullName().c_str(), getVar<uint64_t>(ptr, len - off));
			break;

		case TLM_TYPE_INT64:
			display("%s: %" PRId64 "\n", varDesc.getFullName().c_str(), getVar<int64_t>(ptr, len - off));
			break;

		case TLM_TYPE_FLOAT32:
			display("%s: %f\n", varDesc.getFullName().c_str(), getVar<float>(ptr, len - off));
			break;

		case TLM_TYPE_FLOAT64:
			display("%s: %lf\n", varDesc.getFullName().c_str(), getVar<double>(ptr, len - off));
			break;

		case TLM_TYPE_STRING:
			break;

		case TLM_TYPE_BINARY:
			break;

		case TLM_TYPE_INVALID: /* NO BREAK */
		default:
			break;
		}

		/* Advance buffer to next element in case of array */
		ptr += varDesc.getSize();
		off += varDesc.getSize();
	}
}

/**
 */
void GndCtrlApp::connected(const telemetry::GndCtrlItfCb::PropertyMap &properties)
{
	telemetry::GndCtrlItfCb::PropertyMap::const_iterator it;
	display("Connected\n");
	for (it = properties.begin(); it != properties.end(); ++it) {
		display("%s='%s'\n", it->first.c_str(), it->second.c_str());
	}
}

/**
 */
void GndCtrlApp::disconnected()
{
	display("Disconnected\n");
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
	fprintf(stderr, "Tool that prints data on standard output at a programmable pace.\n"
			"  This requires a connection to a telemetryd process running \n"
			"  the 'ground control' plugin.\n");
	fprintf(stderr, "usage: %s [<options>] <ctrladdr> [<dataport>]\n", progname);
	fprintf(stderr, "       <ctrladdr> format can be inet:xxx.xxx.xxx.xxx:9060\n");
	fprintf(stderr, "       <dataport> port on which the data will be sent\n");
	fprintf(stderr, "Options:\n");
	fprintf(stderr, "   -s <integer>     sample rate in microseconds\n");
	fprintf(stderr, "   -m <integer>     period of data bursts, in microseconds\n");
	fprintf(stderr, "   -r <integer>     combines -s and -m, in *milliseconds*\n");
	fprintf(stderr, "   -u               sends out logs to ulog, instead of stdout\n");
	fprintf(stderr, "   -h               this help page\n");
	fprintf(stderr, "\n");
}

/**
 */
int main(int argc, char *argv[])
{
	int argidx = 0;
	int status = EXIT_SUCCESS;
	std::string ctrlAddr;
	int dataPort = 0;
	int sampleRate = 1000 * 1000;
	int msgRate = 1000 * 1000;
	bool stdoutDisp = true;

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
			/* Sample rate in microsecs */
			sampleRate = atoi(argv[++argidx]);
		} else if (strcmp(argv[argidx], "-m") == 0
				&& argidx + 1 < argc) {
			/* Message rate in microsecs */
			msgRate = atoi(argv[++argidx]);
		} else if (strcmp(argv[argidx], "-r") == 0
				&& argidx + 1 < argc) {
			/* Global rate in milliseconds */
			msgRate = atoi(argv[++argidx]) * 1000;
			sampleRate = msgRate;
		} else if (strcmp(argv[argidx], "-u") == 0
				&& argidx + 1 < argc) {
			/* display towards ulog */
			stdoutDisp = false;
		} else {
			fprintf(stderr, "Unknown option: '%s'\n", argv[argidx]);
			usage(argv[0]);
			goto error;
		}
	}

	/* Get control address */
	if (argc - argidx >= 1) {
		ctrlAddr = argv[argidx++];
	} else {
		fprintf(stderr, "Missing address\n");
		usage(argv[0]);
		goto error;
	}

	/* Get data port */
	if (argc - argidx >= 1) {
		dataPort = atoi(argv[argidx++]);
	} else {
		fprintf(stderr, "No data port specified, tell server"
				"to use TCP protocol to send samples.\n");
	}

	/* Create application object, setup signal handler */
	sApp = new GndCtrlApp("gndctrl-client", ctrlAddr, dataPort, sampleRate, msgRate, stdoutDisp);
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
