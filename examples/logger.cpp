/**
 * Copyright (c) 2015 Parrot S.A.
 * All rights reserved.
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
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <sys/stat.h>

#include "futils/timetools.h"
#include "libtelemetry.hpp"

#define DEFAULT_ROOT_DIR  "/var/lib/tlm-logger"

/** */
static volatile bool running = true;

/** */
class MyLoggerCb : public telemetry::LoggerCb {
private:
	typedef std::map<std::string, FILE *> LogFileMap;
	typedef std::map<std::string, int>    IndexMap;

	telemetry::Logger  *mLogger;
	std::string        mRootDir;
	LogFileMap         mLogFileMap;
	IndexMap           mIndexMap;

	std::string getLogFilePath(const std::string &section);
	void openLogFile(const std::string &section);
	void closeLogFile(const std::string &section);

public:
	inline MyLoggerCb(telemetry::Logger *logger, const std::string &rootDir)
		: mLogger(logger), mRootDir(rootDir) {}
	virtual ~MyLoggerCb();

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
MyLoggerCb::~MyLoggerCb()
{
	while (mLogFileMap.size() != 0) {
		closeLogFile(mLogFileMap.begin()->first);
	}
}

/**
 */
std::string MyLoggerCb::getLogFilePath(const std::string &section)
{
	/* Append '/' to root directory if needed, then file name */
	std::string path = mRootDir;
	if (path.size() > 0 && path[path.size() - 1] != '/')
		path += "/";

	/* Append pid to simulate a dirty log rotation... */
	return path + section + ".log." + std::to_string(getpid());
}

/**
 */
void MyLoggerCb::openLogFile(const std::string &section)
{
	std::string path = getLogFilePath(section);
	fprintf(stdout, "Opening log file '%s'\n", path.c_str());
	FILE *file = fopen(path.c_str(), "wb");
	mLogFileMap.insert(LogFileMap::value_type(section, file));
	mIndexMap.insert(IndexMap::value_type(section, 0));
}

/**
 */
void MyLoggerCb::closeLogFile(const std::string &section)
{
	LogFileMap::iterator it = mLogFileMap.find(section);
	if (it != mLogFileMap.end()) {
		FILE *file = it->second;
		if (file != NULL) {
			std::string path = getLogFilePath(section);
			fprintf(stdout, "Closing log file '%s'\n", path.c_str());
			fclose(file);
		}
		mLogFileMap.erase(it);
	}

	IndexMap::iterator it2 = mIndexMap.find(section);
	if (it2 != mIndexMap.end())
		mIndexMap.erase(it2);
}

/**
 */
void MyLoggerCb::sectionAdded(uint32_t sectionId)
{
	const std::string &section = mLogger->getSection(sectionId);
	fprintf(stdout, "sectionAdded '%s'\n", section.c_str());
	/* Wait for sectionChanged to open log file and write header */
}

/**
 */
void MyLoggerCb::sectionRemoved(uint32_t sectionId)
{
	const std::string &section = mLogger->getSection(sectionId);
	fprintf(stdout, "sectionRemoved '%s'\n", section.c_str());
	closeLogFile(section);
}

/**
 */
void MyLoggerCb::sectionChanged(uint32_t sectionId, const void *buf, size_t len)
{
	const std::string &section = mLogger->getSection(sectionId);
	fprintf(stdout, "sectionChanged '%s'\n", section.c_str());

	/* Reopen log file */
	closeLogFile(section);
	openLogFile(section);
	FILE *file = mLogFileMap[section];
	if (file == NULL)
		return;

	/* Write header */
	fprintf(file, "-- Build infos\n");
	fprintf(file, "product:  BebopDrone\n");
	fprintf(file, "name:     bebop\n");
	fprintf(file, "version:  1.0\n");
	fprintf(file, "date:     01/01/1970\n");
	fprintf(file, "time:     00:00\n");
	fprintf(file, "compiler: gcc\n");
	fprintf(file, "-- Navdata infos\n");

	uint32_t varCount = mLogger->getVarCount(sectionId);

	/* Determine number of entries (expanding arrays), index and timeStamp
	 * are always added */
	uint32_t entryCount = 2;
	for (uint32_t i = 0; i < varCount; i++) {
		const telemetry::VarDesc &varDesc = mLogger->getVarDesc(sectionId, i);
		entryCount += varDesc.getCount();
	}
	fprintf(file, "nentries: %d\n", entryCount);
	fprintf(file, "datasize: 8\n");

	/* Names */
	fprintf(file, "titles: index, time_s");
	for (uint32_t i = 0; i < varCount; i++) {
		const telemetry::VarDesc &varDesc = mLogger->getVarDesc(sectionId, i);
		if (varDesc.getCount() == 1) {
			fprintf(file, ", %s", varDesc.getName().c_str());
		} else {
			for (uint32_t j = 0; j < varDesc.getCount(); j++) {
				fprintf(file, ", %s%d", varDesc.getName().c_str(), j);
			}
		}
	}
	fprintf(file, "\n");
	fprintf(file, "-- Data\n");
}

/**
 */
void MyLoggerCb::sampleBegin(uint32_t sectionId,
		const struct timespec *timestamp,
		const void *buf, size_t len)
{
	const std::string &section = mLogger->getSection(sectionId);
	FILE *file = mLogFileMap[section];
	if (file == NULL)
		return;
	uint64_t ts = 0;
	time_timespec_to_us(timestamp, &ts);
	double data[2] = {(double)mIndexMap[section]++, (double)ts / (1000 * 1000) };

	if (fwrite(data, sizeof(data), 1, file) != 1)
		fprintf(stderr, "fwrite: err=%d (%s)\n", errno, strerror(errno));
}

/**
 */
void MyLoggerCb::sampleEnd(uint32_t sectionId)
{
}

/**
 */
template<typename T>
static double convertToDouble(const void *buf, size_t len)
{
	assert(len >= sizeof(T));
	T val;
	memcpy(&val, buf, sizeof(T));
	return (double)val;
}

/**
 */
void MyLoggerCb::sample(uint32_t sectionId,
		const struct timespec *timestamp,
		uint32_t varId,
		const telemetry::VarDesc &varDesc,
		const void *buf, size_t len)
{
	const std::string &section = mLogger->getSection(sectionId);
	FILE *file = mLogFileMap[section];

	if (file == NULL)
		return;

	/* Convert each variable to a 'double'*/
	const uint8_t *ptr = (const uint8_t *)buf;
	size_t off = 0;
	for (uint32_t i = 0; i < varDesc.getCount(); i++) {
		double data = 0.0;
		switch (varDesc.getType()) {
		case TLM_TYPE_BOOL:
			data = convertToDouble<uint8_t>(ptr, len - off);
			break;

		case TLM_TYPE_UINT8:
			data = convertToDouble<uint8_t>(ptr, len - off);
			break;

		case TLM_TYPE_INT8:
			data = convertToDouble<int8_t>(ptr, len - off);
			break;

		case TLM_TYPE_UINT16:
			data = convertToDouble<uint16_t>(ptr, len - off);
			break;

		case TLM_TYPE_INT16:
			data = convertToDouble<int16_t>(ptr, len - off);
			break;

		case TLM_TYPE_UINT32:
			data = convertToDouble<uint32_t>(ptr, len - off);
			break;

		case TLM_TYPE_INT32:
			data = convertToDouble<int32_t>(ptr, len - off);
			break;

		case TLM_TYPE_UINT64:
			data = convertToDouble<uint64_t>(ptr, len - off);
			break;

		case TLM_TYPE_INT64:
			data = convertToDouble<int64_t>(ptr, len - off);
			break;

		case TLM_TYPE_FLOAT32:
			data = convertToDouble<float>(ptr, len - off);
			break;

		case TLM_TYPE_FLOAT64:
			data = convertToDouble<double>(ptr, len - off);
			break;

		case TLM_TYPE_STRING:
			/* Not supported in bbx format */
			data = 0.0;
			break;

		case TLM_TYPE_BINARY:
			/* Not supported in bbx format */
			data = 0.0;
			break;

		case TLM_TYPE_INVALID: /* NO BREAK */
		default:
			data = 0.0;
			break;
		}

		if (fwrite(&data, sizeof(double), 1, file) != 1) {
			fprintf(stderr, "fwrite: err=%d (%s)\n", errno,
				strerror(errno));
		}

		/* Advance buffer to next element in case of array */
		ptr += varDesc.getSize();
		off += varDesc.getSize();
	}
}

/**
 */
static void usage(const char *progname)
{
	fprintf(stderr, "usage: %s [<rootdir>]\n", progname);
	fprintf(stderr, "Continuously log the contents of all active shared\n");
	fprintf(stderr, "memory sections in log files in bbx format\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "<rootdir>: root directory where to save logs.\n");
	fprintf(stderr, "           default is '%s'\n", DEFAULT_ROOT_DIR);
	fprintf(stderr, "\n");
}

/**
 */
int main(int argc, char *argv[])
{
	signal(SIGINT, [](int signo){running = false;});
	signal(SIGTERM, [](int signo){running = false;});

	/* Check for help */
	if (argc >= 2 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
		usage(argv[0]);
		exit(EXIT_SUCCESS);
	}

	/* Determine root directory */
	std::string rootDir = DEFAULT_ROOT_DIR;
	if (argc >= 2)
		rootDir = argv[1];
	fprintf(stdout, "root directory is '%s'\n", rootDir.c_str());

	/* Create root directory (not recursive) */
	if (mkdir(rootDir.c_str(), 0755) < 0 && errno != EEXIST) {
		fprintf(stderr, "Failed to create root directory '%s': err=%d(%s)\n",
				rootDir.c_str(), errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	/* Create logger */
	telemetry::Logger *logger = telemetry::Logger::create();
	MyLoggerCb cb(logger, rootDir);

	/* Read samples every seconds */
	while (running) {
		logger->fetchNextSamples(&cb);
		usleep(1000 * 1000);
	}

	/* Free resources */
	telemetry::Logger::release(logger);
	return 0;
}
