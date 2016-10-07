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

#include "libtlmblackbox.hpp"

/**
 */
class TlmbReaderApp : public telemetry::BlackboxReaderItfCb {
private:
	telemetry::BlackboxReaderItf  *mItf;

public:
	TlmbReaderApp();
	virtual ~TlmbReaderApp();

	void run(const char *path);

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

	virtual void fileHeader(const PropertyMap &propertyMap);
};

/**
 */
TlmbReaderApp::TlmbReaderApp()
{
	mItf = NULL;
}

/**
 */
TlmbReaderApp::~TlmbReaderApp()
{
}

/**
 */
void TlmbReaderApp::run(const char *path)
{
	int res = 0;
	mItf = telemetry::BlackboxReaderItf::create(this);
	res = mItf->open(path);
	if (res < 0) {
		fprintf(stderr, "Unable to open '%s': err=%d(%s)\n",
				path, res, strerror(-res));
	} else {
		res = mItf->readAll();
		if (res < 0) {
			fprintf(stderr, "readAll: err=%d(%s)\n",
					res, strerror(-res));
		}
		mItf->close();
	}
	delete mItf;
	mItf = NULL;
}

/**
 */
void TlmbReaderApp::sectionAdded(uint32_t sectionId)
{
	const std::string &section = mItf->getSection(sectionId);
	fprintf(stdout, "sectionAdded '%s'\n", section.c_str());
}

/**
 */
void TlmbReaderApp::sectionRemoved(uint32_t sectionId)
{
	const std::string &section = mItf->getSection(sectionId);
	fprintf(stdout, "sectionRemoved '%s'\n", section.c_str());
}

/**
 */
void TlmbReaderApp::sectionChanged(uint32_t sectionId,
		const void *buf, size_t len)
{
	const std::string &section = mItf->getSection(sectionId);
	fprintf(stdout, "sectionChanged '%s'\n", section.c_str());
	fprintf(stdout, "  varCount: %u\n", mItf->getVarCount(sectionId));
	for (uint32_t i = 0; i < mItf->getVarCount(sectionId); i++) {
		fprintf(stdout, "  var: '%s'\n",
				mItf->getVarDesc(sectionId, i).getName().c_str());
	}
}

/**
 */
void TlmbReaderApp::sampleBegin(uint32_t sectionId,
		const struct timespec *timestamp,
		const void *buf, size_t len)
{
}

/**
 */
void TlmbReaderApp::sampleEnd(uint32_t sectionId)
{
}

/**
 */
void TlmbReaderApp::sample(uint32_t sectionId,
		const struct timespec *timestamp,
		uint32_t varId,
		const telemetry::VarDesc &varDesc,
		const void *buf, size_t len)
{
}

/**
 */
void TlmbReaderApp::fileHeader(const PropertyMap &propertyMap)
{
	fprintf(stdout, "fileHeader\n");
	for (PropertyMap::const_iterator it = propertyMap.begin();
			it != propertyMap.end();
			++it) {
		fprintf(stdout, "  %s=%s\n",
				it->first.c_str(),
				it->second.c_str());
	}
}

/**
 */
static void usage(const char *progname)
{
	fprintf(stderr, "usage: %s <tlmbfile>\n", progname);
	fprintf(stderr, "Dump contents of tlmb file.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "<tlmbfile>: Input file.\n");
	fprintf(stderr, "\n");
}

/**
 */
int main(int argc, char *argv[])
{
	if (argc != 2) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	/* Check for help */
	if (argc >= 2 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
		usage(argv[0]);
		exit(EXIT_SUCCESS);
	}

	TlmbReaderApp app;
	app.run(argv[1]);
	return 0;
}
