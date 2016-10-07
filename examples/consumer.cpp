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

#include <signal.h>
#include <unistd.h>
#include "libtelemetry.hpp"

static volatile bool running = true;

/**
 */
static void doTest(uint32_t rate)
{
	uint32_t counter = 0;
	telemetry::Consumer *consumer = telemetry::Consumer::create();
	consumer->reg(counter, "main.counter");
	consumer->regComplete();
	while (running) {
		usleep(rate);
		consumer->getSample(NULL, TLM_LATEST);
		fprintf(stdout, "counter=%u\n", counter);
	}
	telemetry::Consumer::release(consumer);
}

/**
 */
static void printDoubleArray(const char *name, const double *values, uint32_t count)
{
	fprintf(stdout, "%s", name);
	for (uint32_t i = 0; i < count; i++) {
		fprintf(stdout, " %f", values[i]);
	}
	fprintf(stdout, "\n");
}

/**
 */
static void doTestColibry(uint32_t rate)
{
	struct ColibryData {
		double theta[5];
		double phi[5];
		double psi[5];
		double rawGyroX[5];
		double rawGyroY[5];
		double rawGyroZ[5];
		uint8_t Vsync[5];
	} colibryData;
	memset(&colibryData, 0, sizeof(colibryData));

	telemetry::Consumer *consumer = telemetry::Consumer::create();
	consumer->reg(colibryData.theta, "colibry.theta");
	consumer->reg(colibryData.phi, "colibry.phi");
	consumer->reg(colibryData.psi, "colibry.psi");
	consumer->reg(colibryData.rawGyroX, "colibry.rawGyroX");
	consumer->reg(colibryData.rawGyroY, "colibry.rawGyroY");
	consumer->reg(colibryData.rawGyroZ, "colibry.rawGyroZ");
	consumer->reg(colibryData.Vsync, "colibry.Vsync");
	consumer->regComplete();
	while (running) {
		usleep(rate);
		consumer->getSample(NULL, TLM_LATEST);
		printDoubleArray("theta", colibryData.theta, 5);
		printDoubleArray("phi", colibryData.phi, 5);
		printDoubleArray("psi", colibryData.psi, 5);
		printDoubleArray("rawGyroX", colibryData.rawGyroX, 5);
		printDoubleArray("rawGyroY", colibryData.rawGyroY, 5);
		printDoubleArray("rawGyroZ", colibryData.rawGyroZ, 5);
		fprintf(stdout, "Vsync %d %d %d %d %d\n",
				colibryData.Vsync[0],
				colibryData.Vsync[1],
				colibryData.Vsync[2],
				colibryData.Vsync[3],
				colibryData.Vsync[4]);
	}
	telemetry::Consumer::release(consumer);
}

/**
 */
int main(int argc, char *argv[])
{
	signal(SIGINT, [](int signo){running = false;});
	signal(SIGTERM, [](int signo){running = false;});

	if (argc < 2) {
		fprintf(stderr, "%s <rate(us)> [<type>]\n", argv[0]);
		exit(1);
	}

	uint32_t rate = atoi(argv[1]);
	if (rate == 0)
		rate = 500 * 1000;

	if (argc >= 3 && strcmp(argv[2], "counter") == 0) {
		doTest(rate);
	} else {
		doTestColibry(rate);
	}
	return 0;
}
