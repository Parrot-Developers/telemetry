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
 */

#include <signal.h>
#include <unistd.h>
#include "libtelemetry.hpp"

/**
 */
class ClassWithSerialization {
private:
	float f;
	int tab[2];
public:
	inline ClassWithSerialization() {
		f = 42.42f;
		tab[0] = -1;
		tab[1] = -2;
	}

	inline void telemetryRegister(telemetry::ClassDesc &desc) {
		desc.reg(f, "my_float");
		desc.reg(tab, "my_tab");
	}
};

static void compilation_check()
{
	int8_t i8 = -33;
	int8_t i8_3[3] = {-12, 14, 56};
	int i = 67;
	int i_3[3] = {678, 890, 555};
	int i_3_4[3][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}};
	uint8_t u8 = 255;
	uint8_t u8_4[4] = {255, 254, 253, 252};
	unsigned char uc8 = 2;
	unsigned char uc8_4[4] = {100, 101, 102, 103};
	char c = 'H';
	char c_6[6] = {'H', 'j', 'L', 'L', 'O', '\0'};
	ClassWithSerialization cls;
	ClassWithSerialization cls_2[2];

	int *ptr = new int[5];

	telemetry::Producer *producer = telemetry::Producer::create("check", 100, 0);
	producer->reg(i8, "i8");
	producer->reg(i8_3, "i8_3");
	producer->reg(i, "i");
	producer->reg(i_3, "i_3");
	producer->reg(i_3_4, "i_3_4");
	producer->reg(u8, "u8");
	producer->reg(u8_4, "u8_4");
	producer->reg(uc8, "uc8");
	producer->reg(uc8_4, "uc8_4");
	producer->reg(c, "c");
	producer->reg(c_6, "c_6");
	producer->reg(cls, "cls");
	producer->reg(cls_2, "cls_2");
	producer->reg(ptr, "ptr");

	telemetry::Producer::release(producer);
	delete[] ptr;
}

static volatile bool running = true;

/**
 */
template<typename T>
static void doTest(uint32_t rate)
{
	T counter = 0;
	telemetry::Producer *producer = telemetry::Producer::create("main", 100, rate);
	producer->reg(counter, "counter");
	producer->regComplete();
	while (running) {
		usleep(rate);
		counter++;
		producer->putSample(NULL);
	}
	telemetry::Producer::release(producer);
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
	} colibryData = {
		{0.0, 0.1, 0.2, 0.3, 0.4},
		{-0.0, -0.1, -0.2, -0.3, -0.4},
		{1.0, 1.1, 1.2, 1.3, 1.4},

		{2.0, 2.1, 2.2, 2.3, 2.4},
		{-2.0, -2.1, -2.2, -2.3, -2.4},
		{5.0, 5.1, 5.2, 5.3, 5.4},

		{1, 2, 3, 4, 5},
	};

	telemetry::Producer *producer = telemetry::Producer::create("colibry", 100, rate);
	producer->reg(colibryData.theta, "theta");
	producer->reg(colibryData.phi, "phi");
	producer->reg(colibryData.psi, "psi");
	producer->reg(colibryData.rawGyroX, "rawGyroX");
	producer->reg(colibryData.rawGyroY, "rawGyroY");
	producer->reg(colibryData.rawGyroZ, "rawGyroZ");
	producer->reg(colibryData.Vsync, "Vsync");
	producer->regComplete();
	while (running) {
		usleep(rate);
		for (int i= 0; i < 5; i++) {
			colibryData.theta[i] += 0.5;
			colibryData.phi[i] -= 0.5;
			colibryData.psi[i] += 1.5;
			colibryData.rawGyroX[i] -= 0.5;
			colibryData.rawGyroY[i] += 1.5;
			colibryData.rawGyroZ[i] -= 1.5;
			colibryData.Vsync[i] += 5;
		}
		producer->putSample(NULL);
	}
	telemetry::Producer::release(producer);
}

/**
 */
int main(int argc, char *argv[])
{
	signal(SIGINT, [](int signo){running = false;});
	signal(SIGTERM, [](int signo){running = false;});

	compilation_check();

	if (argc < 2) {
		fprintf(stderr, "%s <rate(us)> [<type>]\n", argv[0]);
		exit(1);
	}

	uint32_t rate = atoi(argv[1]);
	if (rate == 0)
		rate = 500 * 1000;

	if (argc >= 3) {
		if (strcmp(argv[2], "i8") == 0) {
			doTest<uint8_t>(rate);
		} else if (strcmp(argv[2], "u8") == 0) {
			doTest<int8_t>(rate);
		} else if (strcmp(argv[2], "i16") == 0) {
			doTest<int16_t>(rate);
		} else if (strcmp(argv[2], "u16") == 0) {
			doTest<uint16_t>(rate);
		} else if (strcmp(argv[2], "i32") == 0) {
			doTest<int32_t>(rate);
		} else if (strcmp(argv[2], "u32") == 0) {
			doTest<uint32_t>(rate);
		} else if (strcmp(argv[2], "i64") == 0) {
			doTest<int64_t>(rate);
		} else if (strcmp(argv[2], "u64") == 0) {
			doTest<uint64_t>(rate);
		} else {
			doTest<uint32_t>(rate);
		}
	} else {
		doTestColibry(rate);
	}

	return 0;
}
