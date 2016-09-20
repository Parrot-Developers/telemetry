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
 * @file tlm_shm_generator.c
 *
 * @brief Tool to generate some content in 'estimator' shared memory.
 *
 * It does not depends on libshdata or libtelemetry directly. It uses
 * directly knowledge of internal data structures of both libraries. This
 * makes it independent in term of source code but tightly coupled in term
 * of data format.
 */

#include "libtelemetry.h"
#include <futils/futils.h>
#include  <signal.h>

#define DIAG_PFX "TLMSHMGEN: "

#define diag(_fmt, ...) \
	fprintf(stderr, DIAG_PFX _fmt "\n", ##__VA_ARGS__)

/* Duration between 2 simulation ticks : 5 ms in us */
#define TICK_DURATION 5000

/* Microseconds count in a second */
#define US_IN_SECOND 1000000

/* Gyro fifo depth, shall be in sync with hal structure */
#define GYRO_FIFO_DEPTH 5

/* Quaternion size array */
#define QUATERNION_SIZE 4

/**
 * Structure that contains data read using specific functions of Colibry
 * The Fsync flag from the IMU is exported as well, but read directly in
 * sensors' structure.
 */
struct exported_data {
	struct {
		float phi[GYRO_FIFO_DEPTH];
		float theta[GYRO_FIFO_DEPTH];
		float psi[GYRO_FIFO_DEPTH];
	} unbiased_gyro;
	struct {
		float phi;
		float theta;
		float psi;
	} flight_angles;
	struct {
		float x;
		float y;
		float z;
	} sum_gyro;
	float altitude;
	uint32_t nb_hal_tick;
	uint8_t fsync_flag[GYRO_FIFO_DEPTH];
	float flight_quaternion[QUATERNION_SIZE];
};

/**
 * Simulation data representing current virtual autopilot state
 */
struct simulation_data {
	uint32_t tick_duration;
	struct timespec ts;
	struct timespec next_ts;
	uint32_t nb_tick;
};

static int keepRunning = 1;

/**
 * SIGINT Handler
 *
 * Stop sample generation
 */
static void intHandler(int signo)
{
	keepRunning = 0;
}

/**
 * Bind exported data structure to a new producer.
 */
static struct tlm_producer *setup_exported_data(struct exported_data *ex_data)
{
	int ret;
	struct tlm_producer *tlm_prod;

	const struct tlm_producer_reg_entry entries[] = {
		{
			.ptr = ex_data->unbiased_gyro.phi,
			.name = "IMU_body_unbiased_gyro_phi",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = GYRO_FIFO_DEPTH,
			.flags = 0,
		}, {
			.ptr = ex_data->unbiased_gyro.theta,
			.name = "IMU_body_unbiased_gyro_theta",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = GYRO_FIFO_DEPTH,
			.flags = 0,
		}, {
			.ptr = ex_data->unbiased_gyro.psi,
			.name = "IMU_body_unbiased_gyro_psi",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = GYRO_FIFO_DEPTH,
			.flags = 0,
		}, {
			.ptr = &ex_data->flight_angles.phi,
			.name = "IMU_body_flight_angles_phi",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = &ex_data->flight_angles.theta,
			.name = "IMU_body_flight_angles_theta",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = &ex_data->flight_angles.psi,
			.name = "IMU_body_flight_angles_psi",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = ex_data->fsync_flag,
			.name = "IMU_body_fsync_flag",
			.type = TLM_TYPE_UINT8,
			.size = sizeof(uint8_t),
			.count = GYRO_FIFO_DEPTH,
			.flags = 0,
		}, {
			.ptr = &ex_data->sum_gyro.x,
			.name = "IMU_body_flight_sum_gyro_x",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = &ex_data->sum_gyro.y,
			.name = "IMU_body_flight_sum_gyro_y",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = &ex_data->sum_gyro.z,
			.name = "IMU_body_flight_sum_gyro_z",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = &ex_data->altitude,
			.name = "IMU_body_flight_altitude",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = &ex_data->nb_hal_tick,
			.name = "IMU_body_flight_nb_hal_tick",
			.type = TLM_TYPE_UINT32,
			.size = sizeof(uint32_t),
			.count = 1,
			.flags = 0,
		}, {
			.ptr = ex_data->flight_quaternion,
			.name = "IMU_body_flight_quaternion",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = QUATERNION_SIZE,
			.flags = 0,
		}
	};

	tlm_prod = tlm_producer_new("estimators", 50, TICK_DURATION);
	if (tlm_prod == NULL) {
		diag("tlm_producer_new failed");
		goto error;
	}

	ret = tlm_producer_reg_array(tlm_prod, entries, SIZEOF_ARRAY(entries));
	if (ret < 0) {
		diag("tlm_producer_reg_array error: %s", strerror(-ret));
		goto error;
	}

	ret = tlm_producer_reg_complete(tlm_prod);
	if (ret < 0) {
		diag("tlm_producer_reg_complete error: %s", strerror(-ret));
		goto error;
	}

	return tlm_prod;

error:
	if (tlm_prod)
		tlm_producer_destroy(tlm_prod);
	return NULL;
}

/**
 * Simulation initial state
 */
static void simulation_init(struct simulation_data *sim_data,
		const struct timespec *now_ts)
{
	sim_data->nb_tick = 0;
	sim_data->tick_duration = TICK_DURATION;
	sim_data->next_ts = *now_ts;
}

/**
 * Perform simulation step if the current timestamp is after the next timestamp
 *
 * The method returns true if a step had been performed. It also fill the
 * duration_to_next_step timestamp to the duration from now to the next step.
 */
static int simulation_step(struct simulation_data *sim_data,
		const struct timespec *now_ts,
		struct timespec *duration_to_next_step)
{
	int has_step = 0;

	if (time_timespec_cmp(&sim_data->next_ts, now_ts) <= 0) {
		/* Do step */
		sim_data->nb_tick++;
		sim_data->ts = sim_data->next_ts;
		time_timespec_add_us(&sim_data->next_ts,
				sim_data->tick_duration,
				&sim_data->next_ts);
		has_step = 1;
	}

	/* The duration_to_next_step must contain the duration to the next step.
	 * If the next step timestamp is in the past the duration must be zero,
	 * else the duration must be the difference between the current
	 * timestamp and the next step timestamp */
	if (time_timespec_cmp(&sim_data->next_ts, now_ts) > 0) {
		time_timespec_diff(now_ts, &sim_data->next_ts,
				duration_to_next_step);
	} else {
		duration_to_next_step->tv_sec = 0;
		duration_to_next_step->tv_nsec = 0;
	}

	return has_step;
}

/**
 * Put a new sample in the shared memory from the simulation data
 */
static void generate_sample(struct tlm_producer *tlm_prod,
		struct simulation_data *sim_data,
		struct exported_data *ex_data)
{
	int i;

	for (i = 0; i < GYRO_FIFO_DEPTH; i++) {
		ex_data->unbiased_gyro.phi[i] = 0.0f;
		ex_data->unbiased_gyro.theta[i] = 0.0f;
		ex_data->unbiased_gyro.psi[i] = 0.0f;
		ex_data->fsync_flag[i] = 0.0;
	}

	ex_data->flight_angles.phi = 0.0f;
	ex_data->flight_angles.theta = 0.0f;
	ex_data->flight_angles.psi = 0.0f;

	ex_data->sum_gyro.x = 0.0f;
	ex_data->sum_gyro.y = 0.0f;
	ex_data->sum_gyro.z = 0.0f;

	ex_data->flight_quaternion[0] = 0.0f;
	ex_data->flight_quaternion[1] = 0.0f;
	ex_data->flight_quaternion[2] = 0.0f;
	ex_data->flight_quaternion[3] = 1.0f;

	ex_data->altitude = 1;

	ex_data->nb_hal_tick = sim_data->nb_tick;

	tlm_producer_put_sample(tlm_prod, &sim_data->ts);
}

/**
 * Generator main loop
 *
 * Run until a SIGINT is received
 */
static int run_generator()
{
	struct tlm_producer *tlm_prod;
	struct exported_data ex_data;
	struct simulation_data sim_data;
	struct timespec now_ts;
	struct timespec time_to_step;

	tlm_prod = setup_exported_data(&ex_data);
	if (tlm_prod == NULL) {
		diag("setup_exported_data failed");
		return -1;
	}

	clock_gettime(CLOCK_MONOTONIC, &now_ts);

	simulation_init(&sim_data, &now_ts);

	diag("Start sample generation at %d Hz ...",
			US_IN_SECOND / sim_data.tick_duration);

	while (keepRunning) {
		/* Try to step*/
		int has_step = simulation_step(&sim_data, &now_ts,
				&time_to_step);

		if (has_step)
			generate_sample(tlm_prod, &sim_data, &ex_data);

		/* Wait for the next step if needed */
		if (time_to_step.tv_sec > 0 || time_to_step.tv_nsec > 0) {
			nanosleep(&time_to_step, NULL);
			clock_gettime(CLOCK_MONOTONIC, &now_ts);
		}
	}
	diag("Stop sample generation");

	tlm_producer_destroy(tlm_prod);

	return 0;
}


/**
 * Print usage
 */
static void usage(const char *progname)
{
	fprintf(stderr, "usage: %s [<options>]\n", progname);
	fprintf(stderr, "Generate stationary flight data to 'estimator'\n");
	fprintf(stderr, "shared memory\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Global options:\n");
	fprintf(stderr, "  -h, --help        this help\n");
	fprintf(stderr, "\n");
}

/**
 * Main
 *
 * Parse command line and show the usage or start generator
 */
int main(int argc, char *argv[])
{
	int res = 0;
	int argidx = 0;

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
		} else {
			diag("Unknown option: '%s'", argv[argidx]);
			goto error;
		}
	}

	signal(SIGINT, intHandler);

	/* Start sample generation */
	if (run_generator() < 0)
		goto error;

	goto out;

error:
	res = -1;
out:
	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

