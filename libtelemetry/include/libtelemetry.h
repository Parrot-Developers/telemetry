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
 *
 * @file libtelemetry.h
 *
 */

#ifndef _LIBTELEMETRY_H_
#define _LIBTELEMETRY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C"  {
#endif /* __cplusplus */

/** Type of variables */
enum tlm_var_type {
	TLM_TYPE_INVALID = -1,
	TLM_TYPE_BOOL = 0,
	TLM_TYPE_UINT8,
	TLM_TYPE_INT8,
	TLM_TYPE_UINT16,
	TLM_TYPE_INT16,
	TLM_TYPE_UINT32,
	TLM_TYPE_INT32,
	TLM_TYPE_UINT64,
	TLM_TYPE_INT64,
	TLM_TYPE_FLOAT32,
	TLM_TYPE_FLOAT64,
	TLM_TYPE_STRING,
	TLM_TYPE_BINARY
};

/** Flags for variable registrations */
enum tlm_flag {
	TLM_FLAG_DEFAULT = 0,     /**< Default: logged */
	TLM_FLAG_INHERIT = 0,     /**< For class fields: inherit from parent*/
	TLM_FLAG_NO_LOG = 1 << 2  /**< Do not log this variable */
};

/** Query method */
enum tlm_method {
	TLM_LATEST,       /**< Get last sample (timestamp of query unsed) */
	TLM_CLOSEST,      /**< Get closest sample */
	TLM_FIRST_AFTER,  /**< Get closest sample AFTER given timestamp) */
	TLM_FIRST_BEFORE  /**< Get closest sample BEFORE given timestamp) */
};

/* Forward declarations */
struct timespec;
struct tlm_producer;
struct tlm_consumer;

/** Producer registration entry */
struct tlm_producer_reg_entry {
	void               *ptr;
	const char         *name;
	enum tlm_var_type  type;
	size_t             size;
	size_t             count;
	uint32_t           flags;
};

/** Consumer registration entry */
struct tlm_consumer_reg_entry {
	void               *ptr;
	const char         *name;
	enum tlm_var_type  type;
	size_t             size;
	size_t             count;
	struct timespec    *timestamp;
};

/**
 * Concert a variable type to a human readable string.
 * @param type: type to convert.
 * @return string with type.
 */
const char *tlm_get_var_type_str(enum tlm_var_type type);

/**
 * Create a new producer.
 * @param section: name of the producer (section).
 * @param maxsamples: Maximum number of samples.
 * @param rate: Approximative rate of samples (in us).
 * @return created producer or NULL in case of error.
 */
struct tlm_producer *tlm_producer_new(
		const char *section,
		uint32_t maxsamples,
		uint32_t rate);

/**
 * Destroy a producer.
 * @param producer: producer to destroy.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_producer_destroy(struct tlm_producer *producer);

/**
 * Register a new variable.
 * @param producer: producer object.
 * @param ptr: pointer to variable.
 * @param name: name of variable.
 * @param type: type of variable.
 * @param size: size of variable.
 * @param count: number of variables for arrays.
 * @param flags: flags for the variable.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_producer_reg(struct tlm_producer *producer,
		void *ptr,
		const char *name,
		enum tlm_var_type type,
		size_t size,
		size_t count,
		uint32_t flags);

/**
 * Register an array of entries.
 * @param producer:  producer object.
 * @param entries: array of entries.
 * @param count: number of entries.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_producer_reg_array(struct tlm_producer *producer,
		const struct tlm_producer_reg_entry *entries,
		size_t count);

/**
 * Complete registration of variables.
 * @param producer: producer object.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_producer_reg_complete(struct tlm_producer *producer);

/**
 * Force recreation of shared memory sections.
 * @param producer: producer object.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_producer_reset(struct tlm_producer *producer);

/**
 * Put a new sample.
 * @param producer: producer object.
 * @param timestamp: timestamp of new sample or NULL to use current time.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_producer_put_sample(struct tlm_producer *producer,
		const struct timespec *timestamp);

/**
 * Create a new consumer.
 * @return created consumer or NULL in case of error.
 */
struct tlm_consumer *tlm_consumer_new(void);

/**
 * Destroy a consumer.
 * @param consumer: consumer to destroy.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_consumer_destroy(struct tlm_consumer *consumer);

/**
 * Register a new variable.
 * @param consumer: consumer object.
 * @param ptr: pointer to variable.
 * @param name: name of variable.
 * @param type: type of variable.
 * @param size: size of variable.
 * @param count: number of variables for arrays.
 * @param timestamp: structure where to store timestamp of samples.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_consumer_reg(struct tlm_consumer *consumer,
		void *ptr,
		const char *name,
		enum tlm_var_type type,
		size_t size,
		size_t count,
		struct timespec *timestamp);

/**
 * Register an array of entries.
 * @param consumer:  consumer object.
 * @param entries: array of entries.
 * @param count: number of entries.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_consumer_reg_array(struct tlm_consumer *consumer,
		const struct tlm_consumer_reg_entry *entries,
		size_t count);

/**
 * Complete the registration of variables.
 * @param consumer: consumer object.
 * @return 0 in case of success, negative errno value in case of error.
 */
int tlm_consumer_reg_complete(struct tlm_consumer *consumer);

/**
 * Get a sample.
 * @param consumer: consumer object.
 * @param timestamp: timestamp of query (can be NULL for LATEST method).
 * @param method: method of query.
 * @return 0 if at least one sample has been read in one of the shared memory
 * where variables are, negative errno value in case of error.
 *
 * @remarks -ENOENT is returned if no samples where retrieved.
 *
 */
int tlm_consumer_get_sample(struct tlm_consumer *consumer,
		const struct timespec *timestamp,
		enum tlm_method method);

/**
 * Get sample before the given timestamp.
 * @param consumer: consumer object.
 * @param timestamp: timestamp read from a previous call.
 * @return 0 if a sample was found.
 *
 * @remarks -ENOENT is returned if no samples where retrieved.
 * @remarks: this will only work if all variables are part of the same
 * section.
 */
int tlm_consumer_get_prev_sample(struct tlm_consumer *consumer,
		const struct timespec *timestamp);

/**
 * Get sample after the given timestamp.
 * @param consumer: consumer object.
 * @param timestamp: timestamp read from a previous call.
 * @param timeout: max time to wait for the sample (in ms), 0 for no wait.
 * @return 0 if a sample was found.
 *
 * @remarks -ENOENT is returned if no samples where retrieved.
 * @remarks: this will only work if all variables are part of the same
 * section.
 */
int tlm_consumer_get_next_sample(struct tlm_consumer *consumer,
		const struct timespec *timestamp,
		uint32_t timeout);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_LIBTELEMETRY_H_ */
