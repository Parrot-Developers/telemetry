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
 * @file wrapper.cpp
 *
 */

#include "private.hpp"

using namespace telemetry;

/**
 */
static Producer *toProducer(struct tlm_producer *ptr)
{
	return (Producer *)ptr;
}

/**
 */
static struct tlm_producer *fromProducer(Producer *ptr)
{
	return (struct tlm_producer *)ptr;
}

/**
 */
static Consumer *toConsumer(struct tlm_consumer *ptr)
{
	return (Consumer *)ptr;
}

/**
 */
static struct tlm_consumer *fromConsumer(Consumer *ptr)
{
	return (struct tlm_consumer *)ptr;
}

/*
 * See documentation in public header.
 */
const char *tlm_get_var_type_str(enum tlm_var_type type)
{
	return getVarTypeStr(type);
}

/*
 * See documentation in public header.
 */
struct tlm_producer *tlm_producer_new(const char *name,
		uint32_t maxsamples,
		uint32_t rate)
{
	if (name == NULL)
		return NULL;
	return fromProducer(Producer::create(name, maxsamples, rate));
}

/*
 * See documentation in public header.
 */
int tlm_producer_destroy(struct tlm_producer *producer)
{
	if (producer == NULL)
		return -EINVAL;
	Producer::release(toProducer(producer));
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_producer_reg(struct tlm_producer *producer,
		void *ptr,
		const char *name,
		enum tlm_var_type type,
		size_t size,
		size_t count,
		uint32_t flags)
{
	if (producer == NULL || ptr == NULL || name == NULL)
		return -EINVAL;
	toProducer(producer)->regInternal(ptr,
			toProducer(producer)->getSection(),
			name, type, size, count, flags, NULL);
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_producer_reg_array(struct tlm_producer *producer,
		const struct tlm_producer_reg_entry *entries,
		size_t count)
{
	int res = 0;
	size_t i = 0;
	const struct tlm_producer_reg_entry *entry = NULL;
	if (producer == NULL || entries == NULL || count == 0)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		entry = &entries[i];
		res = tlm_producer_reg(producer, entry->ptr, entry->name,
				entry->type, entry->size, entry->count,
				entry->flags);
		if (res < 0)
			return res;
	}
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_producer_reg_complete(struct tlm_producer *producer)
{
	if (producer == NULL)
		return -EINVAL;
	toProducer(producer)->regComplete();
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_producer_reset(struct tlm_producer *producer)
{
	if (producer == NULL)
		return -EINVAL;
	toProducer(producer)->reset();
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_producer_put_sample(struct tlm_producer *producer,
		const struct timespec *timestamp)
{
	if (producer == NULL)
		return -EINVAL;
	toProducer(producer)->putSample(timestamp);
	return 0;
}

/*
 * See documentation in public header.
 */
struct tlm_consumer *tlm_consumer_new(void)
{
	return fromConsumer(Consumer::create());
}

/*
 * See documentation in public header.
 */
int tlm_consumer_destroy(struct tlm_consumer *consumer)
{
	if (consumer == NULL)
		return -EINVAL;
	Consumer::release(toConsumer(consumer));
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_consumer_reg(struct tlm_consumer *consumer,
		void *ptr,
		const char *name,
		enum tlm_var_type type,
		size_t size,
		size_t count,
		struct timespec *timestamp)
{
	if (consumer == NULL || ptr == NULL || name == NULL)
		return -EINVAL;
	toConsumer(consumer)->regInternal(ptr, "",
			name, type, size, count, 0, timestamp);
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_consumer_reg_array(struct tlm_consumer *consumer,
		const struct tlm_consumer_reg_entry *entries,
		size_t count)
{
	int res = 0;
	size_t i = 0;
	const struct tlm_consumer_reg_entry *entry = NULL;
	if (consumer == NULL || entries == NULL || count == 0)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		entry = &entries[i];
		res = tlm_consumer_reg(consumer, entry->ptr, entry->name,
				entry->type, entry->size, entry->count,
				entry->timestamp);
		if (res < 0)
			return res;
	}
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_consumer_reg_complete(struct tlm_consumer *consumer)
{
	if (consumer == NULL)
		return -EINVAL;
	toConsumer(consumer)->regComplete();
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_consumer_get_sample(struct tlm_consumer *consumer,
		const struct timespec *timestamp,
		enum tlm_method method)
{
	if (consumer == NULL)
		return -EINVAL;
	if (!toConsumer(consumer)->getSample(timestamp, method))
		return -ENOENT;
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_consumer_get_prev_sample(struct tlm_consumer *consumer,
		const struct timespec *timestamp)
{
	if (consumer == NULL)
		return -EINVAL;
	if (!toConsumer(consumer)->getPrevSample(timestamp))
		return -ENOENT;
	return 0;
}

/*
 * See documentation in public header.
 */
int tlm_consumer_get_next_sample(struct tlm_consumer *consumer,
		const struct timespec *timestamp,
		uint32_t timeout)
{
	if (consumer == NULL)
		return -EINVAL;
	if (!toConsumer(consumer)->getNextSample(timestamp, timeout))
		return -ENOENT;
	return 0;
}
