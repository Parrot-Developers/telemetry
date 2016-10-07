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
 * @file libtelemetry.hpp
 *
 */

#ifndef _LIBTELEMETRY_HPP_
#define _LIBTELEMETRY_HPP_

#include "libtelemetry.h"

#include <map>
#include <string>
#include <vector>

/** Forward declarations */
struct shd_ctx;
struct timespec;
namespace telemetry {
	typedef enum tlm_var_type  VarType;
	typedef enum tlm_method    Method;
	class VarDesc;
	class ClassDesc;
	class Producer;
	class Consumer;
	class Logger;
	class LoggerCb;
	namespace internal {
		class VarInfo;
		class Registrator;
		class ShdCtx;
	}
}

#include "libtelemetry-internal.hpp"

namespace telemetry {

/** Flags for variable registrations */
namespace Flags {
	const uint32_t DEFAULT = TLM_FLAG_DEFAULT;  /**< Default: logged */
	const uint32_t INHERIT = TLM_FLAG_INHERIT;  /**< For class fields: inherit from parent */
	const uint32_t NO_LOG = TLM_FLAG_NO_LOG;    /**< Do not log this variable */
}

/**
 */
const char *getVarTypeStr(VarType type);

/**
 * Describe a registered variable.
 */
class VarDesc {
private:
	std::string  mSection;  /**< Section where this variable is */
	std::string  mName;     /**< Name of variable */
	std::string  mFullName; /**< Full name (section + name) */
	VarType      mType;     /**< Type of variable */
	size_t       mSize;     /**< Size of type */
	size_t       mCount;    /**< Number of elements */
	uint32_t     mFlags;    /**< Flags */

public:
	/* Internal methods to manage record in shared memory */
	size_t getRecordLength() const;
	int writeRecord(void *dst, size_t maxdst) const;
	int readRecord(const void *src, size_t maxsrc);
	static int readRecordArray(std::vector<VarDesc> &varDescVector,
			const std::string &section,
			const void *src, size_t maxsrc);

	/**
	 * Constructor for producer
	 * @param section: name of section.
	 * @param name: name of the variable.
	 * @param type: type of the variable.
	 * @param size: size of the variable.
	 * @param count: number of elements for arrays.
	 * @param flags: flags for the variable.
	 */
	inline VarDesc(
			const std::string &section,
			const std::string &name,
			VarType type,
			size_t size,
			size_t count,
			uint32_t flags)
			:
			mSection(section),
			mName(name),
			mType(type),
			mSize(size),
			mCount(count),
			mFlags(flags) {
		mFullName = mSection + "." + mName;
	}

	/**
	 * Constructor for consumer.
	 * @param section: name of the section.
	 */
	inline VarDesc(const std::string &section)
			:
			mSection(section),
			mName(""),
			mType(TLM_TYPE_INVALID),
			mSize(0),
			mCount(0),
			mFlags(0) {
	}

	inline const std::string &getSection() const {
		return mSection;
	}

	inline const std::string &getName() const {
		return mName;
	}

	inline const std::string &getFullName() const {
		return mFullName;
	}

	inline VarType getType() const {
		return mType;
	}

	inline size_t getSize() const {
		return mSize;
	}

	inline size_t getCount() const {
		return mCount;
	}

	inline uint32_t getFlags() const {
		return mFlags;
	}

	inline size_t getTotalSize() const {
		return mSize * mCount;
	}

	inline bool isArray() const {
		return mCount >= 2;
	}
};

/**
 * Used for registration of variables inside a class.
 */
class ClassDesc {
private:
	std::string  mSection;     /**< Section of parent class */
	std::string  mName;        /**< Name of parent class */
	uint32_t     mFlags;       /**< Flags of parent class */
	struct timespec  *mTimestamp;  /**< Timestamp location of parent class */
	internal::Registrator  &mRegistrator;/**< Associated registrator */

public:
	/**
	 * Constructor.
	 * @param section: name of the section.
	 * @param name: name of the parent variable.
	 * @param flags: flags for the parent variable.
	 * @param timestamp: timestamp location of parent class (consumer only).
	 * @param registrator: object that will be responsible for registration.
	 */
	inline ClassDesc(
			const std::string &section,
			const std::string &name,
			uint32_t flags,
			struct timespec *timestamp,
			internal::Registrator &registrator)
			:
			mSection(section),
			mName(name),
			mFlags(flags),
			mTimestamp(timestamp),
			mRegistrator(registrator) {
	}

	/**
	 * Register a field inside a class.
	 * @param var: reference to registered variable.
	 * @param name: name of variable inside the class.
	 * @param flags: flags for the variable.
	 */
	template<typename T>
	inline void reg(T &var, const std::string &name,
			uint32_t flags=Flags::INHERIT) {
		/* Update parameters with parent class info */
		std::string newName(mName + "." + name);
		if (flags == Flags::INHERIT)
			flags = mFlags;
		mRegistrator.reg(var, mSection, newName, flags, mTimestamp);
	}
};

/**
 * Producer class. Used to register variables sharing the same section. All
 * variables will be put in the same sample at the same timestamp.
 */
class Producer : public internal::Registrator {
private:
	std::string     mSection;     /**< Section */
	std::string     mDir;         /**< Shared memory directory */
	void            *mData;       /**< Pointer to internal cached data */
	size_t          mDataSize;    /**< Size of internal cached data */
	struct shd_ctx  *mShdCtx;     /**< Shared memory context */
	uint32_t        mMaxSamples;  /**< Maximum number of samples */
	uint32_t        mRate;        /**< Approximative rate of samples (in us) */

	/** Constructor */
	Producer(const std::string &section, const std::string &dir,
			uint32_t maxSamples, uint32_t rate);

	/** Destructor */
	~Producer();

	/* Not implemented, just private to avoid copy */
	Producer(const Producer &);
	Producer &operator=(const Producer &);

	void setupVarInfoMap();
	void setupShd();

public:
	inline const std::string &getSection() const {
		return mSection;
	}

	inline const VarDescMap &getVarDescMap() const {
		return mVarDescMap;
	}

	/**
	 * Variable registration.
	 * @param var: reference to registered variable.
	 * @param name: name of the variable.
	 * @param flags: flags for the variable.
	 */
	template<typename T>
	inline void reg(T &var, const std::string &name,
			uint32_t flags = Flags::DEFAULT) {
		Registrator::reg(var, mSection, name, flags, NULL);
	}

	/** Notify end of registrations */
	void regComplete();

	/** Recreate shared memory sections with new revision */
	void reset();

	/**
	 * Put a new sample for variables in the section at the given timestamp.
	 * @param timestamp: timestamp of the sample, NULL to use current time.
	 */
	void putSample(const struct timespec *timestamp);

	/**
	 * Create a new producer.
	 * @param section: name of the producer (section).
	 * @param maxSamples: Maximum number of samples.
	 * @param rate: Approximative rate of samples (in us).
	 * @return created producer.
	 */
	inline static Producer *create(const std::string &section,
			uint32_t maxSamples, uint32_t rate) {
		return new Producer(section, "", maxSamples, rate);
	}

	/**
	 * Create a new producer.
	 * @param section: name of the producer (section).
	 * @param dir: directory where to create the shared memory. If empty
	 * will defaults to "/dev/shm".
	 * @param maxSamples: Maximum number of samples.
	 * @param rate: Approximative rate of samples (in us).
	 * @return created producer.
	 */
	inline static Producer *create(const std::string &section,
			const std::string &dir,
			uint32_t maxSamples, uint32_t rate) {
		return new Producer(section, dir, maxSamples, rate);
	}

	/**
	 * Destroy a producer.
	 * @param producer: producer to destroy.
	 */
	inline static void release(Producer *producer) {
		delete producer;
	}
};

/**
 * Consumer class. Used to register variables in multiple sections to query for
 * samples. It will internally use the correct section of the producer.
 */
class Consumer : public internal::Registrator {
private:
	typedef std::map<std::string, internal::ShdCtx *>  ShdCtxPtrMap;
	typedef std::vector<internal::VarInfo *>           VarInfoPtrVector;
	typedef std::map<std::string, VarInfoPtrVector>    VarInfoMapBySection;

private:
	std::string          mDir;                  /**< Shared memory directory */
	ShdCtxPtrMap         mShdCtxPtrMap;         /**< Shared memory contexts */
	VarInfoMapBySection  mVarInfoMapBySection;  /**< Variables in each shared memory */

private:
	/** Constructor */
	Consumer(const std::string &dir);

	/** Destructor */
	~Consumer();

	/* Not implemented, just private to avoid copy */
	Consumer(const Consumer &);
	Consumer &operator=(const Consumer &);

	bool setupShdCtx(internal::ShdCtx *shdCtx);
	bool getSample(internal::ShdCtx *shdCtx, const struct timespec *timestamp, Method method);

public:
	/**
	 * Variable registration.
	 * @param var: reference to registered variable.
	 * @param name: name of the variable.
	 * @param timestamp: pointer to timestamp where to store queried sample.
	 */
	template<typename T>
	inline void reg(T &var, const std::string &name,
			struct timespec *timestamp = NULL) {
		Registrator::reg(var, "", name, 0, timestamp);
	}

	/** Notify end of registrations */
	void regComplete();

	/**
	 * Get a new sample.
	 * @param timestamp: timestamp of query (can be NULL for LATEST method).
	 * @param method: method of query.
	 * @return true if at least one sample has been read in one of the
	 * shared memory where variables are.
	 */
	bool getSample(const struct timespec *timestamp, Method method);

	/**
	 * Get sample before the given timestamp.
	 * @param timestamp: timestamp read from a previous call.
	 * @return true if a sample was found.
	 * @remarks: this will only work if all variables are part of the same
	 * section.
	 */
	bool getPrevSample(const struct timespec *timestamp);

	/**
	 * Get sample after the given timestamp.
	 * @param timestamp: timestamp read from a previous call.
	 * @param timeout: max time to wait for the sample (in ms), 0 for no wait.
	 * @return true if a sample was found.
	 * @remarks: this will only work if all variables are part of the same
	 * section.
	 */
	bool getNextSample(const struct timespec *timestamp, uint32_t timeout);

	/**
	 * Create a new consumer.
	 * @return created consumer.
	 */
	inline static Consumer *create() {
		return new Consumer("");
	}

	/**
	 * Create a new consumer.
	 * @param dir: directory where to open the shared memory. If empty
	 * will defaults to "/dev/shm".
	 * @return created consumer.
	 */
	inline static Consumer *create(const std::string &dir) {
		return new Consumer(dir);
	}

	/**
	 * Destroy a consumer.
	 * @param consumer: consumer to destroy.
	 */
	inline static void release(Consumer *consumer) {
		delete consumer;
	}
};

/** */
class LoggerCb {
public:
	inline LoggerCb() {}
	inline virtual ~LoggerCb() {}

	/**
	 * Notification of section added.
	 * @param sectionId: internal Id of section.
	 */
	virtual void sectionAdded(uint32_t sectionId) = 0;

	/**
	 * Notification of section removed.
	 * @param sectionId: internal Id of section.
	 */
	virtual void sectionRemoved(uint32_t sectionId) = 0;

	/** Notification of section header changed
	 * @param sectionId: internal Id of section.
	 * @param buf: header with variable metadata.
	 * @param len: size of header.
	 */
	virtual void sectionChanged(uint32_t sectionId,
			const void *buf, size_t len) = 0;

	/**
	 * Start of a sample in a section.
	 * @param sectionId: internal Id of section.
	 * @param timestamp: timestamp of sample.
	 * @param buf: full data of sample.
	 * @param len: size of data of sample.
	 */
	virtual void sampleBegin(uint32_t sectionId,
			const struct timespec *timestamp,
			const void *buf, size_t len) = 0;

	/**
	 * End of a sample in a section
	 * @param sectionId: internal Id of section.
	 */
	virtual void sampleEnd(uint32_t sectionId) = 0;

	/**
	 * Contents of a variable in a sample.
	 * @param sectionId: internal Id of section.
	 * @param timestamp: timestamp of sample.
	 * @param varDesc: variable description.
	 * @param buf: data of variable.
	 * @param len: size of data of variable.
	 */
	virtual void sample(uint32_t sectionId,
			const struct timespec *timestamp,
			uint32_t varId,
			const VarDesc &varDesc,
			const void *buf, size_t len) = 0;

	/**
	  * Can be overriden by child class if needed.
	  */
	virtual void reset() {};

};

/** */
class Logger {
private:
	class SectionInfo;
	typedef std::vector<SectionInfo *>  SectionInfoVector;
	typedef std::vector<std::string>    StringVector;

private:
	std::string        mDir;                  /**< Shared memory directory */
	SectionInfoVector  mSectionInfoVector;
	uint32_t           mNextSectionId;
	uint64_t           mLastSectionListTime;
	StringVector       mFilter;

private:
	/** Constructor */
	Logger(const std::string &dir);

	/** Destructor */
	~Logger();

	/* Not implemented, just private to avoid copy */
	Logger(const Logger &);
	Logger &operator=(const Logger &);

	SectionInfo *findSectionInfo(const std::string &section) const;
	SectionInfo *findSectionInfo(uint32_t sectionId) const;
	bool isSectionMatch(const std::string &section) const;
	void getSectionList(std::vector<std::string> &sections) const;
	void checkSectionList(LoggerCb *cb);
	bool setupShd(SectionInfo *sectionInfo);
	void notifySample(SectionInfo *sectionInfo, LoggerCb *cb, uint32_t idx);
	bool fetchNextSamples(SectionInfo *sectionInfo, LoggerCb *cb);

public:
	/**
	 * Ask the logger to notify the given callback for sections currently
	 * opened. Useful when a callback has dropped previous event because
	 * its internal state was disabled and wants to know the current state.
	 * @param cb : callback for notification
	 */
	void replaySectionEvents(LoggerCb *cb);

	/**
	 * Set the filter of sections to log. This is a ';' separated list of
	 * patterns to match. Patterns can contains wildcard.
	 * Default is '*' (all sections)
	 * @param filter: new filter to apply
	 */
	void setFilter(const std::string &filter);

	/**
	 * Fetch as much samples as possible and notify them in callback.
	 * @param cb : callback for notification
	 * @return true if some sample were fetched, false otherwise.
	 */
	bool fetchNextSamples(LoggerCb *cb);

	/**
	 * Get the name of a section.
	 * @param sectionId: internal Id of section.
	 * @return name of the section.
	 */
	const std::string &getSection(uint32_t sectionId) const;

	/**
	 * Get the number of variables in a section.
	 * @param sectionId: internal Id of section.
	 * @return number of variables in the section.
	 */
	uint32_t getVarCount(uint32_t sectionId) const;

	/**
	 * Get the description of a variable.
	 * @param sectionId: internal Id of section.
	 * @param varId: Id (index) of the variable in the section.
	 * @return description of the variable.
	 */
	const VarDesc &getVarDesc(uint32_t sectionId, uint32_t varId) const;

	/**
	 * Reset logger.
	 * Handle reset of time clock.
	 */
	void reset();

	/**
	 * Create a new logger.
	 * @return created logger.
	 */
	inline static Logger *create() {
		return new Logger("");
	}

	/**
	 * Create a new logger.
	 * @param dir: directory where to open the shared memory. If empty
	 * will defaults to "/dev/shm".
	 * @return created logger.
	 */
	inline static Logger *create(const std::string &dir) {
		return new Logger(dir);
	}

	/**
	 * Destroy a logger.
	 * @param logger: logger to destroy.
	 */
	inline static void release(Logger *logger) {
		delete logger;
	}
};

/* Some implementation of internal stuff requires ClassDesc */
namespace internal {

/** Registration helper for class types */
template<typename T>
inline void Registrator::RegHelper<T, true>::reg(Registrator &registrator, T &var,
		const std::string &section,
		const std::string &name,
		uint32_t flags,
		struct timespec *timestamp) {
	ClassDesc classDesc(section, name, flags,
			timestamp, registrator);
	var.telemetryRegister(classDesc);
}

/** Registration helper for arrays of class types */
template<typename T, size_t N>
inline void Registrator::RegHelper<T[N], true>::reg(Registrator &registrator, T (&var)[N],
		const std::string &section,
		const std::string &name,
		uint32_t flags,
		struct timespec *timestamp) {
	for (size_t i = 0; i < N; i++) {
		std::string itemName = name + "." + std::to_string(i);
		ClassDesc classDesc(section, itemName, flags,
				timestamp, registrator);
		var[i].telemetryRegister(classDesc);
	}
}

} /* namespace internal */

} /* namespace telemetry */

#endif /* !_LIBTELEMETRY_HPP_ */
