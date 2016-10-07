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
 * @file logfile.hpp
 *
 */

#ifndef _LOGFILE_HPP_
#define _LOGFILE_HPP_

/**
 * Write data to a log file, possibly compressed in independent blocks.
 */
class LogFile {
private:
	std::string  mPath;
	FILE         *mFile;
	long         mBlockBeginOffset;
	bool         mCompressed;
	z_stream     mZStream;

private:
	int writeDataCompressed(const void *buf, size_t len, bool flush);
	int flush();
	int seek(long off, int whence);

public:
	LogFile(const std::string &path, bool compressed);
	~LogFile();

	inline const std::string &getPath() const {return mPath;}
	inline bool isCompressed() const {return mCompressed;}

	int open();
	int close();
	int writeData(const void *buf, size_t len);
	int writeDataRaw(const void *buf, size_t len);

	int getCurrentBlockSize(uint32_t *size);
	int beginBlock();
	int finishBlock();
	int finishFile();
	int getPos(long *pos);
};

#endif /* !_LOGFILE_HPP_ */
