#!/usr/bin/env python
#
# Copyright (c) 2015 Parrot S.A.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the <organization> nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import sys, os
import struct
import zlib
from io import BytesIO

###############################################################################
###############################################################################
class TlmbError(Exception):
    pass

###############################################################################
###############################################################################
class TlmbCb(object):
    def onFileHeader(self, header):
        raise NotImplementedError()
    def onSectionBegin(self, section):
        raise NotImplementedError()
    def onSectionEnd(self, section):
        raise NotImplementedError()
    def onSample(self, section, sample):
        raise NotImplementedError()

###############################################################################
###############################################################################
class TlmbDataReader(object):
    def __init__(self, src):
        if isinstance(src, str) or isinstance(src, bytes):
            self._src = BytesIO(src)
        else:
            self._src = src
        # Determine size
        self._src.seek(0, os.SEEK_END)
        self._size = self._src.tell()
        self._src.seek(0, os.SEEK_SET)

    def readData(self, count):
        data = self._src.read(count)
        if len(data) != count:
            raise TlmbError("Unable to read %d bytes" % count)
        return data

    def readU8(self):
        return struct.unpack("<B", self.readData(1))[0]

    def readU32(self):
        return struct.unpack("<I", self.readData(4))[0]

    def readString(self):
        slen = self.readU8()
        if slen == 0:
            raise TlmbError("Invalid string length: %d" % slen)
        sdata = self.readData(slen).decode("UTF-8")
        if len(sdata) == 0 or sdata[-1] != "\0":
            raise TlmbError("String is not null-terminated")
        return sdata[:-1]

    def remaining(self):
        return self._size - self._src.tell()

###############################################################################
###############################################################################
class TlmbVarType(object):
    INVALID = -1
    BOOL = 0
    UINT8 = 1
    INT8 = 2
    UINT16 = 3
    INT16 = 4
    UINT32 = 5
    INT32 = 6
    UINT64 = 7
    INT64 = 8
    FLOAT32 = 9
    FLOAT64 = 10
    STRING = 11
    BINARY = 12

    _TO_STRING = {
        BOOL:    "BOOL",
        UINT8:   "UINT8",
        INT8:    "INT8",
        UINT16:  "UINT16",
        INT16:   "INT16",
        UINT32:  "UINT32",
        INT32:   "INT32",
        UINT64:  "UINT64",
        INT64:   "INT64",
        FLOAT32: "FLOAT32",
        FLOAT64: "FLOAT64",
        STRING:  "STRING",
        BINARY:  "BINARY",
    }

    @staticmethod
    def toString(_type):
        return TlmbVarType._TO_STRING.get(_type, "Unknown(%d)" % _type)

###############################################################################
###############################################################################
class TlmbVarDesc(object):
    def __init__(self, section, rec):
        self.section = section
        reader = TlmbDataReader(rec)
        namelen = reader.readU32()
        self.varType = reader.readU32()
        self.size = reader.readU32()
        self.count = reader.readU32()
        self.flags = reader.readU32()
        self.name = reader.readData(namelen).decode("UTF-8")
        self.flatOff = None

    def getTotalSize(self):
        return self.size * self.count

    def getFullName(self):
        return self.section.sectionName + "." + self.name

    def getUnpackFmt(self):
        _type = self.varType
        if _type == TlmbVarType.BOOL:
            return "%d?" % self.count
        elif _type == TlmbVarType.UINT8:
            return "%dB" % self.count
        elif _type == TlmbVarType.INT8:
            return "%db" % self.count
        elif _type == TlmbVarType.UINT16:
            return "%dH" % self.count
        elif _type == TlmbVarType.INT16:
            return "%dh" % self.count
        elif _type == TlmbVarType.UINT32:
            return "%dI" % self.count
        elif _type == TlmbVarType.INT32:
            return "%di" % self.count
        elif _type == TlmbVarType.UINT64:
            return "%dQ" % self.count
        elif _type == TlmbVarType.INT64:
            return "%dq" % self.count
        elif _type == TlmbVarType.FLOAT32:
            return "%df" % self.count
        elif _type == TlmbVarType.FLOAT64:
            return "%dd" % self.count
        elif _type == TlmbVarType.STRING:
            return "%ds" % self.getTotalSize()
        elif _type == TlmbVarType.BINARY:
            return "%dp" % self.getTotalSize()
        else:
            raise TlmbError("Unknown var type %d" % _type)

    def __repr__(self):
        return "{name='%s', type=%s, size=%d, count=%d, flags=0x%x}" % (
                self.name, TlmbVarType.toString(self.varType),
                self.size, self.count, self.flags)

###############################################################################
###############################################################################
class TlmbVarDescList(list):
    def __init__(self):
        list.__init__(self)
        self.unpackFmt = None

    def setupUnpackfmt(self):
        self.unpackFmt = "<"
        size = 0
        off = 0
        for varDesc in self:
            self.unpackFmt += varDesc.getUnpackFmt()
            size += varDesc.getTotalSize()
            varDesc.flatOff = off
            off += varDesc.count
        if size != struct.calcsize(self.unpackFmt):
            raise TlmbError("Unable to setup struct.unpack format")

###############################################################################
###############################################################################
class TlmbSample(object):
    def __init__(self, ts, data):
        self.ts = ts
        self.data = data

    def extractQuantities(self, varDescs, flat=False):
        quantitiesFlat = struct.unpack(varDescs.unpackFmt, self.data)
        if flat:
            return quantitiesFlat
        quantities = [
            quantitiesFlat[varDesc.flatOff] if varDesc.count == 1
                else quantitiesFlat[varDesc.flatOff:varDesc.flatOff+varDesc.count]
            for varDesc in varDescs
        ]
        return quantities

    @staticmethod
    def extractQuantity(varDesc, data):
        quantity = struct.unpack("<" + varDesc.getUnpackFmt(), data)
        return quantity[0] if varDesc.count == 1 else quantity

###############################################################################
###############################################################################
class TlmbSection(object):
    def __init__(self, sectionId, sectionName):
        self.sectionId = sectionId
        self.sectionName = sectionName
        self.varDescs = TlmbVarDescList()
        self.samples = []
        self.sampleCount = 0
        self.headerRead = False
        self.finished = False

    def readHeader(self, header):
        assert len(self.varDescs) == 0
        assert len(self.samples) == 0
        assert not self.headerRead
        assert not self.finished
        reader = TlmbDataReader(header)
        count = reader.readU32()
        for _ in range(0, count):
            reclen = reader.readU32()
            rec = reader.readData(reclen - 4)
            self.varDescs.append(TlmbVarDesc(self, rec))
        self.headerRead = True
        self.varDescs.setupUnpackfmt()

    def addSample(self, ts, data, store=True):
        assert self.headerRead
        assert not self.finished
        sample = TlmbSample(ts, data)
        if store:
            self.samples.append(sample)
        self.sampleCount += 1
        return sample

    def __repr__(self):
        return "{id=%d, name=%s, sampleCount=%d, vars=%s}" % (
                self.sectionId, self.sectionName,
                self.sampleCount, repr(self.varDescs))

###############################################################################
###############################################################################
class TlmbBlock(object):
    _TAG_SECTION_ADDED = 0
    _TAG_SECTION_REMOVED = 1
    _TAG_SECTION_CHANGED = 2
    _TAG_SAMPLE = 3

    def __init__(self, parent, reader, cb=None):
        self._parent = parent
        self._reader = reader
        self._cb = cb
        while self._readTag():
            pass

    def _readTag(self):
        # Stop at end of reader
        if self._reader.remaining() < 1:
            return False

        # Read tag, process its data
        tag = self._reader.readU8()
        try:
            processMap = {
                TlmbBlock._TAG_SECTION_ADDED: self._readTagSectionAdded,
                TlmbBlock._TAG_SECTION_REMOVED: self._readTagSectionRemoved,
                TlmbBlock._TAG_SECTION_CHANGED: self._readTagSectionChanged,
                TlmbBlock._TAG_SAMPLE: self._readTagSample,
            }
            if tag not in processMap:
                sys.stderr.write("Unknown tag 0x%02x\n" % tag)
                return False
            processMap[tag]()
            return True
        except TlmbError as ex:
            # Print message and abort reading
            sys.stderr.write("%s\n" % ex)
            return False

    def _readTagSectionAdded(self):
        # Add a new section in table
        sectionId = self._reader.readU32()
        sectionName = self._reader.readString()
        section = TlmbSection(sectionId, sectionName)
        if sectionId not in self._parent.sections:
            self._parent.sections[sectionId] = []
        elif not self._parent.sections[sectionId][-1].finished:
            self._parent.endSection(self._parent.sections[sectionId][-1])
        self._parent.addSection(section)

    def _readTagSectionRemoved(self):
        # Mark section as finished
        sectionId = self._reader.readU32()
        section = self._parent.sections[sectionId][-1]
        self._parent.endSection(section)

    def _readTagSectionChanged(self):
        sectionId = self._reader.readU32()
        dataSize = self._reader.readU32()
        data = self._reader.readData(dataSize)
        section = self._parent.sections[sectionId][-1]
        if section.headerRead:
            # Recreate a new section and mark old as finished
            self._parent.endSection(section)
            section = TlmbSection(section.sectionId, section.sectionName)
            self._parent.addSection(section)
        section.readHeader(data)
        self._parent.beginSection(section)

    def _readTagSample(self):
        sectionId = self._reader.readU32()
        ts = (self._reader.readU32(), self._reader.readU32())
        dataSize = self._reader.readU32()
        data = self._reader.readData(dataSize)
        section = self._parent.sections[sectionId][-1]
        sample = section.addSample(ts, data, store=(self._cb is None))
        if self._cb:
            self._cb.onSample(section, sample)

###############################################################################
###############################################################################
class TlmbFile(object):
    _VERSION = 1         # File format version
    _MAGIC = 0x424d4c54  # File magic 'TLMB'

    def __init__(self, reader, cb=None):
        self._reader = reader
        self._cb = cb
        self.magic = 0
        self.version = 0
        self.header = {}
        self.sections = {}

        self._readHeader()
        if self._cb:
            self._cb.onFileHeader(self.header)
        while self._readBlock():
            pass
        for sectionId in self.sections:
            if not self.sections[sectionId][-1].finished:
                self.endSection(self.sections[sectionId][-1])

    def addSection(self, section):
        self.sections[section.sectionId].append(section)

    def beginSection(self, section):
        if self._cb:
            self._cb.onSectionBegin(section)

    def endSection(self, section):
        section.finished = True
        if self._cb:
            self._cb.onSectionEnd(section)

    def _readHeader(self):
        # Read file magic and version
        self.magic = self._reader.readU32()
        self.version = self._reader.readU8()
        if self.magic != TlmbFile._MAGIC:
            raise TlmbError("Bad magic: 0x%08x(0x%08x)" % (self.magic, TlmbFile._MAGIC))
        if self.version != TlmbFile._VERSION:
            raise TlmbError("Bad version: %d" % self.version)

        # Get number of properties in header
        count = self._reader.readU32()
        for _ in range(0, count):
            key = self._reader.readString()
            val = self._reader.readString()
            self.header[key] = val

    def _readBlock(self):
        # Stop at end of reader
        if self._reader.remaining() < 4:
            return False

        # Read block size, check for end of file marker
        blockSize = self._reader.readU32()
        if blockSize == 0xffffffff:
            return False

        try:
            # Read data, uncompress if needed
            blockData = self._reader.readData(blockSize)
            if self.header.get("compression", "0") == "1":
                blockData = zlib.decompress(blockData)
        except TlmbError as ex:
            # Print message and stop reading blocks
            sys.stderr.write("Truncated block\n")
            return False
        except zlib.error as ex:
            # Print message and skip this block
            sys.stderr.write("%s\n" % ex)
            return True

        # Parse block
        print("Parse block: size=%d" % blockSize)
        TlmbBlock(self, TlmbDataReader(blockData), self._cb)
        return True

###############################################################################
###############################################################################
def usage():
    sys.stderr.write("usage: %s <logfile>\n" % sys.argv[0])

###############################################################################
###############################################################################
def main():
    if len(sys.argv) != 2:
        usage()
        sys.exit(1)
    elif sys.argv[1] == "-h" or sys.argv[1] == "--help":
        usage()
        sys.exit(0)

    try:
        fd = open(sys.argv[1], "rb")
    except IOError as ex:
        sys.stderr.write("Unable to open '%s': err=%d(%s)\n" % (
                ex.filename, ex.errno, ex.strerror))
        sys.exit(1)

    class MyCb(TlmbCb):
        def onFileHeader(self, header):
            print(header)
        def onSectionBegin(self, section):
            pass
        def onSectionEnd(self, section):
            print("section %d(%s): %d samples" % (
                    section.sectionId, section.sectionName, section.sampleCount))
            for varDesc in section.varDescs:
                print(repr(varDesc))
        def onSample(self, section, sample):
            pass

    try:
        TlmbFile(TlmbDataReader(fd), MyCb())
    except TlmbError as ex:
        sys.stderr.write("%s\n" % ex)
        sys.exit(1)

    fd.close()

###############################################################################
###############################################################################
if __name__ == "__main__":
    main()
