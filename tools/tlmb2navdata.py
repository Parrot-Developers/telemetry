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
#   * Neither the name of the Parrot Company nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import sys, os
import struct
import argparse
from tlmb_parser import TlmbCb, TlmbFile, TlmbDataReader, TlmbVarType, TlmbError

###############################################################################
###############################################################################
def convert2float(val):
    try:
        return float(val)
    except ValueError:
        return 0.0

###############################################################################
###############################################################################
class Context(TlmbCb):
    def __init__(self, outDir, float_format):
        self._outDir = outDir
        self._float_format = float_format
        self._outputs = {}
        self._pack_fmt = {}
        self._header = None

    def onFileHeader(self, header):
        self._header = header

    def onSectionBegin(self, section):
        # Construct file path
        filePath = os.path.join(self._outDir, section.sectionName + "-nvd")
        if section.sectionId not in self._outputs:
            self._outputs[section.sectionId] = {}
            self._pack_fmt[section.sectionId] = {}
        num = len(self._outputs[section.sectionId])
        if num > 0:
            filePath += "-%d" % num
        filePath += ".log"

        # Create output file
        try:
            out = open(filePath, "wb")
        except IOError as ex:
            sys.stderr.write("Unable to create '%s': err=%d(%s)\n" % (
                    ex.filename, ex.errno, ex.strerror))
            return
        self._outputs[section.sectionId][section] = out

        # Write header
        out.write(b"-- Build infos\n")
        for key in self._header:
            out.write(b"%s: %s\n" % (key.encode("UTF-8"), self._header[key].encode("UTF-8")))
        out.write(b"-- Navdata infos\n")

        # Determine number of entries (expanding arrays), index and timeStamp
        # are always added
        entryCount = 2
        for varDesc in section.varDescs:
            entryCount += varDesc.count
        out.write(b"nentries: %d\n" % entryCount)
        if self._float_format:
            out.write(b"datasize: 4\n")
        else:
            out.write(b"datasize: 8\n")

        # Construct pack format
        if self._float_format:
            self._pack_fmt[section.sectionId][section] = "<" + "f" * entryCount
        else:
            self._pack_fmt[section.sectionId][section] = "<" + "d" * entryCount

        # Names
        out.write(b"titles: index, time_s")
        for varDesc in section.varDescs:
            if varDesc.count == 1:
                out.write(b", %s" % varDesc.name.encode("UTF-8"))
            else:
                for i in range(0, varDesc.count):
                    out.write(b", %s%d" % (varDesc.name.encode("UTF-8"), i))
        out.write(b"\n")
        out.write(b"-- Data\n")

    def onSectionEnd(self, section):
        # Get output file
        out = self._outputs[section.sectionId].get(section, None)
        if out is not None:
            out.close()
            self._outputs[section.sectionId][section] = None
            self._pack_fmt[section.sectionId][section] = None

    def onSample(self, section, sample):
        # Get output file
        out = self._outputs[section.sectionId].get(section, None)
        if out is None:
            return

        # Index, timestamp and Data
        ts = sample.ts[0] + sample.ts[1] / (1000.0 * 1000.0 * 1000.0)
        quantities = sample.extractQuantities(section.varDescs, flat=True)
        values = [convert2float(q) for q in quantities]

        pack_fmt = self._pack_fmt[section.sectionId][section]
        out.write(struct.pack(pack_fmt, section.sampleCount - 1,  ts, *values))

###############################################################################
###############################################################################
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile", type=str, nargs=1,
                        help="file generated by telemetryd blackbox.")
    parser.add_argument("-f", "--float-format", action="store_true",
                        default=False,
                        help="export data to float instead of double")
    args = parser.parse_args()

    try:
        fd = open(sys.argv[1], "rb")
    except IOError as ex:
        sys.stderr.write("Unable to open '%s': err=%d(%s)\n" % (
                ex.filename, ex.errno, ex.strerror))
        sys.exit(1)

    outDir = "."

    try:
        TlmbFile(TlmbDataReader(fd), Context(outDir, args.float_format))
    except TlmbError as ex:
        sys.stderr.write("%s\n" % ex)
        sys.exit(1)

    fd.close()

###############################################################################
###############################################################################
if __name__ == "__main__":
    main()
