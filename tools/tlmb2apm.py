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
import argparse
from tlmb_parser import TlmbCb, TlmbFile, TlmbDataReader, TlmbVarType, TlmbError

# Support for 64-bit quantity supported ?
# As of 2015/07/31 apm_planner does not support this for text logs
SUPPORT_Qq = False

###############################################################################
###############################################################################
class Context(TlmbCb):
    _varType2Fmt = {
        TlmbVarType.BOOL: ("B", "%d"),
        TlmbVarType.UINT8: ("B", "%d"),
        TlmbVarType.INT8: ("b", "%d"),
        TlmbVarType.UINT16: ("H", "%d"),
        TlmbVarType.INT16: ("h", "%d"),
        TlmbVarType.UINT32: ("I", "%d"),
        TlmbVarType.INT32: ("i", "%d"),
        TlmbVarType.UINT64: ("Q" if SUPPORT_Qq else "I", "%d"),
        TlmbVarType.INT64: ("q" if SUPPORT_Qq else "i", "%d"),
        TlmbVarType.FLOAT32: ("f", "%f"),
        TlmbVarType.FLOAT64: ("f", "%f"),   # No double supported
    }

    def __init__(self, outDir, use_rawts):
        self._outDir = outDir
        self._use_rawts = use_rawts
        self._outputs = {}
        self._refts = {}
        self._header = None

    def onFileHeader(self, header):
        self._header = header

    def onSectionBegin(self, section):
        # Construct file path
        filePath = os.path.join(self._outDir, section.sectionName + "-apm")
        if section.sectionId not in self._outputs:
            self._outputs[section.sectionId] = {}
        num = len(self._outputs[section.sectionId])
        if num > 0:
            filePath += "-%d" % num
        filePath += ".log"

        # Create output file
        try:
            out = open(filePath, "w")
        except IOError as ex:
            sys.stderr.write("Unable to create '%s': err=%d(%s)\n" % (
                    ex.filename, ex.errno, ex.strerror))
            return
        self._outputs[section.sectionId][section] = out
        self._refts[section] = None

        # Determine length of record and format (adding timestamp as uint64)
        reclen = 8
        fmt = "Q" if SUPPORT_Qq else "I"
        names = "TimeUS"
        for varDesc in section.varDescs:
            if varDesc.varType not in Context._varType2Fmt:
                continue
            reclen += varDesc.size * varDesc.count
            fmt += Context._varType2Fmt[varDesc.varType][0] * varDesc.count
            if varDesc.count == 1:
                names += ",%s" % varDesc.name
            else:
                for i in range(0, varDesc.count):
                    names += ",%s%d" % (varDesc.name, i)

        # Write header
        out.write("FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n")
        out.write("FMT, 200, %d, DATA, %s, %s\n" % (reclen, fmt, names))

    def onSectionEnd(self, section):
        # Get output file
        out = self._outputs[section.sectionId].get(section, None)
        if out is not None:
            out.close()
            self._outputs[section.sectionId][section] = None

    def onSample(self, section, sample):
        # Get output file
        out = self._outputs[section.sectionId].get(section, None)
        if out is None:
            return

        # Timestamp (minus first used as reference to avoid values too big
        ts = sample.ts[0] * 1000 * 1000 + sample.ts[1] // 1000
        if not self._use_rawts:
            if self._refts[section] is None:
                self._refts[section] = ts
            ts -= self._refts[section]

        # Data
        out.write("DATA, %d" % ts)
        quantities = sample.extractQuantities(section.varDescs)
        for i in range(0, len(quantities)):
            varDesc = section.varDescs[i]
            if varDesc.varType not in Context._varType2Fmt:
                continue
            fmt = Context._varType2Fmt[varDesc.varType][1]
            if varDesc.count == 1:
                out.write(", " + fmt % quantities[i])
            else:
                for j in range(0, varDesc.count):
                    out.write(", " + fmt % quantities[i][j])
        out.write("\n")

###############################################################################
###############################################################################
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile", type=str, nargs=1,
                        help="File generated by telemetryd blackbox.")
    parser.add_argument("-r", "--raw-ts", dest="raw_ts", action="store_true",
                        default=False,
                        help="Use raw timestamp instead of relative ones")
    args = parser.parse_args()

    try:
        fd = open(args.logfile[0], "rb")
    except IOError as ex:
        sys.stderr.write("Unable to open '%s': err=%d(%s)\n" % (
                ex.filename, ex.errno, ex.strerror))
        sys.exit(1)

    outDir = "."

    try:
        TlmbFile(TlmbDataReader(fd), Context(outDir, args.raw_ts))
    except TlmbError as ex:
        sys.stderr.write("%s\n" % ex)
        sys.exit(1)

    fd.close()

###############################################################################
###############################################################################
if __name__ == "__main__":
    main()
