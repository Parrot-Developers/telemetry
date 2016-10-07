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

###############################################################################
###############################################################################
class Navdata(object):
    def __init__(self):
        self.titles = []
        self.datasize = 0
        self.nentries = 0
        self.samples = []

    def parse(self, fd):
        self._parseHeader(fd)
        self._parseData(fd)

    def _parseHeader(self, fd):
        while True :
            line = fd.readline()
            if line == b"-- Data\n":
                break
            if line.startswith(b"nentries: "):
                self.nentries = int(line[10:-1])
            elif line.startswith(b"datasize: "):
                self.datasize = int(line[10:-1])
            elif line.startswith(b"titles: "):
                self.titles = line[8:-1].decode("UTF-8").split(", ")

        assert len(self.titles) == self.nentries
        assert self.datasize == 8

    def _parseData(self, fd):
        while True :
            sample = []
            for _ in range(0, self.nentries):
                raw = fd.read(8)
                if len(raw) != 8:
                    return
                sample.append(struct.unpack("d", raw)[0])
            self.samples.append(sample)

###############################################################################
###############################################################################
def writeApmLog(navdata, filePath):
    try:
        out = open(filePath, "w")
    except IOError as ex:
        sys.stderr.write("Unable to create '%s': err=%d(%s)\n" % (
                ex.filename, ex.errno, ex.strerror))
        return

    reclen = 8 * navdata.nentries
    fmt = "f" * navdata.nentries
    names = ",".join(navdata.titles).replace("index", "_index")

    # Write header
    out.write("FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n")
    out.write("FMT, 200, %d, DATA, %s, %s\n" % (reclen, fmt, names))

    # Write data
    for sample in navdata.samples:
        out.write("DATA")
        for val in sample:
            out.write(", %f" % val)
        out.write("\n")

    out.close()

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

    outDir = "."

    navdata = Navdata()
    navdata.parse(fd)
    fd.close()

    filePath = os.path.join(outDir, os.path.basename(sys.argv[1] + ".apm.log"))
    writeApmLog(navdata, filePath)

###############################################################################
###############################################################################
if __name__ == "__main__":
    main()
