#!/usr/bin/env python3

import sys
import numpy as np
import tlmb_parser as tlmb

""" Read TLMB files and present the data series into Numpy arrays

    Sections and variables become attributes of a Python object. See __main__
    for example usage.
"""

def _dict_to_attr(obj, d):
    # Transform dictionary into attributes
    for k, v in d.items():
        setattr(obj, k, v)
    return obj


class SectionReader:
    def __init__(self, section):
        self.var = dict()
        self.nbsamples = 0
        self.ts = list()
        self.data = dict()
        for v in section.varDescs:
            self.var[v.name] = dict()
            self.var[v.name]["dtype"] = ReadCb._to_dtype(v.varType)
            self.var[v.name]["samples"] = list()
            self.var[v.name]["count"] = v.count

    def add(self, section, sample):
        self.ts.append(sample.ts)
        self.nbsamples += 1
        quantities = sample.extractQuantities(section.varDescs)
        assert(len(quantities) == len(section.varDescs))
        for i in range(0, len(section.varDescs)):
            self.var[section.varDescs[i].name]["samples"].append(quantities[i])

    def finish(self, section):
        assert(self.nbsamples == section.sampleCount)
        assert(self.nbsamples == len(self.ts))

        if self.nbsamples == 0:
            print("Empty section: %s" % section.sectionName)
            return
        ts = np.asarray(self.ts, dtype=np.uint64)
        self.data["ts"] = ts[:,0] * (1000 * 1000 * 1000) + ts[:,1]

        for k, v in self.var.items():
            assert(self.nbsamples == len(v["samples"]))
            # Skip data series with count==0
            if v["count"] == 0:
                continue
            # Variables with count = 1 give 1D Numpy arrays
            # Variables with count > 1 give 2D Numpy arrays
            self.data[k] = np.asarray(v["samples"], dtype=v["dtype"])
            if v["count"] > 1:
                assert(self.data[k].shape[1] == v["count"])

        # Compute time in seconds
        if "time_s" not in self.data.keys():
            self.data["time_s"] = self.data["ts"] / 1e9


class FileReader:
    def __init__(self):
        self.data = dict()

    def add(self, name, value):
        if name not in self.data.keys():
            self.data[name] = list()
        self.data[name].append(value)

    def finish(self, obj):
        # Convert SectionReader's to InnerReader's with corresponding attributes
        for k, v in self.data.items():
            self.data[k] = [ _dict_to_attr(InnerReader(), x.data) for x in v ]
        # Transfer dictionary as attributes of obj
        _dict_to_attr(obj, self.data)


class ReadCb(tlmb.TlmbCb):
    def __init__(self):
        self.section = dict()
        self.reader = FileReader()

    @staticmethod
    def _to_dtype(tlmb_type):
        d = {
            tlmb.TlmbVarType.BOOL:       np.dtype("b"), 
            tlmb.TlmbVarType.UINT8:      np.dtype("u1"),
            tlmb.TlmbVarType.INT8:       np.dtype("i1"),
            tlmb.TlmbVarType.UINT16:     np.dtype("u2"), 
            tlmb.TlmbVarType.INT16:      np.dtype("i2"),
            tlmb.TlmbVarType.UINT32:     np.dtype("u4"),
            tlmb.TlmbVarType.INT32:      np.dtype("i4"),
            tlmb.TlmbVarType.UINT64:     np.dtype("u8"),
            tlmb.TlmbVarType.INT64:      np.dtype("i8"),
            tlmb.TlmbVarType.FLOAT32:    np.dtype("f4"),
            tlmb.TlmbVarType.FLOAT64:    np.dtype("f8"),
            tlmb.TlmbVarType.STRING:     np.dtype("S255"),
            tlmb.TlmbVarType.BINARY:     np.dtype("V")
        }
        return d[tlmb_type]

    def onFileHeader(self, header):
        for k in sorted(header.keys()):
            print("%s: %s" % (k, header[k]))

    def onSectionBegin(self, section):
        if section.sectionName in self.section.keys():
            # Section already exists: error
            raise ValueError("Already have section %s" % section.sectionName)
        self.section[section.sectionName] = SectionReader(section)

    def onSectionEnd(self, section):
        self.section[section.sectionName].finish(section)
        # Do not keep empty sections
        if len(self.section[section.sectionName].data) != 0:
            self.reader.add(section.sectionName, self.section[section.sectionName])
        # Remove from dictionary
        self.section.pop(section.sectionName)

    def onSample(self, section, sample):
        self.section[section.sectionName].add(section, sample)


class InnerReader:
    # Empty object whose attributes will be set programmatically
    pass


class Reader:
    # Empty object: sections will become attributes
    def __init__(self, filename):
        r = ReadCb()
        tlmb.TlmbFile(tlmb.TlmbDataReader(open(filename, "rb")), r)
        r.reader.finish(self)

def nr_test(r):
    import pickle
    pickle.dump(r, open("reader.dump", "wb"))
    quit()

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    for d in sys.argv[1:]:
        r = Reader(d)
        try:
            for a in r.navdata:
                plt.plot(a.time_s, a.sensor_ultrasound_height_m, label="US")
                plt.legend()
                plt.show()
        except AttributeError:
            print("No navdata in '%s'" % d)

        try:
            for a in r.heat:
                plt.subplot(211)
                plt.plot(a.time_s, a.us_rx_ref_temp_kelvin, label="US RX ref temp", linestyle="--")
                plt.plot(a.time_s, a.us_rx_last_temp_kelvin, label="US RX last temp")
                plt.plot(a.time_s, a.us_tx_ref_temp_kelvin, label="US TX ref temp", linestyle="--")
                plt.plot(a.time_s, a.us_tx_last_temp_kelvin, label="US TX last temp")
                plt.legend()
                plt.subplot(212)
                plt.plot(a.time_s, a.us_rx_pwm_duty_cycle, label="US RX pwm")
                plt.plot(a.time_s, a.us_tx_pwm_duty_cycle, label="US TX pwm")
                plt.legend()
                plt.show()
        except AttributeError:
            print("No heat in '%s'" % d)

        try:
            for a in r.lidar:
                plt.subplot(211)
                plt.plot(a.time_s, a.distance_m, label="distance")
                plt.legend()
                plt.subplot(212)
                plt.plot(a.time_s, a.confidence_pc, label="confidence")
                plt.legend()
                plt.show()
        except AttributeError:
            print("No lidar in '%s'" % d)

        try:
            for a in r.ultrasound:
                plt.subplot(211)
                plt.plot(a.time_s, a.distance_m, label="distance")
                plt.legend()
                plt.subplot(212)
                plt.plot(a.time_s, a.snr_db, label="SNR")
                plt.legend()
                plt.show()
        except AttributeError:
            print("No ultrasound in '%s'" % d)
