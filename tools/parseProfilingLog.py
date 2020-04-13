#!/usr/bin/env python

import sys
import re
import math

timings = {}

# parse the logs
with open(sys.argv[1], 'r') as logFile:
    for line in logFile:
        if "Profiling" in line and "ms" in line:
            functionName = re.findall("\[Profiling.*\]\s(.*)\s:\s.*ms", line)[0]
            runtime = re.findall("\[Profiling.*\]\s.*\s:\s(.*)\sms", line)[0]
            timings.setdefault(functionName, []).append(float(runtime))

# sort each runtime list.
for key in timings:
    timings[key].sort()

# print the percentiles.
for key in timings:
    print("{}:".format(key))
    print("    P10: {:.6f} ms".format(timings[key][int(math.floor(0.1 * (len(timings[key]) - 1)))]))
    print("    P50: {:.6f} ms".format(timings[key][int(math.floor(0.5 * (len(timings[key]) - 1)))]))
    print("    P90: {:.6f} ms".format(timings[key][int(math.floor(0.9 * (len(timings[key]) - 1)))]))
    print("    P99: {:.6f} ms".format(timings[key][int(math.floor(0.99 * (len(timings[key]) - 1)))]))
    print("")
