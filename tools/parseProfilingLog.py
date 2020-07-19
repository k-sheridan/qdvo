#!/usr/bin/env python

import sys
import re
import math


def parseProfilingLog(logFilePath, printOutput=False):
    timings = {}
    events = []
    trace = {}

    # parse the logs
    with open(logFilePath, 'r') as logFile:
        for line in logFile:
            if "Profiling" in line and "ms" in line:
                functionName = re.findall("\[Profiling.*\]\s(.*)\s:\s.*ms.*ms", line)[0]
                runtime = re.findall("\[Profiling.*\]\s.*\s:\s(.*)\sms.*ms", line)[0]
                timestamp = re.findall("\[Profiling.*\]\s.*\s:\s.*\sms\s:\s(.*)\sms", line)[0]
                timings.setdefault(functionName, []).append(float(runtime))
                events.append([str(functionName), float(timestamp), float(runtime)])

    # Log the trace events
    trace['traceEvents'] = []
    for functionName, timestamp, runtime in events:
        event = {}
        event['name'] = functionName
        event['tid'] = 'mainThread'
        event['pid'] = '1'
        event['ph'] = 'B'
        event['ts'] = timestamp * 1000
        trace['traceEvents'].append(event);

        eventEnd = {}
        eventEnd['name'] = functionName
        eventEnd['tid'] = 'mainThread'
        eventEnd['pid'] = '1'
        eventEnd['ph'] = 'E'
        eventEnd['ts'] = (timestamp * 1000) + (runtime * 1000)
        trace['traceEvents'].append(eventEnd);


    # sort each runtime list.
    for key in timings:
        timings[key].sort()
    
    # print the percentiles.
    result = {}
    for key in timings:
        p10 = timings[key][int(math.floor(0.1 * (len(timings[key]) - 1)))]
        p50 = timings[key][int(math.floor(0.5 * (len(timings[key]) - 1)))]
        p90 = timings[key][int(math.floor(0.9 * (len(timings[key]) - 1)))]
        p99 = timings[key][int(math.floor(0.99 * (len(timings[key]) - 1)))]

        result[key] = {}

        result[key]['p10_ms'] = p10
        result[key]['p50_ms'] = p50
        result[key]['p90_ms'] = p90
        result[key]['p99_ms'] = p99
        result[key]['mean_ms'] = sum(timings[key]) / len(timings[key])

        if printOutput:
            print("{}:".format(key))
            print("    P10: {:.6f} ms".format(p10))
            print("    P50: {:.6f} ms".format(p50))
            print("    P90: {:.6f} ms".format(p90))
            print("    P99: {:.6f} ms".format(p99))
            print("    Mean: {:.6f} ms".format(result[key]['mean_ms']))
            print("")

    return result, trace

if __name__ == "__main__":
    parseProfilingLog(sys.argv[1])
