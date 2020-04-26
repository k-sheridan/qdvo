import subprocess
import os
import argparse
import json
import csv

from parseProfilingLog import parseProfilingLog

parser = argparse.ArgumentParser(description='Evaluate a euroc format dataset using QDVO.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--datasetPath', type=str, required=True, help='Path to the folder above mav0 of the dataset.')
parser.add_argument('--visualizerBinaryPath', type=str, default='../build/visualizer/runEurocDatasets', help='Path to the binary used to run QDVO.')
parser.add_argument('--visualize', type=bool, default=False, help='Should the dataset be visualized while running?')
parser.add_argument('--frames', type=int, default=1e12, help='How many frames should the dataset be run for?')
parser.add_argument('--output', type=str, required=True, help='Path to the folder where the results will be output.')

args = parser.parse_args()

if not os.path.exists(args.visualizerBinaryPath):
    raise Exception('visualizer binary does not exist')

if not os.path.exists(args.datasetPath):
    raise Exception('dataset path does not exist')

# Open the log file.
if not os.path.exists(args.output):
    os.mkdir(args.output)
logFile = open(args.output + "/log.txt", 'w+')

# Run the dataset.
subprocess.call([args.visualizerBinaryPath, 
    "--headless", str(not args.visualize), 
    "--logTrackingData", 
    "--trackingLogPath",  args.output + "/trackingLog.json", 
    "--frames", str(args.frames),
    "--datasetPath", args.datasetPath], 
    stdout=logFile)

# Open the profiling file.
profilingFile = open(args.output + "/runtimes.json", 'w+')

# Extract the runtimes
runtimes = parseProfilingLog(args.output + "/log.txt")
# Write the runtimes to a json file.
json.dump(runtimes, profilingFile, indent=4, separators=(',', ': '))

# Open the tracking log json.
trackingLog = json.load(open(args.output + "/trackingLog.json"))  
