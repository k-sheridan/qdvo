import subprocess
import os
import argparse
import json
import csv
from termcolor import colored, cprint

from parseProfilingLog import parseProfilingLog
from metrics import computeMetrics

def evaluate(visualizerBinaryPath, datasetPath, visualize, frames, output):
    if not os.path.exists(visualizerBinaryPath):
        raise Exception('visualizer binary does not exist')
    
    if not os.path.exists(datasetPath):
        raise Exception('dataset path does not exist')
    
    groundTruthPath = os.path.join(datasetPath, "mav0/mocap0/data.csv")
    if not os.path.exists(groundTruthPath):
        raise Exception('Ground truth does not exist.')
    
    # Open the log file.
    if not os.path.exists(output):
        os.mkdir(output)
    logFile = open(os.path.join(output,"log.txt"), 'w+')
    
    # Run the dataset.
    print(colored('Running dataset: ', 'green', attrs=['bold']), colored(f"{datasetPath}", 'white', attrs=['bold']))
    subprocess.call([visualizerBinaryPath, 
        "--headless", str(not visualize), 
        "--logTrackingData", 
        "--trackingLogPath",  os.path.join(output, "trackingLog.json"), 
        "--frames", str(frames),
        "--datasetPath", datasetPath], 
        stdout=logFile)
    print(colored('Finished dataset.', 'green', attrs=['bold']))
    
    
    print(colored(f"Logging runtimes", 'green', attrs=['bold']))
    # Open the profiling file.
    profilingFile = open(os.path.join(output, "runtimes.json"), 'w+')
    
    # Extract the runtimes
    runtimes = parseProfilingLog(os.path.join(output, "log.txt"))
    # Write the runtimes to a json file.
    json.dump(runtimes, profilingFile, indent=4, separators=(',', ': '))
    print(colored(f"Finished logging {len(runtimes.keys())} runtimes", 'green', attrs=['bold']))
    
    print(colored(f"Loading tracking log.", 'green', attrs=['bold']))
    # Open the tracking log json.
    trackingLog = json.load(open(os.path.join(output, "trackingLog.json")))  
    
    print(colored(f"Loaded tracking log.", 'green', attrs=['bold']))
    print(colored(f"Loading Ground Truth.", 'green', attrs=['bold']))
    # Open the ground truth csv.
    groundTruthCSV = open(groundTruthPath)
    print(colored(f"Loaded Ground Truth.", 'green', attrs=['bold']))
    
    # Compute metrics.
    print(colored('Computing metrics', 'green', attrs=['bold']))
    metrics = computeMetrics(trackingLog, groundTruthCSV)
    print(colored('Finished computing metrics', 'green', attrs=['bold']))
    
    # Write the metrics to a file
    metricsFile = open(os.path.join(output, "metrics.json"), 'w+')
    json.dump(metrics, metricsFile, indent=4, separators=(',', ': '))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluate a euroc format dataset using QDVO.',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--datasetPath', type=str, required=True, help='Path to the folder above mav0 of the dataset.')
    parser.add_argument('--visualizerBinaryPath', type=str, default='../build/visualizer/runEurocDatasets', help='Path to the binary used to run QDVO.')
    parser.add_argument('--visualize', type=bool, default=False, help='Should the dataset be visualized while running?')
    parser.add_argument('--frames', type=int, default=1e12, help='How many frames should the dataset be run for?')
    parser.add_argument('--output', type=str, required=True, help='Path to the folder where the results will be output.')
    
    args = parser.parse_args()

    evaluate(args.visualizerBinaryPath, args.datasetPath, args.visualize, args.frames, args.output)
