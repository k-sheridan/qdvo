import subprocess
import json
import os
from termcolor import colored, cprint

# A set of datasets to evaluate.
# [datasetPath, numberOfFrames]
datasets = [['../test/qdvo-test-datasets/dataset-room1_512_16_chopped/', 1000],
            ['/mnt/repositories/tumvi-datasets/dataset-room6_512_16/', 2600],
            ['/mnt/repositories/tumvi-datasets/dataset-room2_512_16/', 2600]]

referenceFolder = os.path.expanduser('~/metrics_temp/reference')
outputFolder = os.path.expanduser('~/metrics_temp/evaluation')
processes = []

print(colored(f"Dispatching dataset evaluations", 'green', attrs=['bold']))

for datasetPath, frameCount in datasets:
    print(colored(f"Dispatching {datasetPath}", 'white', attrs=['bold']))
    outputPath = os.path.join(outputFolder,
                              os.path.basename(os.path.relpath(datasetPath)))
    print(colored(f"Output folder: {outputPath}", 'white', attrs=['bold']))

    processes.append(
        subprocess.Popen([
            'python3', 'evaluateDataset.py', '--datasetPath', datasetPath,
            '--frames',
            str(frameCount), '--output', outputPath
        ],
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT))

print(colored(f"Evaluations dispatched...", 'green', attrs=['bold']))

for p in processes:
    p.wait()

print(colored(f"Finished evaluating datasets", 'green', attrs=['bold']))

# Compare top line metrics
for datasetPath, frameCount in datasets:
    evaluationPath = os.path.join(
        outputFolder, os.path.basename(os.path.relpath(datasetPath)))

    referencePath = os.path.join(
        referenceFolder, os.path.basename(os.path.relpath(datasetPath)))

    bMetrics = json.load(open(os.path.join(evaluationPath, 'metrics.json')))
    aMetrics = json.load(open(os.path.join(referencePath, 'metrics.json')))

    print(colored(f"Dataset: {datasetPath}", 'white', attrs=['bold']))
    print(
        colored(
            f"Reference pos_rsme: {aMetrics['trajectory_rsme']['position_m']} Evaluation pos_rsme: {bMetrics['trajectory_rsme']['position_m']}",
            'white',
            attrs=['bold']))
    print(
        colored(
            f"Reference rotation_rsme: {aMetrics['trajectory_rsme']['rotation_rad']} Evaluation rotation_rsme: {bMetrics['trajectory_rsme']['rotation_rad']}",
            'white',
            attrs=['bold']))
