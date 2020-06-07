import os
import subprocess
import argparse
from evaluateDataset import evaluate


def get_git_revision_hash():
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'])


def build_dataset_player():
    os.system('(cd ../build && make -j runEurocDatasets)')


if __name__ == "__main__":
    currentHash = get_git_revision_hash()

    parser = argparse.ArgumentParser(
        description='Evaluate a euroc format dataset using QDVO.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--datasetPath',
                        type=str,
                        required=True,
                        help='Path to the folder above mav0 of the dataset.')
    parser.add_argument('--visualizerBinaryPath',
                        type=str,
                        default='../build/visualizer/runEurocDatasets',
                        help='Path to the binary used to run QDVO.')
    parser.add_argument('--visualize',
                        type=bool,
                        default=False,
                        help='Should the dataset be visualized while running?')
    parser.add_argument('--frames',
                        type=int,
                        default=1e12,
                        help='How many frames should the dataset be run for?')
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help='Path to the folder where the results will be output.')

    args = parser.parse_args()

    basePath = os.path.join(args.output, 'base')

    build_dataset_player()
    evaluate(args.visualizerBinaryPath, args.datasetPath, args.visualize,
             args.frames, basePath)

    subprocess.call(['git', 'checkout', 'HEAD~1'])

    comparePath = os.path.join(args.output, 'change')

    build_dataset_player()
    evaluate(args.visualizerBinaryPath, args.datasetPath, args.visualize,
             args.frames, comparePath)

    subprocess.call(['git', 'checkout', currentHash])

    subprocess.call([
        'diff',
        os.path.join(basePath, 'metrics.json'),
        os.path.join(comparePath, 'metrics.json')
    ])
