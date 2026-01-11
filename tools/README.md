# QDVO Evaluation Tools

This directory contains Python scripts for evaluating QDVO performance on EuRoC format datasets. The main script `evaluateDataset.py` runs QDVO on a dataset, extracts profiling data, and computes trajectory accuracy metrics.

## Requirements
- Python 3 with packages: `numpy`, `scipy`, `termcolor`, `sophuspy`
- QDVO visualizer binary (`runEurocDatasets`)
- A dataset in EuRoC format with ground truth

## Running Evaluation with Docker

### 1. Start Docker Container From QDVO Root

```bash
docker run -it --platform linux/amd64 -v "$(pwd):/workspace" -w /workspace ubuntu:latest bash
```

### 2. Install System Dependencies

```bash
apt-get update -qq
apt-get install -y \
  clang \
  cmake \
  git \
  libeigen3-dev \
  libopencv-dev \
  libboost-dev \
  libboost-filesystem-dev \
  libgl1-mesa-dev \
  libglew-dev \
  libepoxy-dev \
  python3 \
  python3-pip
```

### 3. Build and Install Pangolin

```bash
git clone https://github.com/stevenlovegrove/Pangolin
cmake -S Pangolin -B Pangolin/build
cmake --build Pangolin/build 
cmake --install Pangolin/build
```

### 4. Configure and Build QDVO

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_C_COMPILER=clang \
  -DCMAKE_CXX_COMPILER=clang++

cmake --build build
```

### 5. Install Python Dependencies

```bash
# Install basic Python packages
pip3 install --break-system-packages numpy scipy termcolor

# Build and install sophuspy from source with explicit compiler settings
git clone https://github.com/craigstar/SophusPy
cd SophusPy
export CC=clang
export CXX=clang++
pip3 install --break-system-packages .
cd ..
```

**Note:** Setting `CC` and `CXX` to use clang ensures sophuspy is compiled with the same compiler as QDVO, preventing ABI compatibility issues that can cause segmentation faults.

### 6. Run Evaluation

```bash
python3 tools/evaluateDataset.py \
  --datasetPath test/qdvo-test-datasets/dataset-room1_512_16_chopped/ \
  --visualizerBinaryPath build/visualizer/runEurocDatasets \
  --frames 1000 \
  --output ./evaluation_results
```

## Output Files

The evaluation generates the following files in the output directory:

- **log.txt** - Raw stdout from the QDVO visualizer
- **trackingLog.json** - Frame-by-frame tracking data (poses, landmarks, status)
- **runtimes.json** - Profiling percentiles (p10, p50, p90, p99, mean) for each timed function
- **profile.trace** - Chrome tracing format timeline for visualization
- **metrics.json** - Trajectory accuracy metrics including:
  - Position/rotation RMSE vs ground truth
  - Scale error
  - Per-frame odometry errors
  - Estimator convergence metrics
  - Tracking status statistics

## Command Line Arguments

- `--datasetPath` - Path to the folder containing `mav0/` directory
- `--visualizerBinaryPath` - Path to the `runEurocDatasets` binary (default: `../build/visualizer/runEurocDatasets`)
- `--frames` - Maximum number of frames to process (default: 1e12)
- `--output` - Output directory for results (required)
- `--visualize` - Enable visualization during run (default: False)