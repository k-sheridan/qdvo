# Test dataset used in unit tests
The dataset used for integration tests consists of a precalibrated global shutter stereo camera pair with a 200 hz IMU.
To reduce the overall size of this dataset, it was chopped to 1000 512X512 image pairs.
## The TUM VI Benchmark for Evaluating Visual-Inertial Odometry
(D. Schubert, T. Goll, N. Demmel, V. Usenko, J. Stueckler and D. Cremers), In International Conference on Intelligent Robots and Systems (IROS), 2018.

## Running Tests and Benchmarks with Docker

### 1. Start Docker Container From QDVO Root

```bash
docker run -it --platform linux/amd64 -v "$(pwd):/workspace" -w /workspace ubuntu:latest bash
```

### 2. Install Dependencies

```bash
apt-get update -qq
apt-get install -y \
  clang \
  cmake \
  git \
  ccache \
  gfortran \
  libc++-dev \
  libgoogle-glog-dev \
  libatlas-base-dev \
  libsuitesparse-dev \
  libeigen3-dev \
  libopencv-dev \
  libboost-dev \
  libboost-filesystem-dev \
  libbenchmark-dev \
  libgl1-mesa-dev \
  libglew-dev \
  libepoxy-dev
```

### 3. Configure

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_C_COMPILER=clang \
  -DCMAKE_CXX_COMPILER=clang++
```

### 4. Build

```bash
cmake --build build -j --target test_qdvo
cmake --build build -j --target benchmark_qdvo
```

### 5. Test

```bash
(cd build && ./test/test_qdvo)
```

### 6. Benchmark

```bash
(cd build && ./bench/benchmark_qdvo)
```
