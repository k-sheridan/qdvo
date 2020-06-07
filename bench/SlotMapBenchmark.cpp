#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include "Optimizer/SlotMap.h"

using namespace ArgMin;

static void BM_SlotMapInsert(benchmark::State &state) {
  SlotMap<double, SlotMapKeyBase> map;

  for (auto _ : state) {
    map.insert(1.0);
  }
}

static void BM_SlotMapErase(benchmark::State &state) {
  SlotMap<double, SlotMapKeyBase> map;
  std::vector<SlotMapKeyBase> keys;
  for (int i = 0; i < state.max_iterations; ++i) {
    keys.push_back(map.insert(1.0));
  }

  for (auto _ : state) {
    map.erase(keys.at(state.iterations()));
  }
}

static void BM_SlotMapAt(benchmark::State &state) {
  SlotMap<double, SlotMapKeyBase> map;
  std::vector<SlotMapKeyBase> keys;
  for (int i = 0; i < state.max_iterations; ++i) {
    keys.push_back(map.insert(1.0));
  }

  for (auto _ : state) {
    map.at(keys.at(state.iterations()));
  }
}

static void BM_VectorIterate(benchmark::State &state) {
  struct Data {
    float num1 = 1.9;
    float num2 = 5.0;
    float sum = 0.0;
    std::array<double, 11 * 11> waste;
  };
  std::vector<Data> data;
  data.resize(1e3);

  for (auto _ : state) {
    for (auto &d : data) {
      d.sum = d.num1 * d.num2;
    }
  }
}

static void BM_VectorTransformFloat(benchmark::State &state) {
  std::vector<float> num1, num2, sum;
  std::vector<std::array<double, 11 * 11>> waste;
  num1.resize(1e3, 1.9);
  num2.resize(1e3, 5.0);
  sum.resize(1e3, 0.0);
  waste.resize(1e3);

  for (auto _ : state) {
    std::transform(num1.begin(), num1.end(), num2.begin(), sum.begin(),
                   [](float a, float b) { return a * b; });
  }
}

BENCHMARK(BM_SlotMapInsert)->Unit(benchmark::kNanosecond)->Iterations(100000);
BENCHMARK(BM_SlotMapErase)->Unit(benchmark::kNanosecond)->Iterations(100000);
BENCHMARK(BM_SlotMapAt)->Unit(benchmark::kNanosecond)->Iterations(100000);

BENCHMARK(BM_VectorIterate)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_VectorTransformFloat)->Unit(benchmark::kNanosecond);
