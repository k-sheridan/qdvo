#include <benchmark/benchmark.h>
#include <enoki/array.h>
#include <enoki/dynamic.h>
#include <enoki/stl.h>

#include <algorithm>
#include <array>

#include "Optimizer/Containers/soa/soa_vector.h"
#include "Optimizer/SlotMap.h"

using namespace ArgMin;

template <bool Direct>
static void BM_SlotMapInsert(benchmark::State &state) {
  SlotMap<double, SlotMapKeyBase, std::vector<double>, Direct> map;

  for (auto _ : state) {
    map.insert(1.0);
  }
}

template <bool Direct>
static void BM_SlotMapErase(benchmark::State &state) {
  SlotMap<double, SlotMapKeyBase, std::vector<double>, Direct> map;
  std::vector<SlotMapKeyBase> keys;
  for (int i = 0; i < state.max_iterations; ++i) {
    keys.push_back(map.insert(1.0));
  }

  for (auto _ : state) {
    map.erase(keys.at(state.iterations()));
  }
}

template <bool Direct>
static void BM_SlotMapAt(benchmark::State &state) {
  SlotMap<double, SlotMapKeyBase, std::vector<double>, Direct> map;
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

struct MyData {
  float num1 = 1.9;
  float num2 = 5.0;
  float sum = 0.0;
  std::array<double, 11 * 11> waste;
};
SOA_DEFINE_TYPE(MyData, num1, num2, sum, waste);

static void BM_SOAVectorTransformFloat(benchmark::State &state) {
  soa::vector<MyData> vec;
  vec.resize(1e3);

  for (auto _ : state) {
    for (auto e : vec) {
      e.sum = e.num1 * e.num2;
    }
  }
}

struct A {
  float num1 = 1.9;
  float num2 = 5.0;
  float sum = 0.0;
};
struct B {
  std::array<double, 11 * 11> waste;
};
struct MyDataCombined : public A, public B {};
SOA_DEFINE_TYPE(MyDataCombined, num1, num2, sum, waste);

static void BM_SOAVectorTransformFloatWithInheritance(benchmark::State &state) {
  soa::vector<MyDataCombined> vec;
  vec.resize(1e3);

  for (auto _ : state) {
    for (auto &&e : vec) {
      e.sum = e.num2 * e.num1;
    }
  }
}

template <typename Value>
struct EnokiData {
  using FloatA = enoki::float32_array_t<Value>;
  using WasteA = enoki::replace_scalar_t<Value, enoki::Array<double, 11 * 11>>;

  FloatA num1 = 1.9;
  FloatA num2 = 5.0;
  FloatA sum = 0.0;
  WasteA waste;

  void fn() { sum = num1 * num2; }

  ENOKI_STRUCT(EnokiData, num1, num2, sum, waste)
};
ENOKI_STRUCT_SUPPORT(EnokiData, num1, num2, sum, waste)

static void BM_EnokiSOA(benchmark::State &state) {
  using Pack = enoki::Packet<double>;
  EnokiData<enoki::DynamicArray<Pack>> dynamicData;
  enoki::set_slices(dynamicData, 1e3);

  for (auto _ : state) {
    enoki::vectorize(
        [](auto &&num1, auto &&num2, auto &&sum) { sum = num2 * num1; },
        dynamicData.num1, dynamicData.num2, dynamicData.sum);
  }
}

static void BM_EnokiSOAManual(benchmark::State &state) {
  using Pack = enoki::Packet<double>;
  EnokiData<enoki::DynamicArray<Pack>> dynamicData;
  enoki::set_slices(dynamicData, 1e3);

  for (auto _ : state) {
    for (int i = 0; i < enoki::slices(dynamicData); ++i) {
      auto &&s = enoki::slice(dynamicData, i);
      s.sum = s.num1 * s.num2;
    }
  }
}

constexpr bool Direct = false;
constexpr bool Contiguous = true;
BENCHMARK_TEMPLATE(BM_SlotMapInsert, Direct)
    ->Unit(benchmark::kNanosecond)
    ->Iterations(1000000);
BENCHMARK_TEMPLATE(BM_SlotMapErase, Direct)
    ->Unit(benchmark::kNanosecond)
    ->Iterations(1000000);
BENCHMARK_TEMPLATE(BM_SlotMapAt, Direct)
    ->Unit(benchmark::kNanosecond)
    ->Iterations(1000000);
BENCHMARK_TEMPLATE(BM_SlotMapInsert, Contiguous)
    ->Unit(benchmark::kNanosecond)
    ->Iterations(1000000);
BENCHMARK_TEMPLATE(BM_SlotMapErase, Contiguous)
    ->Unit(benchmark::kNanosecond)
    ->Iterations(1000000);
BENCHMARK_TEMPLATE(BM_SlotMapAt, Contiguous)
    ->Unit(benchmark::kNanosecond)
    ->Iterations(1000000);

BENCHMARK(BM_VectorIterate)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_VectorTransformFloat)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_SOAVectorTransformFloat)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_SOAVectorTransformFloatWithInheritance)
    ->Unit(benchmark::kNanosecond);
BENCHMARK(BM_EnokiSOA)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_EnokiSOAManual)->Unit(benchmark::kNanosecond);
