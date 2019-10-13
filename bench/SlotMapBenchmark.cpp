#include <benchmark/benchmark.h>
#include "SlotMap.h"

static void BM_SlotMapInsert(benchmark::State &state)
{
    SlotMap<double, SlotMapKeyBase> map;

    for(auto _ : state)
    {
        map.insert(1.0);
    }

}

static void BM_SlotMapErase(benchmark::State &state)
{
    SlotMap<double, SlotMapKeyBase> map;
    std::vector<SlotMapKeyBase> keys;
    for (int i = 0; i < state.max_iterations; ++i)
    {
        keys.push_back(map.insert(1.0));
    }

    for(auto _ : state)
    {
        map.erase(keys.at(state.iterations()));
    }

}

static void BM_SlotMapAt(benchmark::State &state)
{
    SlotMap<double, SlotMapKeyBase> map;
    std::vector<SlotMapKeyBase> keys;
    for (int i = 0; i < state.max_iterations; ++i)
    {
        keys.push_back(map.insert(1.0));
    }

    for(auto _ : state)
    {
        map.at(keys.at(state.iterations()));
    }

}

BENCHMARK(BM_SlotMapInsert)->Unit(benchmark::kNanosecond)->Iterations(100000);
BENCHMARK(BM_SlotMapErase)->Unit(benchmark::kNanosecond)->Iterations(100000);
BENCHMARK(BM_SlotMapAt)->Unit(benchmark::kNanosecond)->Iterations(100000);