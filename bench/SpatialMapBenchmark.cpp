#include <benchmark/benchmark.h>
#include "DataStructures/SpatialMap.h"
#include <random>

static void BM_SpatialMapInsert512X512(benchmark::State &state)
{
    struct RandomType
    {
        double *data = nullptr;

        void reset()
        {
        }
    };

    QDVO::SpatialMap<RandomType> map(512);

    for (auto _ : state)
    {
        for (int x = 0; x < 512; ++x)
        {
            for (int y = 0; y < 512; ++y)
            {
                map.get(Eigen::Vector2i(x, y));
            }
        }
    }
}

static void BM_SpatialMapGetOneFrom512X512(benchmark::State &state)
{
    struct RandomType
    {
        double *data = nullptr;

        void reset()
        {
        }
    };

    QDVO::SpatialMap<RandomType> map(512);
    // allocate memory for the map
    for (int x = 0; x < 512; ++x)
    {
        for (int y = 0; y < 512; ++y)
        {
            map.get(Eigen::Vector2i(x, y));
        }
    }

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(0, 511);

    // random select a pixel.
    Eigen::Vector2i px(dis(gen), dis(gen));

    for (auto _ : state)
    {
        map.get(px);
    }
}

BENCHMARK(BM_SpatialMapInsert512X512)->Unit(benchmark::kMillisecond)->Iterations(1);
BENCHMARK(BM_SpatialMapGetOneFrom512X512)->Unit(benchmark::kNanosecond)->Iterations(1000);