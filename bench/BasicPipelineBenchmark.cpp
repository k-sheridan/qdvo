#include <benchmark/benchmark.h>

#include "BenchmarkHelpers.h"
#include "Pipeline/BasicPipeline.h"

static QDVO::BasicPipeline constructPipeline() {
  QDVO::BasicPipeline pipeline;
  auto cmKey = pipeline.addCamera(getTestCameraModel());
  auto extrinsicKey = pipeline.graph.getExtrinsicMap().insert(
      pipeline.graph.getCameraModelMap().at(cmKey)->second);

  pipeline.initialize();

  auto img = getTestImage();
  pipeline.addFrame(img, 0, cmKey, extrinsicKey);
  pipeline.addFrame(img, 1, cmKey, extrinsicKey);

  return pipeline;
}

static void BM_InitializeCorrespondenceDistribution(benchmark::State &state) {
  static auto pipeline = constructPipeline();

  for (auto _ : state) {
    pipeline.initializeCorrespondenceDistributionsForCurrentFrame();
  }
}
BENCHMARK(BM_InitializeCorrespondenceDistribution)
    ->Unit(benchmark::kMillisecond);

static void BM_FrontEndVisualOdometry(benchmark::State &state) {
  static auto pipeline = constructPipeline();

  for (auto _ : state) {
    pipeline.frontEndVisualOdometry.run(pipeline.graph);
  }
}
BENCHMARK(BM_FrontEndVisualOdometry)->Unit(benchmark::kMillisecond);

static void BM_LinearizeQuasiDirectErrorTerms(benchmark::State &state) {
  static auto pipeline = constructPipeline();

  for (auto _ : state) {
    pipeline.frontEndVisualOdometry.solver.linearize(
        pipeline.frontEndVisualOdometry.variableContainer,
        pipeline.frontEndVisualOdometry.errorTermContainer);
  }
}
BENCHMARK(BM_LinearizeQuasiDirectErrorTerms)->Unit(benchmark::kMillisecond);
