swe = SlidingWindowEstimator(3);
swe.optimizer.numberOfErrorTerms()
swe.initializeGyroOnly(vio.graph);
swe.optimizer.numberOfErrorTerms()
swe.frameIdsToOptimize
vio.graph = swe.optimize(vio.graph)



