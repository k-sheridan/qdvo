swe = SlidingWindowEstimator(3);
swe.optimizer.numberOfErrorTerms()
swe.initializeGyroOnly(vio.graph);
swe.optimizer.numberOfErrorTerms()

