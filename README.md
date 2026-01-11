# [QDVO: Quasi-Direct Visual Odometry](https://youtu.be/pYeO0l5dH5Q)

[![Presentation](https://img.shields.io/badge/Purdue_AAE_RSS-Presentation-blue)](QDVO_presentation.pdf)
![](https://github.com/k-sheridan/qdvo/workflows/C/C++%20CI/badge.svg)

QDVO is a real-time monocular visual odometry algorithm which combines the benefits of a indirect and direct methods. It does this by modelling point correspondences with a non-gaussian *correspondence distribution*. By doing this, QDVO is able to use both corner and edge landmarks which overcoming the sensistivity to illumination changes often seen with direct (photometric) methods.

This work was presented at [Purdue's AAE Research Symposium Series in 2019](https://engineering.purdue.edu/AAE/academics/studentorgs/aaerss/past_presenters/index_html) and won "Best Undergraduate Presentation". You can find the slides for this presentation [here](https://github.com/k-sheridan/qdvo/blob/master/QDVO_presentation.pdf). 

## Demo Videos
[![TUM VI Demo](http://img.youtube.com/vi/TU5lfFeBUuo/0.jpg)](https://youtu.be/TU5lfFeBUuo)
[![Euroc V101 Demo](http://img.youtube.com/vi/5oMmafv2QXk/0.jpg)](https://youtu.be/5oMmafv2QXk)

# A Note on Correspondence Distributions

The correspondence distributions are represented with gaussian mixture models. In QDVO, the similarity score used is ZNCC. To ensure convergence far from the optimial solution, QDVO filters low similarity pixels in the correspondence distribution.
<img width="950" height="536" alt="correspondence_distribution" src="https://github.com/user-attachments/assets/0e6de5ba-1b13-4a2d-a1ca-65095fa17b96" />

## Quasi-Direct Residual
To ensure we can efficiently minimimize the negative log likelihood, QDVO uses *Iteratively Reweighted Non-Linear Least Squares* optimization. By doing this, QDVO is able to practically bound the compute per residual by exploiting the exponential fall-off in the influence of far away gaussians in the GMM.
<img width="953" height="637" alt="Quasi-Direct_Residual_1" src="https://github.com/user-attachments/assets/a101fa79-1840-41cd-9ff6-195f34577c2f" />
<img width="949" height="714" alt="Quasi-Direct_Residual_2" src="https://github.com/user-attachments/assets/b881e9eb-01a2-44d5-aefa-268b37ebac5d" />

# ArgMin: a semi-generic, fully templated optimizer designed for Sliding Window Estimators

The CPP implementation of QDVO is based on a custom optimizer with the following capabilities:
- Supports arbitrary manifold variables
  - Built in support for SE3, SO3, InverseDepth, and Scalar
- Supports arbitrary error terms
- Built in support for marginalization of error terms and variables into a gaussian prior
- Based on a Sparse Schur Solver implementation to exploit sparsity in landmark variables.
- Built on top of [SlotMap and Slot Array](docs/source/SlotMap.md) implementation for cache friendly access of sub-matrices

## ArgMin: Examples
The most complete example on how to use ArgMin is in this test: [TestArgMinExampleProblem.cpp](https://github.com/k-sheridan/qdvo/blob/master/test/TestArgMinExampleProblem.cpp)

## ArgMin: Documentation
The most complete documentation of ArgMin's SSEOptimizer can be found [here](docs/source/SSEOptimizer.md)

## Running QDVO

### Running Tests
To build and run QDVO's unit tests and benchmarks, see the [test README](test/README.md) for detailed Docker-based instructions.

### Running Evaluation
To evaluate QDVO on EuRoC format datasets and compute trajectory accuracy metrics, see the [tools README](tools/README.md) for evaluation scripts and usage instructions.

