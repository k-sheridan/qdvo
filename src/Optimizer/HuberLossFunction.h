#pragma once

/**
 * The huber loss function is used for robust regression.
 * 
 * More information can be found here: https://en.wikipedia.org/wiki/Huber_loss
 * 
 * This class simply defines a loss function which 
 */
template <typename ScalarType>
class HuberLossFunction
{
public:

    HuberLossFunction(ScalarType c) c(c) {} 

    ScalarType c;

    ScalarType loss(ScalarType errorNorm, ScalarType errorNormSquared)
    {
    }
};