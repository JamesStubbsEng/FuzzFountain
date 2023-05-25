/*
  ==============================================================================

    DiodeClipperCircuit.h
    Created: 16 May 2023 3:22:08pm
    Author:  James

  ==============================================================================
*/

#pragma once

#include "CircuitBase.h"
#include <Eigen/Dense>
#include "CircuitBase.h"
#include "DiodeNLEQ.h"

class DiodeClipperCircuit
{
public:
    DiodeClipperCircuit();
    void prepare(float sampleRate);
    void process(float* block, const int numSamples) noexcept;
    void reset();
private:

    std::unique_ptr<CircuitBase> rcDiodeClipper;
    std::unique_ptr<DiodeNLEQ> diodeNLEQ;
};