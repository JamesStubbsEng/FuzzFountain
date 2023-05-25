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

class DiodeClipperCircuit : public CircuitBase
{
public:
    /*Fill in circuit details in constructor*/
    DiodeClipperCircuit();
private:
    std::unique_ptr<DiodeNLEQ> diodeNLEQ;
};