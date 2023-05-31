/*
  ==============================================================================

    PNPCommonEmitterCircuit.h
    Created: 31 May 2023 2:36:33pm
    Author:  James

  ==============================================================================
*/

#pragma once
#include "CircuitBase.h"
#include <Eigen/Dense>
#include "PNP_NLEQ.h"
#include "NPN_NLEQ.h"

class PNPCommonEmitterCircuit : public CircuitBase
{
public:
    /*Fill in circuit details in constructor*/
    PNPCommonEmitterCircuit();
private:
    std::unique_ptr<PNP_NLEQ> pnpNLEQ;
    std::unique_ptr<PNP_NLEQ> pnpNLEQ2;
};