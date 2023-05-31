/*
  ==============================================================================

    FuzzFaceCircuit.h
    Created: 31 May 2023 12:29:28pm
    Author:  James

  ==============================================================================
*/

#pragma once
#include "CircuitBase.h"
#include <Eigen/Dense>
#include "PNP_NLEQ.h"
#include "NPN_NLEQ.h"

class FuzzFaceCircuit : public CircuitBase
{
public:
    /*Fill in circuit details in constructor*/
    FuzzFaceCircuit();
private:
    std::unique_ptr<PNP_NLEQ> pnpNLEQ;
    std::unique_ptr<PNP_NLEQ> pnpNLEQ2;
    std::unique_ptr<NPN_NLEQ> npnNLEQ;
    std::unique_ptr<NPN_NLEQ> npnNLEQ2;
};