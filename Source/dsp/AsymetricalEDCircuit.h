/*
  ==============================================================================

    AsymetricalEDCircuit.h
    Created: 29 May 2023 11:53:51am
    Author:  James

  ==============================================================================
*/

#pragma once
#include "CircuitBase.h"
#include <Eigen/Dense>
#include "NPN_NLEQ.h"

class AsymetricalEDCircuit : public CircuitBase
{
public:
    /*Fill in circuit details in constructor*/
    AsymetricalEDCircuit();
private:
    std::unique_ptr<NPN_NLEQ> npnNLEQ;
};