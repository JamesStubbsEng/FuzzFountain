/*
  ==============================================================================

    DiodeNLEQ.h
    Created: 16 May 2023 1:39:48pm
    Author:  James

  ==============================================================================
*/

#pragma once

#include "NonLinearEquationBase.h"

class DiodeNLEQ : public NonLinearEquationBase
{
public:
    DiodeNLEQ();
    double calculateCurrents(Eigen::MatrixXd voltages, int vn_index, int function_index) override;
    void calculateJacobian(Eigen::MatrixXd* jacobian, Eigen::MatrixXd voltages, int vn_index) override;

private:
    double Is = 1e-15;
    double Vt = 25.85e-3;
    double eta = 1;
};