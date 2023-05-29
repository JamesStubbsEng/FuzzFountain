/*
  ==============================================================================

    NPN_NLEQ.h
    Created: 29 May 2023 11:53:05am
    Author:  James

  ==============================================================================
*/

#pragma once
#include "NonLinearEquationBase.h"

class NPN_NLEQ : public NonLinearEquationBase
{
public:
    NPN_NLEQ();
    double calculateCurrents(Eigen::MatrixXd voltages, int vn_index, int function_index) override;
    void calculateJacobian(Eigen::MatrixXd* jacobian, Eigen::MatrixXd voltages, int vn_index) override;

private:
    double Is = 1.16e-14;
    double Bf = 200;
    double Br = 3;
    double Vt = 0.02528;
};