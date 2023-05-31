/*
  ==============================================================================

    PNP_NLEQ.h
    Created: 31 May 2023 12:29:11pm
    Author:  James

  ==============================================================================
*/

#pragma once
#include "NonLinearEquationBase.h"

class PNP_NLEQ : public NonLinearEquationBase
{
public:
    /*Vn1 = V_BC, Vn2 = V_EC*/
    PNP_NLEQ(double Is, double Bf, double Br);
    double calculateCurrents(Eigen::MatrixXd voltages, int vn_index, int function_index) override;
    void calculateJacobian(Eigen::MatrixXd* jacobian, Eigen::MatrixXd voltages, int vn_index) override;

private:
    double Is = 85.8e-9;
    double Bf = 85;
    double Br = 20;
    double Vt = 0.02528;
};