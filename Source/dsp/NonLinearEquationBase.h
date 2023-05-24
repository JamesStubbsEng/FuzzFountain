/*
  ==============================================================================

    NonLinearEquationBase.h
    Created: 16 May 2023 1:39:22pm
    Author:  James

  ==============================================================================
*/

#pragma once

#include <Eigen/Dense>

class NonLinearEquationBase
{
public:

    virtual ~NonLinearEquationBase() = default;
    virtual double calculateCurrents(Eigen::MatrixXd voltages, int vn_index, int function_index) { return -666.0; }
    virtual void calculateJacobian(Eigen::MatrixXd* jacobian, Eigen::MatrixXd voltages, int vn_index) {}
    int getNumberOfFunctions() { return numberOfFunctions; };
protected:

    int numberOfFunctions;
};