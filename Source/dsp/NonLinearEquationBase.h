/*
  ==============================================================================

    NonLinearEquationBase.h
    Created: 16 May 2023 1:39:22pm
    Author:  James

  ==============================================================================
*/

#pragma once

#include <Eigen/Core>

class NonLinearEquationBase
{
public:

    virtual ~NonLinearEquationBase() = default;
    virtual float calculateCurrents(Eigen::MatrixXd voltages, int vn_index, int function_index) = 0;
    virtual void calculateJacobian(Eigen::MatrixXd * jacobian, Eigen::MatrixXd voltages, int vn_index) = 0;
    int getNumberOfFunctions() { return numberOfFunctions; };
protected:

    int numberOfFunctions;
};