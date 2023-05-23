/*
  ==============================================================================

    DiodeNLEQ.cpp
    Created: 16 May 2023 1:39:48pm
    Author:  James

  ==============================================================================
*/

#include "DiodeNLEQ.h"

DiodeNLEQ::DiodeNLEQ()
{
    numberOfFunctions = 1;
}
float DiodeNLEQ::calculateCurrents(Eigen::MatrixXd voltages, int vn_index, int function_index)
{
    return -2*Is * (std::sinh(voltages(vn_index) / (eta*Vt)));
}

void DiodeNLEQ::calculateJacobian(Eigen::MatrixXd* jacobian, Eigen::MatrixXd voltages, int vn_index)
{
    (*jacobian)(vn_index, vn_index) = -2 * Is / (eta * Vt) * std::cosh(voltages(vn_index) / (eta * Vt));
}
