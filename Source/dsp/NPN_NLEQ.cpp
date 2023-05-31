/*
  ==============================================================================

    NPN_NLEQ.cpp
    Created: 29 May 2023 11:53:05am
    Author:  James

  ==============================================================================
*/

#include "NPN_NLEQ.h"
#include "JuceHeader.h"

NPN_NLEQ::NPN_NLEQ()
{
    this->numberOfFunctions = 2;
}
double NPN_NLEQ::calculateCurrents(Eigen::MatrixXd voltages, int vn_index, int function_index)
{
    std::ostringstream outStream;
    outStream << "voltages: " << std::endl;
    outStream << voltages << std::endl;
    DBG(outStream.str());
    double returnVal = -1.0;
    if(function_index == 0)
        returnVal = Is*( 1/ Bf * (std::exp((voltages(vn_index + 1) - voltages(vn_index)) / Vt) - 1) + 1 / Br * (std::exp(-voltages(vn_index) / Vt) - 1));
    else
        returnVal = Is * (-(-(exp(-voltages(vn_index) / Vt) - 1) + (1 + Bf) / Bf * (exp((voltages(vn_index + 1) - voltages(vn_index)) / Vt) - 1)));
    DBG("returnVal: " + String(returnVal));
    return returnVal;
}

void NPN_NLEQ::calculateJacobian(Eigen::MatrixXd* jacobian, Eigen::MatrixXd voltages, int vn_index)
{
    (*jacobian)(vn_index, vn_index) = (Is / Vt) * (-1 / Bf * exp((voltages(vn_index + 1) - voltages(vn_index)) / Vt) - 1 / Br * exp(-voltages(vn_index) / Vt));
    (*jacobian)(vn_index, vn_index + 1) = (Is / Vt) * (1 / Bf * exp((voltages(vn_index + 1) - voltages(vn_index)) / Vt));
    (*jacobian)(vn_index + 1, vn_index) = (Is / Vt) * (-exp(-voltages(vn_index) / Vt) + (1 + Bf) / Bf * exp((voltages(vn_index + 1) - voltages(vn_index)) / Vt));
    (*jacobian)(vn_index + 1, vn_index + 1) = (Is / Vt) * (-(1 + Bf) / Bf * exp((voltages(vn_index + 1) - voltages(vn_index)) / Vt));
}