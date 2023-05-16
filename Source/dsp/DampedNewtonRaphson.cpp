/*
  ==============================================================================

    DampedNewtonRaphson.cpp
    Created: 16 May 2023 1:26:20pm
    Author:  James

  ==============================================================================
*/

#include "DampedNewtonRaphson.h"

DampedNewtonRaphson::DampedNewtonRaphson(int numberOfNonLinearFunctions, Eigen::MatrixXd K, std::unique_ptr<std::vector<NonLinearEquationBase*>> nonlinearComponents)
{
    //pass parameters to members
    numberOfNonLinearFunctions = numberOfNonLinearFunctions;
    K = K;
    nonLinearComponents = std::move(nonlinearComponents);
    numComponents = nonLinearComponents->size();

    //initialize member matrices with zeros
    vn_new = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, 1);
    in = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, 1);
    in_new = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, 1);
    eye = Eigen::MatrixXd::Identity(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    componentsJacobian = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    J = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    F = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    F_new = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
}

void DampedNewtonRaphson::solve(Eigen::MatrixXd* vn, Eigen::MatrixXd* p)
{
    step = Eigen::MatrixXd::Ones(numberOfNonLinearFunctions, 1);
    iter = 0;

    //get first in
    getCurrents(vn, p, &in);

    // get first F
    F = *p + K * in - *vn;

    //iterate to solve system of non linear equations
    while (step.norm() > tol && iter < maxIterations)
    {
        getCurrents(vn, p, &in);
        J = K * componentsJacobian - eye;

        step.noalias() = J.inverse() * F;
        vn_new = *vn - b * step;
        getCurrents(&vn_new, p, &in_new);

        F_new = *p + K * in_new - vn_new;
        if (F_new.norm() < F.norm())
        {
            F = F_new;
            *vn = vn_new;
            b = 1;
        }
        else
        {
            b /= 2;
        }
        iter += 1;
    }

    //Something is wrong!
    jassert(iter < 100);
}

void DampedNewtonRaphson::getCurrents(Eigen::MatrixXd* vn, Eigen::MatrixXd* p, Eigen::MatrixXd* in)
{
    vn_index = 0;
    for (int componentIndex = 0; componentIndex < numComponents; componentIndex++)
    {
        numberOfFunctions = nonLinearComponents->at(componentIndex)->getNumberOfFunctions();
        nonLinearComponents->at(componentIndex)->calculateJacobian(&componentsJacobian, *vn, vn_index);
        for (int functionIndex = 0; functionIndex < numberOfFunctions; functionIndex++)
        {
            (*in)(functionIndex + vn_index) = nonLinearComponents->at(componentIndex)->calculateCurrents(*vn, vn_index, functionIndex);
        }
        vn_index += numberOfFunctions;
    }
}
