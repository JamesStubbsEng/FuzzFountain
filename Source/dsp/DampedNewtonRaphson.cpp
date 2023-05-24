/*
  ==============================================================================

    DampedNewtonRaphson.cpp
    Created: 16 May 2023 1:26:20pm
    Author:  James

  ==============================================================================
*/

#include "DampedNewtonRaphson.h"

DampedNewtonRaphson::DampedNewtonRaphson(int numberOfNonLinearFunctions, Eigen::MatrixXd* K, std::vector<NonLinearEquationBase*>* nonLinearComponents)
{
    //pass parameters to members
    numberOfNonLinearFunctions = numberOfNonLinearFunctions;
    this->K = K;
    this->nonLinearComponents = nonLinearComponents;
    this->numComponents = nonLinearComponents->size();

    //initialize member matrices with zeros
    this->vn_new = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, 1);
    this->in_new = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, 1);
    this->eye = Eigen::MatrixXd::Identity(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    this->componentsJacobian = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    this->J = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    this->F = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
    this->F_new = Eigen::MatrixXd::Zero(numberOfNonLinearFunctions, numberOfNonLinearFunctions);
}

void DampedNewtonRaphson::solve(Eigen::MatrixXd* vn, Eigen::MatrixXd* in, Eigen::MatrixXd* p)
{
    std::ostringstream outStream;

    step = Eigen::MatrixXd::Ones(numberOfNonLinearFunctions, 1);
    iter = 0;

    outStream << "in: " << std::endl;
    outStream << *in << std::endl;
    //get first in
    getCurrents(vn, in);

    // get first F
    F = *p + (*K) * (*in) - (*vn);

    outStream << "vn: " << std::endl;
    outStream << *vn << std::endl;
    outStream << "in: " << std::endl;
    outStream << *in << std::endl;
    outStream << "p: " << std::endl;
    outStream << *p << std::endl;
    outStream << "K: " << std::endl;
    outStream << *K << std::endl;
    outStream << "F: " << std::endl;
    outStream << F << std::endl;
    

    //iterate to solve system of non linear equations
    while (step.norm() > tol && iter < maxIterations)
    {
        getCurrents(vn, in);
        J = (*K) * componentsJacobian - eye;

        step.noalias() = J.inverse() * F;
        vn_new = (*vn) - b * step;
        getCurrents(&vn_new, &in_new);

        F_new = *p + (*K) * in_new - vn_new;
        outStream << "F_new: " << std::endl;
        outStream << F_new << std::endl;
        if (F_new.norm() < F.norm())
        {
            F = F_new;
            (*vn) = vn_new;
            b = 1;
        }
        else
        {
            b /= 2;
        }
        iter += 1;

        //DBG(outStream.str());
    }

    //No convergence if this asserts!
    jassert(iter < 100);
}

void DampedNewtonRaphson::getCurrents(Eigen::MatrixXd* vn, Eigen::MatrixXd* in)
{
    vn_index = 0;
    for (int componentIndex = 0; componentIndex < numComponents; componentIndex++)
    {
        numberOfFunctions = (*nonLinearComponents)[componentIndex]->getNumberOfFunctions();
        (*nonLinearComponents)[componentIndex]->calculateJacobian(&componentsJacobian, *vn, vn_index);
        for (int functionIndex = 0; functionIndex < numberOfFunctions; functionIndex++)
        {
            (*in)(functionIndex + vn_index) = (*nonLinearComponents)[componentIndex]->calculateCurrents(*vn, vn_index, functionIndex);
        }
        vn_index += numberOfFunctions;
    }
}
