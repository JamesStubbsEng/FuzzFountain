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
    this->numberOfNonLinearFunctions = numberOfNonLinearFunctions;
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
    step = Eigen::MatrixXd::Ones(numberOfNonLinearFunctions, 1);
    iter = 0;
    b = 1;

    //-------------dampedNR start ------------------
    //get first in
    //auto start = std::chrono::system_clock::now();
    //getCurrents(vn, in);
    //auto end = std::chrono::system_clock::now();
    //auto elapsed =
    //    std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    ////DBG("getCurrent() time: " + String(elapsed.count()) + " ns");
   

    //// get first F
    //start = std::chrono::system_clock::now();
    //F = *p + (*K) * (*in) - (*vn);
    //end = std::chrono::system_clock::now();
    //elapsed =
    //    std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    ////DBG("First F time: " + String(elapsed.count()) + " ns");
    //
    ////iterate to solve system of non linear equations
    //while (step.norm() > tol && iter < maxIterations)
    //{
    //    getCurrents(vn, in);
    //    J.noalias() = (*K) * componentsJacobian - eye;

    //    start = std::chrono::system_clock::now();
    //    step.noalias() = J.inverse() * F;
    //    //step.noalias() = J.colPivHouseholderQr().solve(F);
    //    //step.noalias() = J.householderQr().solve(F);
    //    //step.noalias() = J.llt().solve(F);
    //    end = std::chrono::system_clock::now();
    //    elapsed =
    //        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    //    //DBG("Eigen solve time: " + String(elapsed.count()) + " ns");

    //    start = std::chrono::system_clock::now();
    //    //vn_new = (*vn) - b * step;
    //    vn_new = (*vn);
    //    vn_new.noalias() -= b * step;
    //    getCurrents(&vn_new, &in_new);

    //    //F_new = *p + (*K) * in_new - vn_new;
    //    F_new.noalias() = (*K) * in_new;
    //    F_new.noalias() += *p;
    //    F_new.noalias() -= vn_new;

    //    end = std::chrono::system_clock::now();
    //    elapsed =
    //        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    //    //DBG("F_new time: " + String(elapsed.count()) + " ns");
    //    if (F_new.norm() < F.norm())
    //    {
    //        F = F_new;
    //        (*vn) = vn_new;
    //        b = 1;
    //    }
    //    else
    //    {
    //        b /= 2;
    //    }
    //    iter += 1;
    //}

    //-------------dampedNR end ------------------
    //-------------non damped NR start ------------------
    //std::ostringstream outStream;
    while (step.norm() > tol && iter < maxIterations)
    {
        getCurrents(vn, in);

        J.noalias() = (*K) * componentsJacobian - eye;
        //outStream << "J is:\n" << J << '\n';
        //outStream << "F is:\n" << F << '\n';
        //outStream << "step is:\n" << step << '\n';
        //outStream << "Jinverse is:\n" << J.inverse() << '\n';
        //DBG(outStream.str());
        
        J.noalias() = (*K) * componentsJacobian;
        J -= eye;
        F.noalias() = (*K) * (*in);
        F += (*p);
        F -= (*vn);
        step.noalias() = J.inverse() * F;
        (*vn) -= step;
        iter++;
    }
    //-------------non damped NR end ------------------

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
