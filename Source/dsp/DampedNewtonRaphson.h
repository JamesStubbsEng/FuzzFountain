/*
  ==============================================================================

    DampedNewtonRaphson.h
    Created: 16 May 2023 1:26:20pm
    Author:  James

  ==============================================================================
*/

#include <JuceHeader.h>
#include <Eigen/Core>
#include "NonLinearEquationBase.h"

#pragma once
class DampedNewtonRaphson
{
public:
    DampedNewtonRaphson(int numNonlinears, Eigen::MatrixXd* K,
        std::vector<NonLinearEquationBase*>* nonLinearComponents);
    void solve(Eigen::MatrixXd* vn, Eigen::MatrixXd* in, Eigen::MatrixXd* p);

    void getCurrentsAndJacobian(Eigen::MatrixXd* vn, Eigen::MatrixXd* in);
    void updateCurrents(Eigen::MatrixXd* vn, Eigen::MatrixXd* in);
private:
    Eigen::MatrixXd eye;
    Eigen::MatrixXd J;
    Eigen::MatrixXd step;
    Eigen::MatrixXd F;
    Eigen::MatrixXd F_new;
    Eigen::MatrixXd componentsJacobian;
    Eigen::MatrixXd vn_new;
    Eigen::MatrixXd in_new;
    Eigen::MatrixXd* K;
    std::vector<NonLinearEquationBase*>* nonLinearComponents;
    int numComponents;
    int vn_index = 0;

    double b = 1;
    int iter = 0;
    double tol = 1e-5;
    //double tol = 3.16e-3;
    int maxIterations = 100;

    int numberOfFunctions;
    int numberOfNonLinearFunctions;
};