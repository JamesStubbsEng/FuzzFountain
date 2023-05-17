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
    DampedNewtonRaphson(int numNonlinears, Eigen::MatrixXf* K,
        std::vector<NonLinearEquationBase*>* nonLinearComponents);
    void solve(Eigen::MatrixXf* vn, Eigen::MatrixXf* in, Eigen::MatrixXf* p);

    void getCurrents(Eigen::MatrixXf* vn, Eigen::MatrixXf* in);
private:
    Eigen::MatrixXf eye;
    Eigen::MatrixXf J;
    Eigen::MatrixXf step;
    Eigen::MatrixXf F;
    Eigen::MatrixXf F_new;
    Eigen::MatrixXf componentsJacobian;
    Eigen::MatrixXf vn_new;
    Eigen::MatrixXf in_new;
    Eigen::MatrixXf* K;
    std::vector<NonLinearEquationBase*>* nonLinearComponents;
    int numComponents;
    int vn_index = 0;

    int b = 1;
    int iter = 0;
    float tol = 1e-5;
    int maxIterations = 100;

    int numberOfFunctions;
    int numberOfNonLinearFunctions;
};