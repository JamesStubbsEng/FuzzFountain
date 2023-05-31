/*
  ==============================================================================

    CircuitBase.h
    Created: 16 May 2023 3:24:27pm
    Author:  James

  ==============================================================================
*/

#pragma once
 
//#include <Eigen/core>
//#include <Eigen/LU>
#include <Eigen/Dense>
#include "NonLinearEquationBase.h"
#include "DampedNewtonRaphson.h"

class CircuitBase
{
public:
    /*Pass in circuit information here. Must call prepare()!!*/
    CircuitBase() = default;
    virtual ~CircuitBase() = default;
    void prepare(float sampleRate);
    void process(float* block, const int numSamples) noexcept;
    void reset();
protected:
    float fs = 44100.0f;
    std::unique_ptr <DampedNewtonRaphson> dnr;

    //user input
    std::vector<float> resistors;
    std::vector<float> capacitors;

    float Vcc = 9.0f;
    bool hasVcc = true;
    int numNonlinears = 1;
    std::vector<NonLinearEquationBase*> nonLinearComponents;

    Eigen::MatrixXd NR;
    Eigen::MatrixXd Nv;
    Eigen::MatrixXd Nx;
    Eigen::MatrixXd Nu;
    Eigen::MatrixXd Nn;
    Eigen::MatrixXd No;

    Eigen::MatrixXd u;
private:
    //generated from user input
    Eigen::MatrixXd GR;
    Eigen::MatrixXd Rv;
    Eigen::MatrixXd Gx;

    Eigen::MatrixXd Z;

    //intermediates used in process function
    Eigen::MatrixXd S11;
    Eigen::MatrixXd S12;
    Eigen::MatrixXd S21;
    Eigen::MatrixXd S22;

    Eigen::MatrixXd S;
    Eigen::MatrixXd Si;

    Eigen::MatrixXd Nrp;
    Eigen::MatrixXd Nxp;
    Eigen::MatrixXd Nnp;
    Eigen::MatrixXd Nop;
    Eigen::MatrixXd Nup;
    Eigen::MatrixXd Nup2;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;
    Eigen::MatrixXd E;
    Eigen::MatrixXd F;
    Eigen::MatrixXd G;
    Eigen::MatrixXd H;
    Eigen::MatrixXd K;

    Eigen::MatrixXd p;
    Eigen::MatrixXd x;
    Eigen::MatrixXd vn;
    Eigen::MatrixXd in;
    Eigen::MatrixXd vo;
};