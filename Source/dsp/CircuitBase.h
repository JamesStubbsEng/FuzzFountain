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
    CircuitBase(
        std::unique_ptr<std::vector<float>> resistors,
        std::unique_ptr<std::vector<float>> capacitors,
        float Vcc,
        std::unique_ptr <Eigen::MatrixXd> NR,
        std::unique_ptr <Eigen::MatrixXd> Nv,
        std::unique_ptr <Eigen::MatrixXd> Nx,
        std::unique_ptr <Eigen::MatrixXd> Nu,
        std::unique_ptr <Eigen::MatrixXd> Nn,
        std::unique_ptr <Eigen::MatrixXd> No,
        std::unique_ptr<std::vector<NonLinearEquationBase*>> nonLinearComponents,
        int numNonlinears,
        bool hasVcc = true);
    void prepare(float sampleRate);
    void process(float* block, const int numSamples) noexcept;
    void reset();
protected:
    float fs = 44100.0f;
    std::unique_ptr <DampedNewtonRaphson> dnr;

    //user input
    std::unique_ptr<std::vector<float>> resistors;
    std::unique_ptr<std::vector<float>> capacitors;

    float Vcc = 9.0f;
    bool hasVcc = true;
    int numNonlinears = 1;
    std::unique_ptr<std::vector<NonLinearEquationBase*>> nonLinearComponents;

    std::unique_ptr <Eigen::MatrixXd> NR;
    std::unique_ptr <Eigen::MatrixXd> Nv;
    std::unique_ptr <Eigen::MatrixXd> Nx;
    std::unique_ptr <Eigen::MatrixXd> Nu;
    std::unique_ptr <Eigen::MatrixXd> Nn;
    std::unique_ptr <Eigen::MatrixXd> No;

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
    Eigen::MatrixXd u;
    Eigen::MatrixXd vn;
    Eigen::MatrixXd in;
    Eigen::MatrixXd vo;
};