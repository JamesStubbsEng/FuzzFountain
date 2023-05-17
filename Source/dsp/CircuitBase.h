/*
  ==============================================================================

    CircuitBase.h
    Created: 16 May 2023 3:24:27pm
    Author:  James

  ==============================================================================
*/

#pragma once
 
#include <Eigen/core>
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
        std::unique_ptr <Eigen::MatrixXf> NR,
        std::unique_ptr <Eigen::MatrixXf> Nv,
        std::unique_ptr <Eigen::MatrixXf> Nx,
        std::unique_ptr <Eigen::MatrixXf> Nu,
        std::unique_ptr <Eigen::MatrixXf> Nn,
        std::unique_ptr <Eigen::MatrixXf> No,
        std::unique_ptr<std::vector<NonLinearEquationBase*>> nonLinearComponents,
        int numNonlinears);
    void prepare(float sampleRate);
    void process(float* block, const int numSamples) noexcept;
protected:
    float fs = 44100.0f;
    std::unique_ptr <DampedNewtonRaphson> dnr;

    //user input
    std::unique_ptr<std::vector<float>> resistors;
    std::unique_ptr<std::vector<float>> capacitors;

    float Vcc = 9.0f;
    int numNonlinears = 1;
    std::unique_ptr<std::vector<NonLinearEquationBase*>> nonLinearComponents;

    std::unique_ptr <Eigen::MatrixXf> NR;
    std::unique_ptr <Eigen::MatrixXf> Nv;
    std::unique_ptr <Eigen::MatrixXf> Nx;
    std::unique_ptr <Eigen::MatrixXf> Nu;
    std::unique_ptr <Eigen::MatrixXf> Nn;
    std::unique_ptr <Eigen::MatrixXf> No;

    //generated from user input
    Eigen::MatrixXf GR;
    Eigen::MatrixXf Rv;
    Eigen::MatrixXf Gx;

    Eigen::MatrixXf Z;

    //intermediates used in process function
    Eigen::MatrixXf S11;
    Eigen::MatrixXf S12;
    Eigen::MatrixXf S21;
    Eigen::MatrixXf S22;

    Eigen::MatrixXf S;
    Eigen::MatrixXf Si;

    Eigen::MatrixXf Nrp;
    Eigen::MatrixXf Nxp;
    Eigen::MatrixXf Nnp;
    Eigen::MatrixXf Nop;
    Eigen::MatrixXf Nup;
    Eigen::MatrixXf Nup2;

    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf D;
    Eigen::MatrixXf E;
    Eigen::MatrixXf F;
    Eigen::MatrixXf G;
    Eigen::MatrixXf H;
    Eigen::MatrixXf K;

    Eigen::MatrixXf p;
    Eigen::MatrixXf x;
    Eigen::MatrixXf u;
    Eigen::MatrixXf vn;
    Eigen::MatrixXf in;
};