/*
  ==============================================================================

    CircuitBase.cpp
    Created: 16 May 2023 3:24:27pm
    Author:  James

  ==============================================================================
*/

#include "CircuitBase.h"

CircuitBase::CircuitBase(
    std::unique_ptr<std::vector<float>> resistors,
    std::unique_ptr<std::vector<float>> capacitors,
    float Vcc,
    std::unique_ptr<Eigen::MatrixXf> NR,
    std::unique_ptr<Eigen::MatrixXf> Nv,
    std::unique_ptr<Eigen::MatrixXf> Nx,
    std::unique_ptr<Eigen::MatrixXf> Nu,
    std::unique_ptr<Eigen::MatrixXf> Nn,
    std::unique_ptr<Eigen::MatrixXf> No,
    std::unique_ptr<std::vector<NonLinearEquationBase*>> nonLinearComponents,
    int numNonlinears)
{
    resistors = std::move(resistors);
    capacitors = std::move(capacitors);
    Vcc = Vcc;
    NR = std::move(NR);
    Nv = std::move(Nv);
    Nx = std::move(Nx);
    Nu = std::move(Nu);
    Nn = std::move(Nn);
    No = std::move(No);
    numNonlinears = numNonlinears;
    nonLinearComponents = std::move(nonLinearComponents);
}

void CircuitBase::prepare(float sampleRate)
{
    fs = sampleRate;
    
    GR = Eigen::MatrixXf::Zero(resistors->size(), resistors->size());
    for (int i = 0; i < resistors->size(); i++)
        GR(i, i) = 1 / (*resistors)[i];

    //no variable resistors yet
    Rv = Eigen::MatrixXf::Zero(1, 1);

    Gx = Eigen::MatrixXf::Zero(capacitors->size(), capacitors->size());
    for (int i = 0; i < capacitors->size(); i++)
        Gx(i, i) = 2 * fs * (*capacitors)[i];

    S11 = NR->transpose() * GR * (*NR) + Nx->transpose() * Gx * (*Nx);
    S12 = Nu->transpose();
    S21 = *Nu;
    S22 = Eigen::MatrixXf::Zero(Nu->rows(), Nu->rows());

    //TODO: verify this with print outs. No idea if the topLeft... syntax is correct
    S = Eigen::MatrixXf::Zero(S11.rows() + S12.rows(), S11.cols() + S21.cols());
    S.topLeftCorner(S11.rows(), S11.cols()) = S11;
    S.topRightCorner(S12.rows(), S12.cols()) = S12;
    S.bottomLeftCorner(S21.rows(), S21.cols()) = S21;
    S.bottomRightCorner(S22.rows(), S22.cols()) = S22;

    Si = S.inverse();

    Nrp = Eigen::MatrixXf::Zero(2 * NR->rows(), NR->cols());
    Nrp.topLeftCorner(NR->rows(), NR->cols()) = (*NR);
    Nxp = Eigen::MatrixXf::Zero(2 * Nx->rows(), Nx->cols());
    Nxp.topLeftCorner(Nx->rows(), Nx->cols()) = (*Nx);
    Nnp = Eigen::MatrixXf::Zero(2 * Nn->rows(), Nn->cols());
    Nnp.topLeftCorner(Nn->rows(), Nn->cols()) = (*Nn);
    Nop = Eigen::MatrixXf::Zero(2 * No->rows(), No->cols());
    Nop.topLeftCorner(No->rows(), No->cols()) = (*No);
    Nup = Eigen::MatrixXf::Zero(2 * Nu->rows(), Nu->cols());
    Nup.topLeftCorner(Nu->rows(), Nu->cols()) = (*Nu);
    // padded identity matrix. 
    Nup2 = Eigen::MatrixXf::Zero(Nu->rows(), Nu->cols() + Nu->rows());
    Nup2.bottomLeftCorner(Nu->rows(), Nu->rows()) = Eigen::MatrixXf::Identity(Nu->rows(), Nu->rows());

    //there are only capacitors, so fill the diagonal matrix Z with 1s
    Z = Eigen::MatrixXf::Zero(capacitors->size(), capacitors->size());
    for (int i = 0; i < capacitors->size(); i++)
        Z(i, i) = 1;

    A = 2 * Z * Gx * Nxp * Si * Nxp.transpose() - Z;
    B = 2 * Z * Gx * Nxp * Si * Nup2;
    C = 2 * Z * Gx * Nxp * Si * Nnp.transpose();
    D = Nop * Si * Nxp.transpose();
    E = Nop * Si * Nup2;
    F = Nop * Si * Nnp.transpose();
    G = Nnp * Si * Nxp.transpose();
    H = Nnp * Si * Nup2;
    K = Nnp * Si * Nnp.transpose();

    vn = Eigen::MatrixXf::Zero(Nn->rows(), 1);
    in = Eigen::MatrixXf::Zero(Nn->rows(), 1);
    x = Eigen::MatrixXf::Zero(Nx->rows(), 1);

    //use is a column vector of vi and vcc
    u = Eigen::MatrixXf::Zero(2, 1);
    u(1, 0) = Vcc;
    p = Eigen::MatrixXf::Zero(G.rows(), x.cols());

    dnr = std::make_unique<DampedNewtonRaphson>(DampedNewtonRaphson(numNonlinears, &K, nonLinearComponents.get()));
}

void CircuitBase::process(float* block, const int numSamples) noexcept
{
    for (int i = 0; i < numSamples; i++)
    {
        u(0, 0) = block[i];
        
        p = G * x + H * u;

        dnr->solve(&vn, &in, &p);

        block[i] = (D * x + E * u + F * in)(0,0);
        x = A * x + B * u + C * in;
    }
}
