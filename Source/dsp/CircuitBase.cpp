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
    std::unique_ptr<Eigen::MatrixXd> NR,
    std::unique_ptr<Eigen::MatrixXd> Nv,
    std::unique_ptr<Eigen::MatrixXd> Nx,
    std::unique_ptr<Eigen::MatrixXd> Nu,
    std::unique_ptr<Eigen::MatrixXd> Nn,
    std::unique_ptr<Eigen::MatrixXd> No,
    std::unique_ptr<std::vector<NonLinearEquationBase*>> nonLinearComponents,
    int numNonlinears,
    bool hasVcc)
{
    this->resistors = std::move(resistors);
    this->capacitors = std::move(capacitors);
    this->Vcc = Vcc;
    this->hasVcc = hasVcc;
    this->NR = std::move(NR);
    this->Nv = std::move(Nv);
    this->Nx = std::move(Nx);
    this->Nu = std::move(Nu);
    this->Nn = std::move(Nn);
    this->No = std::move(No);
    this->numNonlinears = numNonlinears;
    this->nonLinearComponents = std::move(nonLinearComponents);
}

CircuitBase::~CircuitBase()
{
    //TODO: should not be doing this anyways
    for (auto component : *nonLinearComponents)
        delete component;
}

void CircuitBase::prepare(float sampleRate)
{
    //for testing
    std::ostringstream outStream;

    fs = sampleRate;
    
    GR = Eigen::MatrixXd::Zero(resistors->size(), resistors->size());
    for (int i = 0; i < resistors->size(); i++)
        GR(i, i) = 1 / (*resistors)[i];

    //no variable resistors yet
    Rv = Eigen::MatrixXd::Zero(1, 1);

    Gx = Eigen::MatrixXd::Zero(capacitors->size(), capacitors->size());
    for (int i = 0; i < capacitors->size(); i++)
        Gx(i, i) = 2 * fs * (*capacitors)[i];

    S11 = NR->transpose() * GR * (*NR) + Nx->transpose() * Gx * (*Nx);
    S12 = Nu->transpose();
    S21 = *Nu;
    S22 = Eigen::MatrixXd::Zero(Nu->rows(), Nu->rows());

    //TODO: verify this with print outs. No idea if the topLeft... syntax is correct
    S = Eigen::MatrixXd::Zero(S11.rows() + S21.rows(), S11.cols() + S12.cols());
    S.topLeftCorner(S11.rows(), S11.cols()) = S11;
    S.topRightCorner(S12.rows(), S12.cols()) = S12;
    S.bottomLeftCorner(S21.rows(), S21.cols()) = S21;
    S.bottomRightCorner(S22.rows(), S22.cols()) = S22;

    outStream << "S: " << std::endl;
    outStream << S << std::endl;
    DBG(outStream.str());

    Eigen::MatrixXd B(5, 5);
    B = Eigen::MatrixXd::Random(5, 5);
    outStream << "Here is the matrix B:\n" << B << '\n';
    outStream << "The determinant of B is:" << B.determinant() << '\n';
    outStream << "The inverse of B is:\n" << B.inverse() << '\n';
    DBG(outStream.str());

    Si = Eigen::MatrixXd::Zero(S.rows(), S.cols());
    Si = S.inverse();

    Nrp = Eigen::MatrixXd::Zero(NR->rows(), NR->cols() + Nu->rows());
    Nrp.topLeftCorner(NR->rows(), NR->cols()) = (*NR);
    Nxp = Eigen::MatrixXd::Zero(Nx->rows(), Nx->cols() + Nu->rows());
    Nxp.topLeftCorner(Nx->rows(), Nx->cols()) = (*Nx);
    Nnp = Eigen::MatrixXd::Zero(Nn->rows(), Nn->cols() + Nu->rows());
    Nnp.topLeftCorner(Nn->rows(), Nn->cols()) = (*Nn);
    Nop = Eigen::MatrixXd::Zero(No->rows(), No->cols() + Nu->rows());
    Nop.topLeftCorner(No->rows(), No->cols()) = (*No);
    Nup = Eigen::MatrixXd::Zero(Nu->rows(), Nu->cols() + Nu->rows());
    Nup.topLeftCorner(Nu->rows(), Nu->cols()) = (*Nu);
    // padded identity matrix. 
    Nup2 = Eigen::MatrixXd::Zero(NR->cols() + Nu->rows(), Nu->rows());
    Nup2.bottomLeftCorner(Nu->rows(), Nu->rows()) = Eigen::MatrixXd::Identity(Nu->rows(), Nu->rows());

    //there are only capacitors, so fill the diagonal matrix Z with 1s
    Z = Eigen::MatrixXd::Zero(capacitors->size(), capacitors->size());
    for (int i = 0; i < capacitors->size(); i++)
        Z(i, i) = 1;

    outStream << "Z: " << std::endl;
    outStream << Z << std::endl;
    outStream << "Gx: " << std::endl;
    outStream << Gx << std::endl;
    outStream << "S: " << std::endl;
    outStream << S << std::endl;
    outStream << "Si: " << std::endl;
    outStream << Si << std::endl;
    outStream << "Nxp: " << std::endl;
    outStream << Nxp << std::endl;
    outStream << "Nx: " << std::endl;
    outStream << *Nx << std::endl;
    outStream << "Nup2: " << std::endl;
    outStream << Nup2 << std::endl;

    A = 2 * Z * Gx * Nxp * Si * Nxp.transpose() - Z;
    outStream << "A: " << std::endl;
    outStream << A << std::endl;
    DBG(outStream.str());
    B = 2 * Z * Gx * Nxp * Si * Nup2;
    outStream << "B: " << std::endl;
    outStream << B << std::endl;
    DBG(outStream.str());
    C = 2 * Z * Gx * Nxp * Si * Nnp.transpose();
    D = Nop * Si * Nxp.transpose();
    E = Nop * Si * Nup2;
    F = Nop * Si * Nnp.transpose();
    G = Nnp * Si * Nxp.transpose();
    H = Nnp * Si * Nup2;
    K = Nnp * Si * Nnp.transpose();

    vn = Eigen::MatrixXd::Zero(Nn->rows(), 1);
    in = Eigen::MatrixXd::Zero(Nn->rows(), 1);
    x = Eigen::MatrixXd::Zero(Nx->rows(), 1);

    //use is a column vector of vi and vcc
    if (hasVcc)
    {
        u = Eigen::MatrixXd::Zero(2, 1);
        u(1, 0) = Vcc;
    }
    else
    {
        u = Eigen::MatrixXd::Zero(1, 1);
    }

    p = Eigen::MatrixXd::Zero(G.rows(), x.cols());

    dnr = std::make_unique<DampedNewtonRaphson>(DampedNewtonRaphson(numNonlinears, &K, nonLinearComponents.get()));
}

void CircuitBase::process(float* block, const int numSamples) noexcept
{
    std::ostringstream outStream;
    for (int i = 0; i < numSamples; i++)
    {
        u(0, 0) = block[i];

        outStream << "G: " << std::endl;
        outStream << G << std::endl;
        outStream << "x: " << std::endl;
        outStream << x << std::endl;
        outStream << "H: " << std::endl;
        outStream << H << std::endl;
        outStream << "u: " << std::endl;
        outStream << u << std::endl;
        DBG(outStream.str());
        
        p = G * x + H * u;

        outStream << "p: " << std::endl;
        outStream << p << std::endl;

        dnr->solve(&vn, &in, &p);

        outStream << "p: " << std::endl;
        outStream << p << std::endl;
        outStream << "vn: " << std::endl;
        outStream << vn << std::endl;
        outStream << "in: " << std::endl;
        outStream << in << std::endl;
        DBG(outStream.str());

        block[i] = (D * x + E * u + F * in)(0,0);
        x = A * x + B * u + C * in;
    }
}
