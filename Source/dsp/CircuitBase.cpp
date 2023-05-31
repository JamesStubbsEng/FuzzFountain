/*
  ==============================================================================

    CircuitBase.cpp
    Created: 16 May 2023 3:24:27pm
    Author:  James

  ==============================================================================
*/

#include "CircuitBase.h"

void CircuitBase::prepare(float sampleRate)
{
    std::ostringstream outStream;
    fs = sampleRate;
    
    GR = Eigen::MatrixXd::Zero(resistors.size(), resistors.size());
    for (int i = 0; i < resistors.size(); i++)
        GR(i, i) = 1 / resistors[i];

    //no variable resistors yet
    Rv = Eigen::MatrixXd::Zero(1, 1);

    Gx = Eigen::MatrixXd::Zero(capacitors.size(), capacitors.size());
    for (int i = 0; i < capacitors.size(); i++)
        Gx(i, i) = 2 * fs * capacitors[i];

    //outStream << "NR.transpose() * GR * NR: " << std::endl;
    //outStream << NR.transpose() * GR * NR << std::endl;
    //outStream << "Nx.transpose() * Gx * Nx: " << std::endl;
    //outStream << Nx.transpose() * Gx * Nx << std::endl;

    S11.noalias() = NR.transpose() * GR * NR + Nx.transpose() * Gx * Nx;
    S12 = Nu.transpose();
    S21 = Nu;
    S22 = Eigen::MatrixXd::Zero(Nu.rows(), Nu.rows());

    //outStream << "S11: " << std::endl;
    //outStream << S11 << std::endl;
    //outStream << "S12: " << std::endl;
    //outStream << S12 << std::endl;
    //outStream << "S21: " << std::endl;
    //outStream << S21 << std::endl;
    //outStream << "S22: " << std::endl;
    //outStream << S22 << std::endl;

    S = Eigen::MatrixXd::Zero(S11.rows() + S21.rows(), S11.cols() + S12.cols());
    S.topLeftCorner(S11.rows(), S11.cols()) = S11;
    S.topRightCorner(S12.rows(), S12.cols()) = S12;
    S.bottomLeftCorner(S21.rows(), S21.cols()) = S21;
    S.bottomRightCorner(S22.rows(), S22.cols()) = S22;

    Si = Eigen::MatrixXd::Zero(S.rows(), S.cols());
    Si = S.inverse();

    Nrp = Eigen::MatrixXd::Zero(NR.rows(), NR.cols() + Nu.rows());
    Nrp.topLeftCorner(NR.rows(), NR.cols()) = NR;
    Nxp = Eigen::MatrixXd::Zero(Nx.rows(), Nx.cols() + Nu.rows());
    Nxp.topLeftCorner(Nx.rows(), Nx.cols()) = Nx;
    Nnp = Eigen::MatrixXd::Zero(Nn.rows(), Nn.cols() + Nu.rows());
    Nnp.topLeftCorner(Nn.rows(), Nn.cols()) = Nn;
    Nop = Eigen::MatrixXd::Zero(No.rows(), No.cols() + Nu.rows());
    Nop.topLeftCorner(No.rows(), No.cols()) = No;
    Nup = Eigen::MatrixXd::Zero(Nu.rows(), Nu.cols() + Nu.rows());
    Nup.topLeftCorner(Nu.rows(), Nu.cols()) = Nu;
    // padded identity matrix. 
    Nup2 = Eigen::MatrixXd::Zero(NR.cols() + Nu.rows(), Nu.rows());
    Nup2.bottomLeftCorner(Nu.rows(), Nu.rows()) = Eigen::MatrixXd::Identity(Nu.rows(), Nu.rows());

    //there are only capacitors, so fill the diagonal matrix Z with 1s
    Z = Eigen::MatrixXd::Zero(capacitors.size(), capacitors.size());
    for (int i = 0; i < capacitors.size(); i++)
        Z(i, i) = 1;

    A.noalias() = 2 * Z * Gx * Nxp * Si * Nxp.transpose() - Z;

    B.noalias() = 2 * Z * Gx * Nxp * Si * Nup2;

    C.noalias() = 2 * Z * Gx * Nxp * Si * Nnp.transpose();
    D.noalias() = Nop * Si * Nxp.transpose();
    E.noalias() = Nop * Si * Nup2;
    F.noalias() = Nop * Si * Nnp.transpose();
    G.noalias() = Nnp * Si * Nxp.transpose();
    H.noalias() = Nnp * Si * Nup2;
    K.noalias() = Nnp * Si * Nnp.transpose();

    vn = Eigen::MatrixXd::Zero(Nn.rows(), 1);
    in = Eigen::MatrixXd::Zero(Nn.rows(), 1);
    x = Eigen::MatrixXd::Zero(Nx.rows(), 1);

    p = Eigen::MatrixXd::Zero(G.rows(), x.cols());

    dnr = std::make_unique<DampedNewtonRaphson>(DampedNewtonRaphson(numNonlinears, &K, &nonLinearComponents));

    //for testing

    //outStream << "NR: " << std::endl;
    //outStream << NR << std::endl;
    //outStream << "GR: " << std::endl;
    //outStream << GR << std::endl;
    //outStream << "Nx: " << std::endl;
    //outStream << Nx << std::endl;
    //outStream << "Gx: " << std::endl;
    //outStream << Gx << std::endl;
    //outStream << "S: " << std::endl;
    //outStream << S << std::endl;
    //outStream << "Z: " << std::endl;
    //outStream << Z << std::endl;
    //outStream << "Gx: " << std::endl;
    //outStream << Gx << std::endl;
    //outStream << "Si: " << std::endl;
    //outStream << Si << std::endl;
    //outStream << "Nxp: " << std::endl;
    //outStream << Nxp << std::endl;
    //outStream << "Nx: " << std::endl;
    //outStream << Nx << std::endl;
    //outStream << "Nup2: " << std::endl;
    //outStream << Nup2 << std::endl;
    //outStream << "A: " << std::endl;
    //outStream << A << std::endl;
    //outStream << "B: " << std::endl;
    //outStream << B << std::endl;
    //DBG(outStream.str());
}

void CircuitBase::process(float* block, const int numSamples) noexcept
{
    std::ostringstream outStream;
    for (int i = 0; i < numSamples; i++)
    {
        auto start = std::chrono::system_clock::now();
        u(0, 0) = block[i];

        //outStream << "G: " << std::endl;
        //outStream << G << std::endl;
        //outStream << "x: " << std::endl;
        //outStream << x << std::endl;
        //outStream << "H: " << std::endl;
        //outStream << H << std::endl;
        //outStream << "u: " << std::endl;
        //outStream << u << std::endl;
        //outStream << "C: " << std::endl;
        //outStream << C << std::endl;
        
        p.noalias() = G * x;
        p.noalias() += H * u;

        //outStream << "p: " << std::endl;
        //outStream << p << std::endl;

        auto end = std::chrono::system_clock::now();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        //DBG("Pre DNR: " + String(elapsed.count()) + " ns");

        start = std::chrono::system_clock::now();
        dnr->solve(&vn, &in, &p);

        end = std::chrono::system_clock::now();
        elapsed =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        //DBG("Pre DNR to post DNR: " + String(elapsed.count()) + " ns");

        //outStream << "p: " << std::endl;
        //outStream << p << std::endl;
        //outStream << "vn: " << std::endl;
        //outStream << vn << std::endl;
        //outStream << "in: " << std::endl;
        //outStream << in << std::endl;
        //outStream << "x: " << std::endl;
        //outStream << x << std::endl;
        //DBG(outStream.str());

        start = std::chrono::system_clock::now();

        vo.noalias() = D * x;
        vo.noalias() += E * u;
        vo.noalias() += F * in;
        block[i] = vo(0,0);

        x = A * x;
        x.noalias() += B * u;
        x.noalias() += C * in;

        end = std::chrono::system_clock::now();
        elapsed =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        //DBG("post DNR to end of process: " + String(elapsed.count()) + " ns");
    }
}

void CircuitBase::reset()
{
    prepare(fs);
}
