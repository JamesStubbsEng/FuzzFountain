/*
  ==============================================================================

    PNPCommonEmitterCircuit.cpp
    Created: 31 May 2023 2:36:33pm
    Author:  James

  ==============================================================================
*/

#include "PNPCommonEmitterCircuit.h"
#define _USE_MATH_DEFINES
#include <math.h>

PNPCommonEmitterCircuit::PNPCommonEmitterCircuit()
{
    resistors.push_back(1e3);
    resistors.push_back(360e3);
    resistors.push_back(82e3);
    resistors.push_back(22e3);
    resistors.push_back(5.1e3);
    resistors.push_back(25e3);

    capacitors.push_back(0.15e-6);
    capacitors.push_back(4.7e-6);
    capacitors.push_back(0.068e-6);

    Vcc = 9;
    hasVcc = true;
    numNonlinears = 2;

    NR = Eigen::MatrixXd::Zero(6, 7);
    NR << 0, 1, -1, 0, 0, 0, 0,
        1, 0, 0, -1, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0,
        1, 0, 0, 0, 0, -1, 0,
        0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 1;


    Nv = Eigen::MatrixXd::Zero(1, 1);
    Nv << 0;
    Nx = Eigen::MatrixXd::Zero(3, 7);
    Nx << 0, 0, 1, -1, 0, 0, 0,
        0, 0, 0, 0, -1, 0, 0,
        0, 0, 0, 0, 0, 1, -1;

    Nu = Eigen::MatrixXd::Zero(2, 7);
    Nu << -1, 0, 0, 0, 0, 0, 0, 
        0, 1, 0, 0, 0, 0, 0;
    Nn = Eigen::MatrixXd::Zero(2, 7);
    Nn << 0, 0, 0, 1, 0, -1, 0,
        0, 0, 0, 0, 1, -1, 0;

    No = Eigen::MatrixXd::Zero(1, 7);
    No << 0, 0, 0, 0, 0, 0, 1;

    u = Eigen::MatrixXd::Zero(2, 1);
    u(0, 0) = Vcc;

    //DiodeClipperCircuit object owns DiodeNLEQ object.
    pnpNLEQ = std::make_unique<PNP_NLEQ>(1.16e-14, 200, 3);
    nonLinearComponents.push_back(pnpNLEQ.get());


    //OFFLINE TEST CODE
    float fs_test = 48000.0;
    float fc_test = 400.0;

    prepare(fs_test);

    //test first 10 ms of sin
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < 441; i++)
    {
        //vou should be asymetrical with heavily clipped top
        float vin = std::sinf(2 * M_PI * fc_test * i / fs_test);
        //DBG("vin = " + String(vin));
        process(&vin, 1);
        float vo = vin;
        DBG("sample: " + String(i) + " vo = " + String(vo));
    }

    auto end = std::chrono::system_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    DBG("Total Time elapsed: " + String(elapsed.count()) + " us");
}