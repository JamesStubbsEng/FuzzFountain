/*
  ==============================================================================

    FuzzFaceCircuit.cpp
    Created: 31 May 2023 12:29:28pm
    Author:  James

  ==============================================================================
*/

#include "FuzzFaceCircuit.h"
#define _USE_MATH_DEFINES
#include <math.h>

FuzzFaceCircuit::FuzzFaceCircuit()
{
    resistors.push_back(33e3);
    resistors.push_back(470);
    resistors.push_back(8.2e3);
    resistors.push_back(100e3);
    //pots maxed out for testing
    resistors.push_back(1e3);
    resistors.push_back(500e3);

    capacitors.push_back(2.2e-6);
    capacitors.push_back(20e-6);
    capacitors.push_back(0.01e-6);

    Vcc = 9;
    hasVcc = true;
    numNonlinears = 4;

    NR = Eigen::MatrixXd::Zero(6, 8);
    NR << 0, 0, -1, 1, 0, 0, 0, 0,
        0, 0, -1, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1, -1, 0,
        0, 1, 0, 0, -1, 0, 0, 0,
        0, 0, 0, 0, -1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, -1;

    Nv = Eigen::MatrixXd::Zero(1, 1);
    Nv << 0;
    Nx = Eigen::MatrixXd::Zero(3, 8);
    Nx << 1, -1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, -1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, -1, 1;
       
    Nu = Eigen::MatrixXd::Zero(2, 8);
    Nu << 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, -1, 0, 0, 0, 0, 0;
    Nn = Eigen::MatrixXd::Zero(4, 8);
    Nn << 0, 1, 0, -1, 0, 0, 0, 0,
        0, 0, 0, -1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, -1, 0, 0,
        0, 0, 0, 0, 1, -1, 0, 0;
    //Nn << 0, -1, 0, 1, 0, 0, 0, 0,
    //    0, 0, 0, 1, 0, 0, 0, 0,
    //    0, 0, 0, -1, 0, 1, 0, 0,
    //    0, 0, 0, 0, -1, 1, 0, 0;
    No = Eigen::MatrixXd::Zero(1, 8);
    No << 0, 0, 0, 0, 0, 0, 0, 1;

    u = Eigen::MatrixXd::Zero(2, 1);
    u(1, 0) = Vcc;

    //DiodeClipperCircuit object owns DiodeNLEQ object.
    pnpNLEQ = std::make_unique<PNP_NLEQ>(1.16e-14, 200, 3);
    pnpNLEQ2 = std::make_unique<PNP_NLEQ>(1.16e-14, 200, 3);
    //npnNLEQ = std::make_unique<NPN_NLEQ>();
    //npnNLEQ2 = std::make_unique<NPN_NLEQ>();
    nonLinearComponents.push_back(pnpNLEQ.get());
    nonLinearComponents.push_back(pnpNLEQ2.get());
    //nonLinearComponents.push_back(npnNLEQ.get());
    //nonLinearComponents.push_back(npnNLEQ2.get());

    //OFFLINE TEST CODE
    float fs_test = 44100.0;
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