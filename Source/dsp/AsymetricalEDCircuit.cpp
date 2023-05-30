/*
  ==============================================================================

    AsymetricalEDCircuit.cpp
    Created: 29 May 2023 11:53:51am
    Author:  James

  ==============================================================================
*/

#include "AsymetricalEDCircuit.h"
#define _USE_MATH_DEFINES
#include <math.h>

AsymetricalEDCircuit::AsymetricalEDCircuit()
{
    resistors.push_back(100e3);
    resistors.push_back(22);
    resistors.push_back(470e3);
    resistors.push_back(10e3);
    resistors.push_back(700);
    capacitors.push_back(0.047e-6);
    capacitors.push_back(0.25e-9);
    capacitors.push_back(0.47e-6);

    Vcc = 9;
    hasVcc = true;
    numNonlinears = 2;

    NR = Eigen::MatrixXd::Zero(5, 6);
    NR << 0, 0, 0, 0, 1, 0,
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, -1, 1,
          0, 1, 0, 0, 0, -1,
          0, 0, 0, 1, 0, 0;
    Nv = Eigen::MatrixXd::Zero(1, 1);
    Nv << 0;
    Nx = Eigen::MatrixXd::Zero(3, 6);
    Nx << -1, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 1, -1,
           0, 0, 0, -1, 0, 1;
    Nu = Eigen::MatrixXd::Zero(2, 6);
    Nu << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0;
    Nn = Eigen::MatrixXd::Zero(2, 6);
    Nn << 0, 0, 0, 0, -1, 1,
          0, 0, -1, 0, 0, 1;
    No = Eigen::MatrixXd::Zero(1, 6);
    No << 0, 0, 0, 1, 0, 0;

    //DiodeClipperCircuit object owns DiodeNLEQ object.
    npnNLEQ = std::make_unique<NPN_NLEQ>();
    nonLinearComponents.push_back(npnNLEQ.get());

    //OFFLINE TEST CODE
    //float fs_test = 44100.0;
    //float fc_test = 400.0;

    //prepare(fs_test);

    ////test first 10 ms of sin
    //auto start = std::chrono::system_clock::now();
    //for (int i = 0; i < 441; i++)
    //{
    //    //vou should be asymetrical with heavily clipped top
    //    float vin = std::sinf(2 * M_PI * fc_test * i / fs_test);
    //    //DBG("vin = " + String(vin));
    //    process(&vin, 1);
    //    //float vo = vin;
    //    //DBG("sample: " + String(i) + " vo = " + String(vo));
    //}

    //auto end = std::chrono::system_clock::now();
    //auto elapsed =
    //    std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //DBG("Total Time elapsed: " + String(elapsed.count()) + " us");
}