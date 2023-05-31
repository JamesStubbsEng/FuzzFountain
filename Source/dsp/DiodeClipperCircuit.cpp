/*
  ==============================================================================

    DiodeClipperCircuit.cpp
    Created: 16 May 2023 3:22:09pm
    Author:  James

  ==============================================================================
*/

#include "DiodeClipperCircuit.h"
#define _USE_MATH_DEFINES
#include <math.h>

DiodeClipperCircuit::DiodeClipperCircuit()
{
    resistors.push_back(240e3);
    capacitors.push_back(80e-12);

    Vcc = 9;
    hasVcc = false;
    numNonlinears = 1;

    NR = Eigen::MatrixXd::Zero(1, 2);
    NR << 1, -1;
    Nv = Eigen::MatrixXd::Zero(1, 1);
    Nv << 0;
    Nx = Eigen::MatrixXd::Zero(1, 2);
    Nx << 0, 1;
    Nu = Eigen::MatrixXd::Zero(1, 2);
    Nu << 1, 0;
    Nn = Eigen::MatrixXd::Zero(1, 2);
    Nn << 0, 1;
    No = Eigen::MatrixXd::Zero(1, 2);
    No << 0, 1;

    u = Eigen::MatrixXd::Zero(1, 1);

    //DiodeClipperCircuit object owns DiodeNLEQ object.
    diodeNLEQ = std::make_unique<DiodeNLEQ>();
    nonLinearComponents.push_back(diodeNLEQ.get());

    //OFFLINE TEST CODE
    //float fs_test = 44100.0;
    //float fc_test = 400.0;

    //prepare(fs_test);

    ////test first 10 ms of sin
    //auto start = std::chrono::system_clock::now();
    //for (int i = 0; i < 441; i++)
    //{
    //    //vou should be soft clipped at about +-0.5;
    //    float vin = std::sinf(2 * M_PI * fc_test * i / fs_test);
    //    //DBG("vin = " + String(vin));
    //    process(&vin, 1);
    //    float vo = vin;
    //    DBG("sample: " + String(i) + " vo = " + String(vo));
    //}

    //auto end = std::chrono::system_clock::now();
    //auto elapsed =
    //    std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //DBG("Total Time elapsed: " + String(elapsed.count()) + " ms");
}

