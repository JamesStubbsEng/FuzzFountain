/*
  ==============================================================================

    DiodeClipperCircuit.cpp
    Created: 16 May 2023 3:22:09pm
    Author:  James

  ==============================================================================
*/

#include "DiodeClipperCircuit.h"

DiodeClipperCircuit::DiodeClipperCircuit()
{
    //TODO: Make DiodeClipperCircuit inherit from CircuitBase, and make CircuitBase own all objects

    std::unique_ptr<std::vector<float>> resistors = std::make_unique<std::vector<float>>();
    resistors->push_back(240e3);
    std::unique_ptr<std::vector<float>> capacitors = std::make_unique<std::vector<float>>();
    capacitors->push_back(80e-12);

    std::unique_ptr <Eigen::MatrixXd> NR = std::make_unique<Eigen::MatrixXd>();
    *NR = Eigen::MatrixXd::Zero(1, 2);
    *NR << 1, -1;
    std::unique_ptr <Eigen::MatrixXd> Nv = std::make_unique<Eigen::MatrixXd>();
    *Nv = Eigen::MatrixXd::Zero(1, 1);
    *Nv << 0;
    std::unique_ptr <Eigen::MatrixXd> Nx = std::make_unique<Eigen::MatrixXd>();
    *Nx = Eigen::MatrixXd::Zero(1, 2);
    *Nx << 0, 1;
    std::unique_ptr <Eigen::MatrixXd> Nu = std::make_unique<Eigen::MatrixXd>();
    *Nu = Eigen::MatrixXd::Zero(1, 2);
    *Nu << 1, 0;
    std::unique_ptr <Eigen::MatrixXd> Nn = std::make_unique<Eigen::MatrixXd>();
    *Nn = Eigen::MatrixXd::Zero(1, 2);
    *Nn << 0, 1;
    std::unique_ptr <Eigen::MatrixXd> No = std::make_unique<Eigen::MatrixXd>();
    *No = Eigen::MatrixXd::Zero(1, 2);
    *No << 0, 1;
    std::unique_ptr<std::vector<NonLinearEquationBase*>> nonLinearComponents = std::make_unique<std::vector<NonLinearEquationBase*>>();

    //DiodeClipperCircuit object owns DiodeNLEQ object.
    diodeNLEQ = std::make_unique<DiodeNLEQ>();
    nonLinearComponents->push_back(diodeNLEQ.get());

    rcDiodeClipper = std::make_unique<CircuitBase>(std::move(resistors), std::move(capacitors), 9, std::move(NR),
        std::move(Nv), std::move(Nx), std::move(Nu), std::move(Nn),
        std::move(No), std::move(nonLinearComponents), 1, false);


    //OFFLINE TEST CODE
    //float fs_test = 44100.0;
    //float fc_test = 400.0;

    //rcDiodeClipper->prepare(fs_test);

    ////test first 10 ms of sin
    //auto start = std::chrono::system_clock::now();
    //for (int i = 0; i < 441; i++)
    //{
    //    //vou should be soft clipped at about +-0.5;
    //    float vin = std::sinf(2 * M_PI * fc_test * i / fs_test);
    //    //DBG("vin = " + String(vin));
    //    rcDiodeClipper->process(&vin, 1);
    //    float vo = vin;
    //    DBG("sample: " + String(i) + " vo = " + String(vo));
    //}

    //auto end = std::chrono::system_clock::now();
    //auto elapsed =
    //    std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //DBG("Total Time elapsed: " + String(elapsed.count()) + " ms");
}

void DiodeClipperCircuit::prepare(float sampleRate)
{
    rcDiodeClipper->prepare(sampleRate);
}

void DiodeClipperCircuit::process(float* block, const int numSamples) noexcept
{
    rcDiodeClipper->process(block, numSamples);
}

void DiodeClipperCircuit::reset()
{
    rcDiodeClipper->reset();
}
