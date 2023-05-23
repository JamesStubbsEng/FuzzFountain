/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include <Eigen/Dense>
#include "dsp/CircuitBase.h"
#include "dsp/DiodeNLEQ.h"

//==============================================================================
/**
*/
class FuzzFountainAudioProcessor  : public juce::AudioProcessor
{
public:
    //==============================================================================
    FuzzFountainAudioProcessor();
    ~FuzzFountainAudioProcessor() override;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

private:
    //==============================================================================
    AudioProcessorValueTreeState parameters;

    std::atomic<float>* inputGainParameter = nullptr;
    std::atomic<float>* mixParameter = nullptr;
    std::atomic<float>* outputGainParameter = nullptr;

    dsp::DryWetMixer<float> dryWetMixer;

    size_t oversampleFactor = 1;
    dsp::Oversampling<float> oversampling{ 2, oversampleFactor, dsp::Oversampling<float>::filterHalfBandPolyphaseIIR };

    dsp::Gain<float> inputGain;
    dsp::Gain<float> outputGain;

    AudioBuffer<float> parallelBuffer;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FuzzFountainAudioProcessor)
};
