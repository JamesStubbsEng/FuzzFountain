/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
/**
*/
class FuzzFountainAudioProcessorEditor  : public juce::AudioProcessorEditor
{
public:
    FuzzFountainAudioProcessorEditor (FuzzFountainAudioProcessor&, AudioProcessorValueTreeState& vts);
    ~FuzzFountainAudioProcessorEditor() override;

    //==============================================================================
    void paint (juce::Graphics&) override;
    void resized() override;

private:
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    FuzzFountainAudioProcessor& audioProcessor;

    AudioProcessorValueTreeState& valueTreeState;

    typedef AudioProcessorValueTreeState::SliderAttachment SliderAttachment;

    Slider inputGainSlider;
    Slider mixSlider;
    Slider outputGainSlider;

    Label inputGainLabel;
    Label mixLabel;
    Label outputGainLabel;

    std::unique_ptr<SliderAttachment> inputGainSliderAttachment;
    std::unique_ptr<SliderAttachment> mixSliderAttachment;
    std::unique_ptr<SliderAttachment> outputGainSliderAttachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FuzzFountainAudioProcessorEditor)
};
