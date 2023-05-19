/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
FuzzFountainAudioProcessorEditor::FuzzFountainAudioProcessorEditor (FuzzFountainAudioProcessor& p, AudioProcessorValueTreeState& vts)
    : AudioProcessorEditor (&p), audioProcessor (p), valueTreeState(vts)
{
    mixSlider.setSliderStyle(juce::Slider::Rotary);
    mixSlider.setScrollWheelEnabled(false);
    mixSlider.setTextBoxStyle(Slider::TextBoxBelow, false, 50, 30);
    mixSlider.setSliderStyle(Slider::SliderStyle::RotaryVerticalDrag);
    mixLabel.setText("Mix", NotificationType::dontSendNotification);
    mixLabel.attachToComponent(&mixSlider, false);
    addAndMakeVisible(mixSlider);
    mixSliderAttachment.reset(new SliderAttachment(valueTreeState, "mix", mixSlider));

    outputGainSlider.setSliderStyle(juce::Slider::Rotary);
    outputGainSlider.setScrollWheelEnabled(false);
    outputGainSlider.setTextBoxStyle(Slider::TextBoxBelow, false, 50, 30);
    outputGainSlider.setSliderStyle(Slider::SliderStyle::RotaryVerticalDrag);
    outputGainLabel.setText("Output Gain", NotificationType::dontSendNotification);
    outputGainLabel.attachToComponent(&outputGainSlider, false);
    addAndMakeVisible(outputGainSlider);
    outputGainSliderAttachment.reset(new SliderAttachment(valueTreeState, "outputGain", outputGainSlider));

    inputGainSlider.setSliderStyle(juce::Slider::Rotary);
    inputGainSlider.setScrollWheelEnabled(false);
    inputGainSlider.setTextBoxStyle(Slider::TextBoxBelow, false, 50, 30);
    inputGainSlider.setSliderStyle(Slider::SliderStyle::RotaryVerticalDrag);
    inputGainLabel.setText("Input Gain", NotificationType::dontSendNotification);
    inputGainLabel.attachToComponent(&inputGainSlider, false);
    addAndMakeVisible(inputGainSlider);
    inputGainSliderAttachment.reset(new SliderAttachment(valueTreeState, "inputGain", inputGainSlider));

    setSize(600, 450);
}

FuzzFountainAudioProcessorEditor::~FuzzFountainAudioProcessorEditor()
{
}

//==============================================================================
void FuzzFountainAudioProcessorEditor::paint (juce::Graphics& g)
{
    g.fillAll(getLookAndFeel().findColour(juce::ResizableWindow::backgroundColourId));
}

void FuzzFountainAudioProcessorEditor::resized()
{
    auto area = getBounds().reduced(20);
    auto sliderWidth = area.getWidth() / 4;
    auto sliderHeight = area.getHeight() / 2;

    auto topRow = area.removeFromTop(sliderHeight);

    inputGainSlider.setBounds(topRow.removeFromLeft(sliderWidth).reduced(30));
    mixSlider.setBounds(topRow.removeFromLeft(sliderWidth).reduced(30));

    area.removeFromLeft(sliderWidth / 2);
    outputGainSlider.setBounds(area.removeFromLeft(sliderWidth).reduced(30));

    //Eigen test

    Eigen::MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = 2;

    Eigen::MatrixXd m2(2, 1);
    m2(0, 0) = 7;
    m2(1, 0) = -8;

    Eigen::MatrixXd m3(2, 3);
    m3.topLeftCorner(m.rows(), m.cols()) = m;

    std::ostringstream outStream;
    outStream << m3 << std::endl;
    DBG(outStream.str());

    //m3.topRightCorner(m2.rows(), m2.cols()) = m2;
    m3.bottomRightCorner(m2.rows(), m2.cols()) = m2;
    outStream << m3 << std::endl;
    DBG(outStream.str()); 
}
