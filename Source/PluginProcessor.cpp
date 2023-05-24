/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"
#define _USE_MATH_DEFINES
#include <math.h>

//==============================================================================
FuzzFountainAudioProcessor::FuzzFountainAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
    : AudioProcessor(BusesProperties()
#if ! JucePlugin_IsMidiEffect
#if ! JucePlugin_IsSynth
        .withInput("Input", juce::AudioChannelSet::stereo(), true)
#endif
        .withOutput("Output", juce::AudioChannelSet::stereo(), true)
#endif
    ), parameters(*this, nullptr, Identifier("sampld"),
        {
            std::make_unique<AudioParameterFloat>("inputGain", "Input Gain", NormalisableRange<float>(-80.0f, 20.0f,0.1f, 2.0f), 0.0f),
            std::make_unique<AudioParameterFloat>("mix", "Mix", NormalisableRange<float>(0.0f, 10.0f,0.1f), 10.0f),
            std::make_unique<AudioParameterFloat>("outputGain", "Output Gain", NormalisableRange<float>(-30.0f, 30.0f,0.1f), 0.0f),

        })
#endif
{
    inputGainParameter = parameters.getRawParameterValue("inputGain");
    mixParameter = parameters.getRawParameterValue("mix");
    outputGainParameter = parameters.getRawParameterValue("outputGain");

    //Test: build circuit + nonrealtime test with sine funciton
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
    nonLinearComponents->push_back(new DiodeNLEQ());

    CircuitBase rcDiodeClipper(
        std::move(resistors), std::move(capacitors), 9, std::move(NR), 
        std::move(Nv), std::move(Nx), std::move(Nu), std::move(Nn), 
        std::move(No), std::move(nonLinearComponents), 1, false);

    double fs_test = 44100.0;
    double fc_test = 400.0;

    rcDiodeClipper.prepare(fs_test);

    //test first 100 samples of sin
    for (int i = 0; i < 100; i++)
    {
        double vin = std::sin(2 * M_PI * fc_test * i / fs_test);
        //DBG("vin = " + String(vin));
        rcDiodeClipper.process(&vin, 1);
        double vo = vin;
        DBG("vo = " + String(vo));
    }
}

FuzzFountainAudioProcessor::~FuzzFountainAudioProcessor()
{
}

//==============================================================================
const juce::String FuzzFountainAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool FuzzFountainAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool FuzzFountainAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool FuzzFountainAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double FuzzFountainAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int FuzzFountainAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int FuzzFountainAudioProcessor::getCurrentProgram()
{
    return 0;
}

void FuzzFountainAudioProcessor::setCurrentProgram (int index)
{
}

const juce::String FuzzFountainAudioProcessor::getProgramName (int index)
{
    return {};
}

void FuzzFountainAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================
void FuzzFountainAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.maximumBlockSize = samplesPerBlock;
    spec.numChannels = getTotalNumOutputChannels();

    dryWetMixer.prepare(spec);
    dryWetMixer.setMixingRule(juce::dsp::DryWetMixingRule::sin3dB);

    inputGain.prepare(spec);
    outputGain.prepare(spec);

    oversampling.initProcessing(samplesPerBlock);

    parallelBuffer.setSize(getTotalNumOutputChannels(), samplesPerBlock);
}

void FuzzFountainAudioProcessor::releaseResources()
{
    dryWetMixer.reset();
    inputGain.reset();
    outputGain.reset();
    oversampling.reset();
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool FuzzFountainAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif

void FuzzFountainAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    const auto numSamples = buffer.getNumSamples();

    dsp::AudioBlock<float> block(buffer);
    auto context = juce::dsp::ProcessContextReplacing<float>(block);

    // input gain stage
    inputGain.setGainDecibels(*inputGainParameter);
    inputGain.process(context);

    // copy original dry signal into a buffer
    
    jassert(buffer.getNumSamples() == parallelBuffer.getNumSamples());
    jassert(buffer.getNumChannels() == parallelBuffer.getNumChannels());

    parallelBuffer.makeCopyOf(buffer, true);
    dsp::AudioBlock<float> parallelBlock(parallelBuffer);

    //------start of di parallel processing---------
    dryWetMixer.setWetMixProportion(jmap((float)*mixParameter, 0.0f, 10.0f, 0.0f, 1.0f));
    dryWetMixer.pushDrySamples(parallelBlock);

    //------end of di parallel processing-----------

    // mix signals
    dryWetMixer.mixWetSamples(block);

    // Output Gain stage
    outputGain.setGainDecibels(*outputGainParameter);
    outputGain.process(context);
}

//==============================================================================
bool FuzzFountainAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* FuzzFountainAudioProcessor::createEditor()
{
    return new FuzzFountainAudioProcessorEditor (*this, parameters);
}

//==============================================================================
void FuzzFountainAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    auto state = parameters.copyState();
    std::unique_ptr<XmlElement> xml(state.createXml());
    copyXmlToBinary(*xml, destData);
}

void FuzzFountainAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    std::unique_ptr<XmlElement> xmlState(getXmlFromBinary(data, sizeInBytes));

    if (xmlState.get() != nullptr)
        if (xmlState->hasTagName(parameters.state.getType()))
            parameters.replaceState(ValueTree::fromXml(*xmlState));
}

//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new FuzzFountainAudioProcessor();
}
