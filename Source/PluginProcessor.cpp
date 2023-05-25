/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

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

    for (int ch = 0; ch < 2; ch++)
        diodeClipperCircuit[ch] = std::make_unique<DiodeClipperCircuit>();
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

    for(int ch = 0; ch < 2; ch++)
        diodeClipperCircuit[ch]->prepare((float)sampleRate * std::powf(2.0f, float(oversampleFactor))); 
}

void FuzzFountainAudioProcessor::releaseResources()
{
    dryWetMixer.reset();
    inputGain.reset();
    outputGain.reset();
    oversampling.reset();

    for (int ch = 0; ch < 2; ch++)
        diodeClipperCircuit[ch]->reset();
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

    //------Non linear dsp start--------
    auto osBlock = oversampling.processSamplesUp(block);

    for (int ch = 0; ch < 2; ch++)
        diodeClipperCircuit[ch]->process(osBlock.getChannelPointer(ch), osBlock.getNumSamples());

    oversampling.processSamplesDown(block);
        
    //------Non linear dsp end--------

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
