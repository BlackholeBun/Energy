#include "daisy_patch.h"
#include "daisysp.h"
#include "./src/EnergyOsc.hpp"
#include "./Energy.hpp"

using namespace daisy;
using namespace daisysp;

DaisyPatch hw;
Parameter Mass, SpeedOfLight, M_Osc, C_Osc;


// Constants
static const int N_POLY = 16;

float Out[N_POLY];


// Need to save, with reset
FMOp oscM[N_POLY];
FMOp oscC[N_POLY];
int routing;// routing of knob 1. 
	// 0 is independant (i.e. blue only) (bottom light, light index 0),
	// 1 is control (i.e. blue and yellow) (top light, light index 1),
	// 2 is spread (i.e. blue and inv yellow) (middle, light index 2)
int plancks[2];// index is left/right, value is: 0 = not quantized, 1 = semitones, 2 = 5th+octs, 3 = adds -10V offset
int modtypes[2];// index is left/right, value is: {0 to 3} = {bypass, add, amp}
int cross;// cross momentum active or not

// No need to save, with reset
int numChan = 1;
float feedbacks[2][N_POLY];
float modSignals[2][N_POLY];

// No need to save, no reset
//RefreshCounter refresh;
int RefreshCounter = 0;
SchmittTrigger routingTrigger;
SchmittTrigger planckTriggers[2];
SchmittTrigger modtypeTriggers[2];
SchmittTrigger crossTrigger;
SlewLimiter multiplySlewers[N_POLY];
	

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	hw.ProcessAllControls();
	for (size_t i = 0; i < size; i++)
	{
		/*
		out[0][i] = in[0][i];
		out[1][i] = in[1][i];
		out[2][i] = in[2][i];
		out[3][i] = in[3][i];
		*/

		out[0][i] = process(in[0][i], in[1][i]);
	}
}

int main(void)
{
	hw.Init();

	Mass.Init(hw.controls[hw.CTRL_1], 10.0, 110.0f, Parameter::LINEAR);
	SpeedOfLight.Init(hw.controls[hw.CTRL_2], 10.0, 110.0f, Parameter::LINEAR);
	M_Osc.Init(hw.controls[hw.CTRL_3], 10.0, 110.0f, Parameter::LINEAR);
	C_Osc.Init(hw.controls[hw.CTRL_4], 10.0, 110.0f, Parameter::LINEAR);
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAdc();
	hw.StartAudio(AudioCallback);

	freqctrl.Init(
        patch.controls[patch.CTRL_1], 10.0, 110.0f, Parameter::LINEAR);
	while(1) {
		hw.DisplayControls(false);
	}
}

float process(float Multiply, float VpO) {

	float outMixer = 0.0f;
	// user inputs
	/*
	if (refresh.processInputs()) {
		numChan = std::max(1, inputs[FREQCV_INPUT].getChannels());
		numChan = std::min(numChan, (int)N_POLY);
		outputs[ENERGY_OUTPUT].setChannels(numChan);

		// routing
		if (routingTrigger.process(params[ROUTING_PARAM].getValue())) {
			if (++routing > 2)
				routing = 0;
		}
		
		// plancks
		for (int i = 0; i < 2; i++) {
			if (planckTriggers[i].process(params[PLANCK_PARAMS + i].getValue())) {
				if (++plancks[i] > 3)
					plancks[i] = 0;
			}
		}
		
		// modtypes
		for (int i = 0; i < 2; i++) {
			if (modtypeTriggers[i].process(params[MODTYPE_PARAMS + i].getValue())) {
				if (++modtypes[i] > 2)
					modtypes[i] = 0;
			}
		}
		
		// cross
		if (crossTrigger.process(params[CROSS_PARAM].getValue())) {
			if (++cross > 1)
				cross = 0;
		}
	}// userInputs refresh
	*/
	
	
	// main signal flow
	// ----------------		
	for (int c = 0; c < numChan; c++) {
		// pitch modulation and feedbacks
		if ((refreshCounter & 0x3) == (c & 0x3)) {
			// stagger0 updates channels 0, 4, 8,  12
			// stagger1 updates channels 1, 5, 9,  13
			// stagger2 updates channels 2, 6, 10, 14
			// stagger3 updates channels 3, 7, 11, 15
			calcModSignals(c);// voct modulation, a given channel is updated at sample_rate / 4
			calcFeedbacks(c);// feedback (momentum), a given channel is updated at sample_rate / 4
		}
		
		/* Not needed, VCV Rack Req
		if (!outputs[ENERGY_OUTPUT].isConnected()) {// this is placed here such that feedbacks and mod signals of chan 0 are always calculated, since they are used in lights
			break;
		}
		*/
		
		// vocts
		float vocts[2] = {modSignals[0][c] + VpO, modSignals[1][c] + VpO};
		
		// oscillators
		float oscMout = oscM[c].step(vocts[0], feedbacks[0][c] * 0.3f);
		float oscCout = oscC[c].step(vocts[1], feedbacks[1][c] * 0.3f);
		
		// multiply 
		float slewInput = (clamp(Multiply / 10.0f, 0.0f, 1.0f));

		float multiplySlewValue = multiplySlewers[c].next(slewInput) * 0.2f;
		
		// final attenuverters
		float attv1 = oscCout * oscCout * multiplySlewValue;
		float attv2 = attv1 * oscMout * 0.2f;
		
		//write mi
		// output
		outMixer += attv * (1.0f / numChan);
	}


	refreshCounter++;

	if (refreshCounter > 15){
		refreshCounter = 0;
	}

	return outMixer;

	/*
	// lights
	if (refresh.processLights()) {
		// routing
		for (int i = 0; i < 3; i++)
			lights[ROUTING_LIGHTS + i].setBrightness(routing == i ? 1.0f : 0.0f);
		
		for (int i = 0; i < 2; i++) {
			// plancks (was white/blue/red), now BlueYellowWhite
			lights[PLANCK_LIGHTS + i * 3 + 2].setBrightness(plancks[i] == 1 ? 1.0f : 0.0f);// white
			lights[PLANCK_LIGHTS + i * 3 + 0].setBrightness(plancks[i] == 2 ? 1.0f : 0.0f);// blue
			lights[PLANCK_LIGHTS + i * 3 + 1].setBrightness(plancks[i] == 3 ? 1.0f : 0.0f);// yellow (was red)
			
			// modtypes
			lights[ADD_LIGHTS + i].setBrightness(modtypes[i] == 1 ? 1.0f : 0.0f);
			lights[AMP_LIGHTS + i].setBrightness(modtypes[i] == 2 ? 1.0f : 0.0f);
			
			// momentum (cross)
			lights[MOMENTUM_LIGHTS + i].setBrightness(feedbacks[i][0]);// lights show first channel only when poly

			// freq
			float modSignalLight = modSignals[i][0] / 3.0f;
			lights[FREQ_ROUTING_LIGHTS + 2 * i + 0].setBrightness(modSignalLight);// blue diode
			lights[FREQ_ROUTING_LIGHTS + 2 * i + 1].setBrightness(-modSignalLight);// yellow diode
		}
		
		// cross
		lights[CROSS_LIGHT].setBrightness(cross == 1 ? 1.0f : 0.0f);

	}// lightRefreshCounter
	*/
	
}// step()

inline float calcFreqKnob(int osci) {
	if (plancks[osci] == 0)// off (smooth)
		return params[FREQ_PARAMS + osci].getValue();
	if (plancks[osci] == 1)// semitones
		return std::round(params[FREQ_PARAMS + osci].getValue() * 12.0f) / 12.0f;
	if (plancks[osci] == 3)// -10V offset
		return params[FREQ_PARAMS + osci].getValue() - 10.0f;
	// 5ths and octs (plancks[osci] == 2)
	int retcv = (int)std::round((params[FREQ_PARAMS + osci].getValue() + 3.0f) * 2.0f);
	if ((retcv & 0x1) != 0)
		return (float)(retcv)/2.0f - 3.0f + 0.08333333333f;
	return (float)(retcv)/2.0f - 3.0f;
}

inline void calcModSignals(int chan) {
	for (int osci = 0; osci < 2; osci++) {
		float freqValue = calcFreqKnob(osci);
		if (modtypes[osci] == 0 || !inputs[FREQCV_INPUTS + osci].isConnected()) {// bypass
			modSignals[osci][chan] = freqValue;
		}
		else {
			int chanIn = std::min(inputs[FREQCV_INPUTS + osci].getChannels() - 1, chan);
			if (modtypes[osci] == 1) {// add
				modSignals[osci][chan] = freqValue + inputs[FREQCV_INPUTS + osci].getVoltage(chanIn);
			}
			else {// amp
				modSignals[osci][chan] = freqValue * (clamp(inputs[FREQCV_INPUTS + osci].getVoltage(chanIn), 0.0f, 10.0f) / 10.0f);
			}
		}
	}
	if (routing == 1) {
		modSignals[1][chan] += modSignals[0][chan];
	}
	else if (routing == 2) {
		modSignals[1][chan] -= modSignals[0][chan];
	}
}

inline void calcFeedbacks(int chan) {
	float moIn[2]; 	
	for (int osci = 0; osci < 2; osci++) {
		moIn[osci] = 0.0f;
		if (inputs[MOMENTUM_INPUTS + osci].isConnected()) {
			int chanIn = std::min(inputs[MOMENTUM_INPUTS + osci].getChannels() - 1, chan);
			moIn[osci] = inputs[MOMENTUM_INPUTS + osci].getVoltage(chanIn);
		}
		feedbacks[osci][chan] = params[MOMENTUM_PARAMS + osci].getValue();
	}
	
	if (cross == 0) {
		feedbacks[0][chan] += moIn[0] * 0.1f;
		feedbacks[1][chan] += moIn[1] * 0.1f;
	}
	else {// cross momentum
		if (moIn[0] > 0)
			feedbacks[0][chan] += moIn[0] * 0.2f;
		else 
			feedbacks[1][chan] += moIn[0] * -0.2f;
		if (moIn[1] > 0)
			feedbacks[1][chan] += moIn[1] * 0.2f;
		else 
			feedbacks[0][chan] += moIn[1] * -0.2f;
	}
	feedbacks[0][chan] = clamp(feedbacks[0][chan], 0.0f, 1.0f);
	feedbacks[1][chan] = clamp(feedbacks[1][chan], 0.0f, 1.0f);
}