#define PATCH
//#define SUBMODULE

#ifdef PATCH
#include "daisy_patch.h"
#endif

#ifdef SUBMODULE
#include "daisy_patch_sm.h"
#endif

#include "daisysp.h"
#include "./src/EnergyOsc.hpp"
#include "./Energy.hpp"
#include <stdlib.h>

using namespace daisy;
using namespace daisysp;
#ifdef SUBMODULE
using namespace patch_sm;
#endif

#ifdef PATCH
DaisyPatch hw;
Parameter CTRL_1, CTRL_2, CTRL_3, CTRL_4;
Parameter * Params[4] = {&CTRL_1, &CTRL_2, &CTRL_3, &CTRL_4};
ParamIds paramMap[4];// map of param indexes to control indexes
#endif

#ifdef SUBMODULE
DaisyPatchSM hw;
ParamIds paramMap[12];// map of param indexes to control indexes
#endif

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
float oscFreqKnobs[2];
float oscFreqCV[2];
float momentumKnob[2];
float momentumCV[2];
float vpO, multiply;

// No need to save, with reset
int numChan = 1;
float feedbacks[2][N_POLY];
float modSignals[2][N_POLY];

// No need to save, no reset
//RefreshCounter refresh;
int refreshCounter = 0;
SchmittTrigger routingTrigger;
SchmittTrigger planckTriggers[2];
SchmittTrigger modtypeTriggers[2];
SchmittTrigger crossTrigger;
SlewLimiter multiplySlewers[N_POLY];
	

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	/*
	static float massVal;
	static float speedOfLightVal;
	static float multiplyVal;
	static float vpOVal;
	*/
	hw.ProcessAllControls();
	for (size_t i = 0; i < size; i++)
	{
		/*
		out[0][i] = in[0][i];
		out[1][i] = in[1][i];
		out[2][i] = in[2][i];
		out[3][i] = in[3][i];

		massVal = Mass.Update(in[Mass.Index()][i]); // Todo use Mass value
		speedOfLightVal = SpeedOfLight.Update(in[SpeedOfLight.Index()][i]); // Todo use SpeedOfLight value
		multiplyVal = Multiply.Update(in[Multiply.Index()][i]); 
		vpOVal = VpO.Update(in[VpO.Index()][i]);
		*/

		for (int j = 0; j < 4; j++) {

			#ifdef PATCH
			ParamUpdate(Params[j]->Process(), paramMap[j]);
			#endif
			#ifdef SUBMODULE
			ParamUpdate(hw.GetAdcValue(j), paramMap[j]);
			#endif
		}

		//out[0][i] = process(multiplyVal,vpOVal);
		out[0][i] = process(multiply,vpO);
		out[1][i] = out[0][i];
		out[2][i] = out[0][i];
		out[3][i] = out[0][i];
	}
}

int main(void)
{
	hw.Init();

	#ifdef PATCH
	CTRL_1.Init(hw.controls[hw.CTRL_1], -10.0f, 10.0f, Parameter::LINEAR);
	CTRL_2.Init(hw.controls[hw.CTRL_2], -10.0f, 10.0f, Parameter::LINEAR);
	CTRL_3.Init(hw.controls[hw.CTRL_3], -10.0f, 10.0f, Parameter::LINEAR);
	CTRL_4.Init(hw.controls[hw.CTRL_4], -10.0f, 10.0f, Parameter::LINEAR);
	#endif
	// VpO.Init(0, -3.0f, 7.0f, AudioRateParam::LINEAR);
	// Multiply.Init(1, -10.0f, 10.0f, AudioRateParam::LINEAR);
	// Mass.Init(2, -10.0f, 10.0f, AudioRateParam::LINEAR);
	// SpeedOfLight.Init(3, -10.0f, 10.0f, AudioRateParam::LINEAR);

	oscFreqKnobs[0] = 0.0f;
	oscFreqKnobs[1] = 0.0f;
	oscFreqCV[0] = 0.0f;
	oscFreqCV[1] = 0.0f;
	momentumKnob[0] = 0.0f;
	momentumKnob[1] = 0.0f;
	momentumCV[0] = 0.0f;
	momentumCV[1] = 0.0f;
	vpO = 0.0f;
	multiply = 3.0f;
	#ifdef PATCH
	paramMap[0] = VpO;
	paramMap[1] = Multiply;
	paramMap[2] = oscFreqCV1;
	paramMap[3] = oscFreqCV2;
	#endif
	#ifdef SUBMODULE
	paramMap[0] = oscFreqKnob1;
	paramMap[1] = oscFreqKnob2;
	paramMap[2] = momentumKnob1;
	paramMap[3] = momentumKnob2;
	paramMap[4] = VpO;
	paramMap[5] = Multiply;
	paramMap[6] = oscFreqCV1;
	paramMap[7] = oscFreqCV2;
	paramMap[8] = momentumCV1;
	paramMap[9] = momentumCV2;
	#endif



	for (int c = 0; c < N_POLY; c++) {
		oscM[c].construct(48000.0f);
		oscC[c].construct(48000.0f);
		feedbacks[0][c] = 0.0f;
		feedbacks[1][c] = 0.0f;
	}

	hw.SetAudioBlockSize(1); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAdc();
	hw.StartAudio(AudioCallback);

	while(1) {
		//hw.DisplayControls(false);
		#ifdef PATCH
		UpdateOled();
		#endif
	}
}

void ParamUpdate(float value, int id){


	switch (id)
	{
		case 0:
			 vpO = value;
			break;
		case 1:
			multiply = value;
			break;
		case 2:
			oscFreqCV[0] = value;
			break;
		case 3:
			oscFreqCV[1] = value;
			break;
		case 4:
			momentumCV[0] = value;
			break;
		case 5:
			momentumCV[1] = value;
			break;
		case 6:
			oscFreqKnobs[0] = value;
			break;
		case 7:	
			oscFreqKnobs[1] = value;
			break;
		case 8:
			momentumKnob[0] = value;
			break;
		case 9:
			momentumKnob[1] = value;
			break;
		default:
			break;
	}
}
/*  D
|******************|
|    M    |    C   |
| BAR GRAPHS OF CV |
| M - Add | C - Mul|
| P - 5_O | P = semi|
|    X MOD - M     |
********************
*/

float process(float Multiply, float VpO) {

	float outMixer = 0.0f;
	// user inputs
	/*
	if (refresh.processInputs()) {
		numChan = std::max(1, inputs[FREQCV_INPUT].getChannels());
		numChan = std::min(numChan, (int)N_POLY);
		outputs[ENERGY_OUTPUT].setChannels(numChan);

		// routing
		if (routingTrigger.process(oscFreqKnobs[ROUTING_PARAM].getValue())) {
			if (++routing > 2)
				routing = 0;
		}
		
		// plancks
		for (int i = 0; i < 2; i++) {
			if (planckTriggers[i].process(oscFreqKnobs[PLANCK_PARAMS + i].getValue())) {
				if (++plancks[i] > 3)
					plancks[i] = 0;
			}
		}
		
		// modtypes
		for (int i = 0; i < 2; i++) {
			if (modtypeTriggers[i].process(oscFreqKnobs[MODTYPE_PARAMS + i].getValue())) {
				if (++modtypes[i] > 2)
					modtypes[i] = 0;
			}
		}
		
		// cross
		if (crossTrigger.process(oscFreqKnobs[CROSS_PARAM].getValue())) {
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
		float vocts[2] = {modSignals[0][c] + vpO, modSignals[1][c] + vpO};
		
		// oscillators
		float oscMout = oscM[c].step(vocts[0], feedbacks[0][c] * 0.3f);
		float oscCout = oscC[c].step(vocts[1], feedbacks[1][c] * 0.3f);
		
		// multiply 
		float slewInput = (clamp(multiply / 10.0f, 0.0f, 1.0f));

		float multiplySlewValue = multiplySlewers[c].next(slewInput) * 0.2f;
		
		// final attenuverters
		float attv1 = oscCout * oscCout * multiplySlewValue;
		float attv2 = attv1 * oscMout * 0.2f;
		
		//write mi
		// output
		outMixer += attv2 * (1.0f / numChan);
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

float calcFreqKnob(int osci) {
	if (plancks[osci] == 0)// off (smooth)
		return oscFreqKnobs[osci];
	if (plancks[osci] == 1)// semitones
		return std::round(oscFreqKnobs[osci] * 12.0f) / 12.0f;
	if (plancks[osci] == 3)// -10V offset
		return oscFreqKnobs[osci] - 10.0f;
	// 5ths and octs (plancks[osci] == 2)
	int retcv = (int)std::round((oscFreqKnobs[osci] + 3.0f) * 2.0f);
	if ((retcv & 0x1) != 0)
		return (float)(retcv)/2.0f - 3.0f + 0.08333333333f;
	return (float)(retcv)/2.0f - 3.0f;
}

void calcModSignals(int chan) {
	for (int osci = 0; osci < 2; osci++) {
		float freqValue = calcFreqKnob(osci);
		if (modtypes[osci] == 0 ) {// bypass
			modSignals[osci][chan] = freqValue;
		}
		else {
			//int chanIn = std::min(inputs[osci].getChannels() - 1, chan);
			if (modtypes[osci] == 1) {// add
				modSignals[osci][chan] = freqValue + oscFreqCV[osci];
			}
			else {// amp
				modSignals[osci][chan] = freqValue * (clamp(oscFreqCV[osci], 0.0f, 10.0f) / 10.0f);
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

void calcFeedbacks(int chan) {
	float moIn[2]; 	
	for (int osci = 0; osci < 2; osci++) {
		moIn[osci] = 0.0f;
		//if (inputs[MOMENTUM_INPUTS + osci].isConnected()) {
			//int chanIn = std::min(inputs[MOMENTUM_INPUTS + osci].getChannels() - 1, chan);
			moIn[osci] = momentumCV[osci];
		//}
		feedbacks[osci][chan] = momentumKnob[osci];
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

#ifdef PATCH
void UpdateOled()
{
    hw.display.Fill(false);

    //std::string str  = "!";
    //char*       cstr = &str[0];

	const int charWidth = 7;
	const int charHeight = 10;


    hw.display.SetCursor(0, charHeight);
    hw.display.WriteChar('M', Font_7x10, true);
	hw.display.SetCursor(65, charHeight);
	hw.display.WriteChar('C', Font_7x10, true);
	hw.display.DrawLine(64,0,64,64, true);
	// Set to scaled CV for M
	//hw.display.DrawRect
	// Set to scaled CV for C

	hw.display.SetCursor(0, charHeight * 2 + 1);
	hw.display.WriteString("M - ", Font_7x10, true);
	writeModToDisplay(modtypes[0]);

	hw.display.SetCursor(65, charHeight * 2 + 1);
	hw.display.WriteString("C - ", Font_7x10, true);
	writeModToDisplay(modtypes[1]);

	hw.display.SetCursor(0, charHeight * 3 + 3);
	hw.display.WriteString("P - ", Font_7x10, true);
	
	writeQuantToDisplay(plancks[0]);
	hw.display.SetCursor(65,charHeight * 3 + 3);
	hw.display.WriteString("P - ", Font_7x10, true);
	writeQuantToDisplay(plancks[1]);



	


    hw.display.Update();
}

void writeQuantToDisplay(int mode){
	switch (mode)
	{
		case 0: // 
			hw.display.WriteString("Off", Font_7x10, true);
			break;
		case 1: // 
			hw.display.WriteString("Semi", Font_7x10, true);
			break;
		case 2: // 
			hw.display.WriteString("5_O", Font_7x10, true);
			break;
		default:
			break;
	}
}

void writeModToDisplay(int mode){
	switch (mode)
	{
	case 0:
		hw.display.WriteString("Add", Font_7x10, true);
		break;
	case 1:
		hw.display.WriteString("Amp", Font_7x10, true);
		break;
	
	default:
		break;
	}
}
#endif