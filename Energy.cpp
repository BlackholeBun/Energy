#define PATCH
//#define SUBMODULE

#ifdef PATCH
#include "daisy_patch.h"
#include "util/CpuLoadMeter.h"
#endif

#ifdef SUBMODULE
#include "daisy_patch_sm.h"
#endif

#include "daisysp.h"
#include "./src/EnergyOsc.hpp"
#include "./Energy.hpp"
#include "Energy.hpp"

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
int menuIndex = 0;
int menuPage = 0;
bool cvChangeMode = false;
int changeParam = 0;
const int charWidth = 7;
const int charHeight = 10;
const int lineHeight = 16;
const int screenWidth = 128;
const int screenHeight = 64;
const int lineOffset = (lineHeight - charHeight) / 2;
const int firstLineY = 0 + lineOffset;//lineHeight;
const int secondLineY = lineHeight * 1 + lineOffset;
const int thirdLineY = lineHeight * 2 + lineOffset;
const int fourthLineY = lineHeight * 3 + lineOffset;
const int column1x = 1;
const int column2x = 66;
int encoderDebounce = 0;
bool encoderLast = false;
CpuLoadMeter cpuMeter;
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
int unisonVoices = 0;
float unisonDetune = -0.001f;

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
	cpuMeter.OnBlockStart();
	hw.ProcessAllControls();
	
	#ifdef PATCH
	if (cpuMeter.GetAvgCpuLoad() > 0.9f){
		numChan--;
		unisonVoices--;
		cpuMeter.Reset();
	}
	int temp = hw.encoder.Increment();
	if (!cvChangeMode){
		menuIndex += temp;
		if (menuIndex < 0) {
			menuIndex = 21;
		} else if (menuIndex > 21) {
			menuIndex = 0;
		}
		menuPage = ((menuIndex / 10) * 2) + ((menuIndex % 10) / 6);
	} else {
		if ((changeParam < 10) || (changeParam > 13)){
			ParamUpdate(temp, changeParam, true);
		} else {
			paramMap[changeParam - 10] = static_cast<ParamIds>(static_cast<int>(paramMap[changeParam - 10] + temp));
			if (paramMap[changeParam - 10] < 0) {
				paramMap[changeParam - 10] = VpO;
			} else if (paramMap[changeParam - 10] > 9) {
				paramMap[changeParam - 10] = momentumKnob2;
			}
		}
	}
	if (hw.encoder.Pressed()){
		if (!encoderLast){
			DoMenu();
			encoderLast = true;
			encoderDebounce = 1000;
		} else {
			encoderDebounce = 1000;
		}
	}
	if (encoderLast){
		encoderDebounce--;
		if (encoderDebounce <= 0){
			encoderLast = false;
		}
	}
	#endif

	
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

		#ifdef PATCH
		for (int j = 0; j < 4; j++) {
			ParamUpdate(Params[j]->Process(), paramMap[j], false);
		}
		#endif

		#ifdef SUBMODULE
		for (int j = 0; j < 10; j++) {
			ParamUpdate(hw.GetAdcValue(j), paramMap[j], false);
		}
		#endif

		//out[0][i] = process(multiplyVal,vpOVal);
		out[0][i] = process(multiply,vpO);
		out[1][i] = out[0][i];
		#ifdef PATCH
		out[2][i] = out[0][i];
		out[3][i] = out[0][i];
		#endif
	}
	cpuMeter.OnBlockEnd();
}

int main(void)
{
	hw.Init();
	cpuMeter.Init(hw.AudioSampleRate(), 1);

	#ifdef PATCH
	CTRL_1.Init(hw.controls[hw.CTRL_1], 0.0f, 10.0f, Parameter::LINEAR);
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
	paramMap[1] = momentumKnob1;
	paramMap[2] = oscFreqKnob1;
	paramMap[3] = oscFreqKnob2;
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

void ParamUpdate(float value, int id, bool inc){

	value = clamp(value, -10.0f, 10.0f);
	
	switch (id)
	{
		case 0:
			 vpO = ((value/2) - 3) + (inc ? vpO: 0.0f);
			break;
		case 1:
			multiply = value + (inc ? multiply: 0.0f);
			break;
		case 2:
			oscFreqCV[0] = value + (inc ? oscFreqCV[0]: 0.0f);
			break;
		case 3:
			oscFreqCV[1] = value + (inc ? oscFreqCV[1]: 0.0f);
			break;
		case 4:
			momentumCV[0] = value + (inc ? momentumCV[0]: 0.0f);
			break;
		case 5:
			momentumCV[1] = value + (inc ? momentumCV[1]: 0.0f);
			break;
		case 6:
			oscFreqKnobs[0] = value + (inc ? oscFreqKnobs[0]: 0.0f);
			break;
		case 7:	
			oscFreqKnobs[1] = value + (inc ? oscFreqKnobs[1]: 0.0f);
			break;
		case 8:
			momentumKnob[0] = value + (inc ? momentumKnob[0]: 0.0f);
			break;
		case 9:
			momentumKnob[1] = value + (inc ? momentumKnob[1]: 0.0f);
			break;
		case 14:
			unisonVoices = ((int)value + (inc ? unisonVoices: 0)) & 0xf;
			numChan = std::max(1, unisonVoices);
			break;
		case 15:
			unisonDetune = (value/1000.0f) + (inc ? unisonDetune: 0.0f);
			break;
		default:
			break;
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

	// pitch modulation and feedbacks
	float mixratio = 1.0f / (float)numChan;
	// main signal flow
	// ----------------		

	for (int c = 0; c < numChan; c++) {
		
		if (refreshCounter && 0x3) {	
			calcModSignals(c);// voct modulation, a given channel is updated at sample_rate / 4
			calcFeedbacks(c);// feedback (momentum), a given channel is updated at sample_rate / 4
		}
		/* Not needed, VCV Rack Req
		if (!outputs[ENERGY_OUTPUT].isConnected()) {// this is placed here such that feedbacks and mod signals of chan 0 are always calculated, since they are used in lights
			break;
		}
		*/
		
		// vocts
		float cycletune = (float)vpO + ((float)unisonDetune * (float)c * (c % 2 ? -1.0f : 1.0f));
		float vocts[2] = {modSignals[0][c] + cycletune, modSignals[1][c] + cycletune};
		
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
		outMixer += attv2 * mixratio;
	}


	refreshCounter++;
	refreshCounter &= 0xf;

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
void DoMenu(){
	switch (menuIndex)
	{
		case 0: modtypes[0] = (modtypes[0] + 1) % 2; break;
		case 1: plancks[0] = (plancks[0] + 1) % 3; break;
		case 2: routing = (routing + 1) % 3; break;
		case 3: modtypes[1] = (modtypes[1] + 1) % 2; break;
		case 4: plancks[1] = (plancks[1] + 1) % 3; break;
		case 5: cross = (cross + 1) % 2; break;
		case 6: 
			cvChangeMode = !cvChangeMode;
			changeParam = oscFreqKnob1;
			break;
		case 7: 
			cvChangeMode = !cvChangeMode;
			changeParam = oscFreqKnob2;
			break;
		case 8:
			cvChangeMode = !cvChangeMode;
			changeParam = momentumKnob1;
			break;
		case 9:
			cvChangeMode = !cvChangeMode;
			changeParam = momentumKnob2;
			break;
		case 10: 
			cvChangeMode = !cvChangeMode;
			changeParam = oscFreqCV1;
			break;
		case 11:
			cvChangeMode = !cvChangeMode;
			changeParam = oscFreqCV2;
			break;
		case 12:
			cvChangeMode = !cvChangeMode;
			changeParam = momentumCV1;
			break;
		case 13:
			cvChangeMode = !cvChangeMode;
			changeParam = momentumCV2;
			break;
		case 14:
			cvChangeMode = !cvChangeMode;
			changeParam = VpO;
			break;
		case 15:
			cvChangeMode = !cvChangeMode;
			changeParam = Multiply;
			break;
		case 16:
			cvChangeMode = !cvChangeMode;
			changeParam = 10;
			break;
		case 17:	
			cvChangeMode = !cvChangeMode;
			changeParam = 11;
			break;
		case 18:
			cvChangeMode = !cvChangeMode;
			changeParam = 12;
			break;
		case 19:
			cvChangeMode = !cvChangeMode;
			changeParam = 13;
			break;
		case 20:
			cvChangeMode = !cvChangeMode;
			changeParam = 14;
			break;
		case 21:
			cvChangeMode = !cvChangeMode;
			changeParam = 15;
			break;
		default: break;
	}
		
}
void UpdateOled()
{
    hw.display.Fill(false);

    //std::string str  = "!";
    //char*       cstr = &str[0];


	switch (menuPage)
	{
	case 0: DrawPage1(); break;
	case 1: DrawPage2(); break;
	case 2: DrawPage3(); break;
	case 3: DrawPage4(); break;
	case 4: DrawPage5(); break;
	default: DrawPage1(); break;
	}

    hw.display.Update();
}

/*  D
|*******************|
| M - 100 | C - 100 |
| M - Add | C - Mul |
| P - 5_O | P = semi|
| R - M   | X - M   |
********************
*/
void DrawPage1(){

	// The first line will display the M and C knob values
	// The second line will display the M and C mod types
	// The third line will display the P and P planck values
	// The fourth line will display the routing and cross values

	// The values on lines 2-4 will be selectable
	hw.display.Fill(false);


	hw.display.DrawLine((screenWidth/2),0,(screenWidth/2),screenHeight, true);
	hw.display.DrawLine(0,(lineHeight*4),screenWidth,(lineHeight*4), true);
	//draw box around selected item
	DrawCursor8(menuIndex + 2, false);

	// Get string values for knob positions
	std::string freq1 = "M - " + std::to_string((int)std::round(oscFreqKnobs[0] * 100.0f));
	std::string freq2 = "c - " + std::to_string((int)std::round(oscFreqKnobs[1] * 100.0f));
	// generate pointers to the strings
	char* freq1c = &freq1[0];
	char* freq2c = &freq2[0];


	// Line 1
	hw.display.SetCursor(column1x, firstLineY);
	hw.display.WriteString(freq1c, Font_7x10, true);
	hw.display.SetCursor(column2x, firstLineY);
	hw.display.WriteString(freq2c, Font_7x10, true);

	// Line 2
	hw.display.SetCursor(column1x, secondLineY);
	hw.display.WriteString("M - ", Font_7x10, menuIndex == 0);
	writeModToDisplay(modtypes[0]);

	hw.display.SetCursor(column2x, secondLineY);
	hw.display.WriteString("C - ", Font_7x10, menuIndex == 3);
	writeModToDisplay(modtypes[1]);

	// Line 3
	hw.display.SetCursor(column1x, thirdLineY);
	hw.display.WriteString("P - ", Font_7x10, menuIndex == 1);
	writeQuantToDisplay(plancks[0]);

	hw.display.SetCursor(column2x, thirdLineY);
	hw.display.WriteString("P - ", Font_7x10, menuIndex == 4);
	writeQuantToDisplay(plancks[1]);

	// Line 4
	hw.display.SetCursor(column1x, fourthLineY);
	hw.display.WriteString("R - ", Font_7x10, menuIndex == 2);
	writeRoutingToDisplay(routing);

	hw.display.SetCursor(column2x, fourthLineY);
	hw.display.WriteString("X - ", Font_7x10, menuIndex == 5);
	writeCrossToDisplay(cross);
	
	hw.display.Update();
}

void DrawPage2(){
	// Page 2 will display the numerical values of the knobs
	// There will be 1 knob per line
	// The selected knob will be highlighted
	// The selected knob will be able to be changed

	hw.display.Fill(false);
	DrawCursor4(menuIndex - 6, cvChangeMode);

	// Get string values for knob positions
	std::string freq1 = "M_Osc - " + std::to_string((int)std::round(oscFreqKnobs[0] * 100.0f));
	std::string freq2 = "C_Osc - " + std::to_string((int)std::round(oscFreqKnobs[1] * 100.0f));
	std::string freq3 = "M_Momentum - " + std::to_string((int)std::round(momentumKnob[0] * 100.0f));
	std::string freq4 = "C_Momentum - " + std::to_string((int)std::round(momentumKnob[1] * 100.0f));
	// generate pointers to the strings
	char* freq1c = &freq1[0];
	char* freq2c = &freq2[0];
	char* freq3c = &freq3[0];
	char* freq4c = &freq4[0];


	// Line 1
	hw.display.SetCursor(column1x, firstLineY);
	hw.display.WriteString(freq1c, Font_7x10, menuIndex == 6);

	// Line 2
	hw.display.SetCursor(column1x, secondLineY);
	hw.display.WriteString(freq2c, Font_7x10, menuIndex == 7);

	// Line 3
	hw.display.SetCursor(column1x, thirdLineY);
	hw.display.WriteString(freq3c, Font_7x10, menuIndex == 8);

	// Line 4
	hw.display.SetCursor(column1x, fourthLineY);
	hw.display.WriteString(freq4c, Font_7x10, menuIndex == 9);

	hw.display.Update();
}

void DrawPage3(){
	// Page 3 will display the numerical values of the CVs
	// There will be 2 CVs per line
	// The selected CV will be highlighted
	// The selected CV will be able to be changed
	// There are a total of 6 CVs

	hw.display.Fill(false);
	DrawCursor8(menuIndex - 10, cvChangeMode);

	// Get string values for CVs
	std::string cv1 = "O_M - " + std::to_string((int)std::round(oscFreqCV[0] * 100.0f));
	std::string cv2 = "O_C - " + std::to_string((int)std::round(oscFreqCV[1] * 100.0f));
	std::string cv3 = "M_M - " + std::to_string((int)std::round(momentumCV[0] * 100.0f));
	std::string cv4 = "M_C - " + std::to_string((int)std::round(momentumCV[1] * 100.0f));
	std::string cv5 = "VpO - " + std::to_string((int)std::round(vpO * 100.0f));
	std::string cv6 = "Mul - " + std::to_string((int)std::round(multiply * 100.0f));

	// generate pointers to the strings
	char* cv1c = &cv1[0];
	char* cv2c = &cv2[0];
	char* cv3c = &cv3[0];
	char* cv4c = &cv4[0];
	char* cv5c = &cv5[0];
	char* cv6c = &cv6[0];

	// Line 1
	hw.display.SetCursor(column1x, firstLineY);
	hw.display.WriteString(cv1c, Font_7x10, menuIndex == 10);
	hw.display.SetCursor(column2x, firstLineY);
	hw.display.WriteString(cv4c, Font_7x10, menuIndex == 13);

	// Line 2
	hw.display.SetCursor(column1x, secondLineY);
	hw.display.WriteString(cv2c, Font_7x10, menuIndex == 11);
	hw.display.SetCursor(column2x, secondLineY);
	hw.display.WriteString(cv5c, Font_7x10, menuIndex == 14);

	// Line 3
	hw.display.SetCursor(column1x, thirdLineY);
	hw.display.WriteString(cv3c, Font_7x10, menuIndex == 12);
	hw.display.SetCursor(column2x, thirdLineY);
	hw.display.WriteString(cv6c, Font_7x10, menuIndex == 15);

	hw.display.Update();

}

void DrawPage4(){
	//paramMap

	// Page 4 will show the mapping of the knobs to parameters
	// there will be 1 parameter per line
	// the selected parameter will be highlighted
	// the selected parameter will be able to be changed

	hw.display.Fill(false);
	DrawCursor4(menuIndex - 16, cvChangeMode);

	// Get string values for parameters
	std::string param1 = "CTRL_1 - " + paramEnumToString( paramMap[0]);
	std::string param2 = "CTRL_2 - " + paramEnumToString( paramMap[1]);
	std::string param3 = "CTRL_3 - " + paramEnumToString( paramMap[2]);
	std::string param4 = "CTRL_4 - " + paramEnumToString( paramMap[3]);

	// generate pointers to the strings
	char* param1c = &param1[0];
	char* param2c = &param2[0];
	char* param3c = &param3[0];
	char* param4c = &param4[0];


	// Line 1
	hw.display.SetCursor(column1x, firstLineY);
	hw.display.WriteString(param1c, Font_7x10, menuIndex == 16);

	// Line 2
	hw.display.SetCursor(column1x, secondLineY);
	hw.display.WriteString(param2c, Font_7x10, menuIndex == 17);

	// Line 3
	hw.display.SetCursor(column1x, thirdLineY);
	hw.display.WriteString(param3c, Font_7x10, menuIndex == 18);

	// Line 4
	hw.display.SetCursor(column1x, fourthLineY);
	hw.display.WriteString(param4c, Font_7x10, menuIndex == 19);

	hw.display.Update();

}
void DrawPage5(){
	//paramMap

	// Page 5 will be for unison settings.
	// The first item will be unison voices and the second will be unison detune

	hw.display.Fill(false);
	DrawCursor4(menuIndex - 16, cvChangeMode);

	// Get string values for parameters
	std::string param1 = "UnVoices - " + std::to_string(unisonVoices);
	std::string param2 = "UnDetune - " + std::to_string((int)std::round(unisonDetune * 1000.0f));
	std::string param3 = "CPULoad - " + std::to_string((int)std::round(cpuMeter.GetAvgCpuLoad() * 100.0f));

	// generate pointers to the strings
	char* param1c = &param1[0];
	char* param2c = &param2[0];
	char* param3c = &param3[0];


	// Line 1
	hw.display.SetCursor(column1x, firstLineY);
	hw.display.WriteString(param1c, Font_7x10, menuIndex == 20);

	// Line 2
	hw.display.SetCursor(column1x, secondLineY);
	hw.display.WriteString(param2c, Font_7x10, menuIndex == 21);

	// Line 3
	hw.display.SetCursor(column1x, thirdLineY);
	hw.display.WriteString(param3c, Font_7x10, true );

	hw.display.Update();

}
void DrawCursor8(int location, bool invert){
    // location is index of menu item
    // there are 4 rows per page
    // 2 menu items per row
    // 0-3 are column 1
    // 4-7 are column 2
    
    int row = location % 3; // calculate row based on location
    int col = location / 3; // calculate column based on location
    
    int x1 = col * (screenWidth / 2); // calculate x coordinate of left edge of rectangle
    int x2 = x1 + (screenWidth / 2) - 1; // calculate x coordinate of right edge of rectangle
    int y1 = row * lineHeight + 1; // calculate y coordinate of top edge of rectangle
    int y2 = y1 + lineHeight - 1; // calculate y coordinate of bottom edge of rectangle
    
    hw.display.DrawRect(x1, y1, x2, y2, invert); // draw rectangle
}

void DrawCursor4(int location, bool invert){
    // location is index of menu item
    // there are 4 rows per page
    // 1 menu items per row
	
	int row = location; // calculate row based on location

	int x1 = 0; // calculate x coordinate of left edge of rectangle
	int x2 = screenWidth; // calculate x coordinate of right edge of rectangle
	int y1 = row * lineHeight + 1; // calculate y coordinate of top edge of rectangle
	int y2 = y1 + lineHeight - 1; // calculate y coordinate of bottom edge of rectangle

    hw.display.DrawRect(x1, y1, x2, y2, invert); // draw rectangle
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

void writeRoutingToDisplay(int routing){
	switch (routing)
	{
	case 0:
		hw.display.WriteString("I", Font_7x10, true);
		break;
	case 1:
		hw.display.WriteString("C", Font_7x10, true);
		break;
	case 2:
		hw.display.WriteString("S", Font_7x10, true);
		break;
	}
}

void writeCrossToDisplay(int cross){
	switch (cross)
	{
	case 0:
		hw.display.WriteString("M", Font_7x10, true);
		break;
	case 1:
		hw.display.WriteString("C", Font_7x10, true);
		break;
	}
}

// Returns an array pointer to a string with the name of the selected parameter
std::string paramEnumToString(ParamIds e) {
    switch (e) {
        case VpO:
            return "VpO";
        case Multiply:
            return "Multiply";
        case oscFreqCV1:
            return "oscFCV1";
        case oscFreqCV2:
            return "oscFCV2";
        case momentumCV1:
            return "momCV1";
        case momentumCV2:
            return "momCV2";
        case oscFreqKnob1:
            return "oscFK1";
        case oscFreqKnob2:
            return "oscFK2";
        case momentumKnob1:
            return "momK1";
        case momentumKnob2:
            return "momK2";
        default:
            return "";
    }
}

#endif