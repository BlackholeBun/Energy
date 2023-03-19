#pragma once

struct SchmittTrigger {
	// implements a 0.1V - 1.0V SchmittTrigger (include/dsp/digital.hpp) instead of 
	//   calling SchmittTriggerInstance.process(math::rescale(in, 0.1f, 1.f, 0.f, 1.f))
	bool state = false;
	bool process(float in) {
		if (state) {
			// HIGH to LOW
			if (in <= 0.1f) {
				state = false;
			}
		}
		else {
			// LOW to HIGH
			if (in >= 1.0f) {
				state = true;
				return true;
			}
		}
		return false;
	}	
};

float process(float, float);
void UpdateOled();
void writeQuantToDisplay(int mode);
void writeModToDisplay(int mode);
void writeRoutingToDisplay(int mode);
void writeFreqToDisplay(int mode);
void writeMomentumToDisplay(int mode);
void writeCrossToDisplay(int mode);
void writeEnergyToDisplay(int mode);
void writeFreqToDisplay(int mode);
float calcFreqKnob(int osci);
void calcModSignals(int chan);
void calcFeedbacks(int chan);

inline float clamp(float x, float min, float max) {
	return x < min ? min : (x > max ? max : x);
};
/*
struct Energy : Module {
	enum ParamIds {
		ENUMS(PLANCK_PARAMS, 2),// push buttons
		ENUMS(MODTYPE_PARAMS, 2),// push buttons
		ROUTING_PARAM,// push button
		ENUMS(FREQ_PARAMS, 2),// rotary knobs (middle)
		ENUMS(MOMENTUM_PARAMS, 2),// rotary knobs (top)
		CROSS_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		ENUMS(FREQCV_INPUTS, 2), // respectively "mass" and "speed of light"
		FREQCV_INPUT, // main voct input
		MULTIPLY_INPUT,
		ENUMS(MOMENTUM_INPUTS, 2),
		NUM_INPUTS
	};
	enum OutputIds {
		ENERGY_OUTPUT,// main output
		NUM_OUTPUTS
	};
	enum LightIds {
		ENUMS(PLANCK_LIGHTS, 2 * 3), // room for two Blue Yellow White leds
		ENUMS(ADD_LIGHTS, 2),
		ENUMS(AMP_LIGHTS, 2),
		ENUMS(ROUTING_LIGHTS, 3),
		ENUMS(MOMENTUM_LIGHTS, 2),
		ENUMS(FREQ_ROUTING_LIGHTS, 2 * 2),// room for blue/yellow
		CROSS_LIGHT,
		NUM_LIGHTS
	};
};
*/