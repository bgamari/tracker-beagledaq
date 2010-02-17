/* tracker - Back-focal plane droplet tracker
 *
 * Copyright © 2010 Ben Gamari
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * Author: Ben Gamari <bgamari@physics.umass.edu>
 */


#include <cstdint>
#include "bitfield.h"
#include "max5590.h"
#include "max1270.h"
#include "tracker.h"

static const char* in_dev = "/dev/spidev1.0";
static const char* out_dev = "/dev/spidev1.1";

static uint16_t delay = 0;
static uint8_t mode = 0;
static uint8_t bits = 16;
static uint32_t speed = 500000;

using std::tr1::array;

struct max1270_inputs : input_channels {
	max1270& adc;
	array<int,4> psd_chans;
	array<int,3> feedback_chans;

	max1270_inputs(max1270& adc, array<int,4> psd_chans, array<int,3> feedback_chans) :
		adc(adc), psd_chans(psd_chans), feedback_chans(feedback_chans) { }

	input_data get() {
		input_data v;
		std::vector<max1270::command*> cmds;

		for (int i=0; i<4; i++)
			cmds.push_back( new max1270::take_sample(psd_chans[i], v.psd[i]) );
		for (int i=0; i<3; i++)
			cmds.push_back( new max1270::take_sample(feedback_chans[i], v.feedback[i]) );
		
		adc.submit(cmds);

		for (auto i=cmds.begin(); i!=cmds.end(); i++)
			delete *i;
		return v;
	}
};

struct max5590_outputs : output_channels {
	max5590& dac;
	array<max5590::input_reg,3> stage_chans;

	max5590_outputs(max5590& dac, array<max5590::input_reg,3> stage_chans) :
		dac(dac), stage_chans(stage_chans) { }

	void set(output_data d) {
		std::vector<max5590::command*> cmds;
		for (int i=0; i<3; i++)
			cmds.push_back( new max5590::load_input_cmd(stage_chans[i], d.stage[i]) );

		dac.submit(cmds);

		for (auto i=cmds.begin(); i!=cmds.end(); i++)
			delete *i;
	}
};

int main(int argc, char** argv)
{
	using std::tr1::array;
	array<int,4> psd_chans = {{0,1,2,3}};
	array<int,3> feedback_chans = {{4,5,6}};
	array<max5590::input_reg,3> stage_chans = {{
		max5590::input_reg::A,
		max5590::input_reg::B,
		max5590::input_reg::C }};

	max1270 adc("/dev/spidev0.0");
	max1270_inputs inputs(adc, psd_chans, feedback_chans);

	max5590 dac("/dev/spidev0.1");
	max5590_outputs outputs(dac, stage_chans);

	track(inputs, outputs);
}

