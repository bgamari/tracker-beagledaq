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
#include "max5134.h"
#include "max1302.h"
#include "tracker.h"

static const char* psd_adc_dev = "/dev/spidev3.0";
static const char* fb_adc_dev = "/dev/spidev3.1";
static const char* stage_pos_dac_dev = "/dev/spidev4.0";

using std::tr1::array;

struct test_inputs : input_channels {
	input_data get() {
		input_data v = {};
		return v;
	}
};

struct max1302_inputs : input_channels {
	max1302& psd_adc;
	max1302& fb_adc;
	array<int,4> psd_chans;
	array<int,3> fb_chans;

	max1302_inputs(max1302& psd_adc, max1302& fb_adc, array<int,4> psd_chans, array<int,3> feedback_chans) :
		psd_adc(psd_adc), fb_adc(fb_adc),
		psd_chans(psd_chans), fb_chans(feedback_chans)
	{
		std::vector<max1302::command*> cmds;
		
		for (int i=0; i<4; i++) {
			cmds.push_back(new max1302::input_config_cmd(psd_chans[i], max1302::EXT_CLOCK));
			cmds.push_back(new max1302::mode_cntrl_cmd(psd_chans[i], max1302::SE_ZERO_PLUS_VREF));
		}
		psd_adc.submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;

		cmds.clear();
		for (int i=0; i<3; i++) {
			cmds.push_back(new max1302::input_config_cmd(fb_chans[i], max1302::EXT_CLOCK));
			cmds.push_back(new max1302::mode_cntrl_cmd(fb_chans[i], max1302::SE_ZERO_PLUS_VREF));
		}
		fb_adc.submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}

	input_data get() {
		std::vector<max1302::command*> cmds;
		array<uint16_t,4> psd;
		array<uint16_t,3> fb;
		input_data v;

		for (int i=0; i<4; i++)
			cmds.push_back(new max1302::start_conversion_cmd(psd_chans[i], &psd[i]));
		psd_adc.submit(cmds);
		for (int i=0; i<2; i++) {
			v.psd_pos[i] = 1.0*psd[i] / 0xffff;
			delete cmds[i];
		}
		v.psd_sum = (1.0*psd[2]/0xffff) + (1.0*psd[3]/0xffff);
		cmds.clear();

		for (int i=0; i<3; i++)
			cmds.push_back(new max1302::start_conversion_cmd(fb_chans[i], &fb[i]));
		fb_adc.submit(cmds);
		for (int i=0; i<3; i++) {
			v.fb_pos[i] = 1.0*fb[i] / 0xffff;
			delete cmds[i];
		}

		return v;
	}
};

struct test_outputs : stage_outputs {
	void move(Vector3f position) {
		printf("stage_pos: ");
		for (int i=0; i<3; i++)
			printf(" %f", position[i]);
		printf("\n");
	}
};

struct max5134_outputs : stage_outputs {
	max5134& dac;
	array<max5134::chan_mask,3> channels;

	max5134_outputs(max5134& stage_dac, array<max5134::chan_mask,3> channels) :
		dac(stage_dac), channels(channels) { }

	void move(Vector3f position) {
		max5134::chan_mask all_mask;
		std::vector<max5134::command*> cmds;
		for (int i=0; i<3; i++) {
			if (position[i] < 0.0 || position[i] > 1.0)
				fprintf(stderr, "Warning: Clamped output\n");
			cmds.push_back(new max5134::write_cmd(channels[i], position[i]*0xffff));
			all_mask |= channels[i];
		}
		cmds.push_back(new max5134::load_dac_cmd (all_mask));

		dac.submit(cmds);
		this->position = position;
	}
};

int main(int argc, char** argv)
{
	using std::tr1::array;
	array<int,4> psd_chans = {{0,1,2,3}};
	array<int,3> feedback_chans = {{0,1,2}};
	array<max5134::chan_mask,3> stage_chans = {{ 0x1, 0x2, 0x4 }};

//#define TEST
#ifndef TEST
	max1302 psd_adc(psd_adc_dev);
	max1302 fb_adc(fb_adc_dev);
	max1302_inputs inputs(psd_adc, fb_adc, psd_chans, feedback_chans);

	max5134 dac(stage_pos_dac_dev);
	max5134_outputs stage_outputs(dac, stage_chans);
#else	
	test_inputs inputs;
	test_outputs stage_outputs;
#endif

//#define TRACK
#ifdef TRACK
	stage_outputs.move({0.5, 0.5, 0.5});
	fprintf(stderr, "Position bead. Press any key.\n");
	getchar();
	track(inputs, stage_outputs);
#else
	printf("# psd_x psd_y\tpsd_sum\tfb_x fb_y fb_z\n");
	int n=0;
	while (n < 10000) {
		input_data d = inputs.get();
		printf("%f\n", d.fb_pos[0]);
		for (int i=0; i<2; i++)
			printf("%f ", d.psd_pos[i]);
		printf("\t%f\t", d.psd_sum);
		for (int i=0; i<3; i++)
			printf("%f ", d.fb_pos[i]);
		printf("\n");
		//usleep(100);
		n++;
	}
#endif
	return 0;
}

