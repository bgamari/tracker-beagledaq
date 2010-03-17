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

static const char* fb_adc_dev = "/dev/spidev3.0";
static const char* psd_adc_dev = "/dev/spidev3.1";
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
		psd_chans(psd_chans), fb_chans(feedback_chans) { }

	input_data get() {
		std::vector<max1302::command*> psd_cmds, fb_cmds;
		std::vector<uint16_t> psd(4,0), fb(3,0); // Need to convert to float

		for (int i=0; i<4; i++)
			psd_cmds.push_back( new max1302::start_conversion_cmd(psd_chans[i], psd[i]) );
		for (int i=0; i<3; i++)
			fb_cmds.push_back( new max1302::start_conversion_cmd(fb_chans[i], fb[i]) );
		
		psd_adc.submit(psd_cmds);
		fb_adc.submit(fb_cmds);

		input_data v;
		for (int i=0; i<2; i++)
			v.psd_pos[i] = 1.0*psd[i] / 0xffff;
		v.psd_sum = (1.0*psd[2]/0xffff) + (1.0*psd[3]/0xffff);

		for (int i=0; i<3; i++)
			v.fb_pos[i] = 1.0*fb[i] / 0xffff;

		for (auto i=psd_cmds.begin(); i!=psd_cmds.end(); i++)
			delete *i;
		for (auto i=fb_cmds.begin(); i!=fb_cmds.end(); i++)
			delete *i;
		return v;
	}
};

struct test_outputs : output_channels {
	void set(output_data d) {
		printf("stage_pos: ");
		for (int i=0; i<3; i++)
			printf(" %f", d.stage_pos[i]);
	}
};

struct max5134_outputs : output_channels {
	max5134& dac;
	array<max5134::chan_mask,3> stage_chans;

	max5134_outputs(max5134& stage_dac, array<max5134::chan_mask,3> stage_chans) :
		dac(stage_dac), stage_chans(stage_chans) { }

	void set(output_data d) {
		max5134::chan_mask all_mask;
		std::vector<max5134::command*> cmds;
		for (int i=0; i<3; i++) {
			cmds.push_back( new max5134::write_cmd(stage_chans[i], d.stage_pos[i]*0xffff) );
			all_mask |= stage_chans[i];
		}
		cmds.push_back( new max5134::load_dac_cmd(all_mask) );

		dac.submit(cmds);

		for (auto i=cmds.begin(); i!=cmds.end(); i++)
			delete *i;
	}
};

int main(int argc, char** argv)
{
	using std::tr1::array;
	array<int,4> psd_chans = {{0,1,2,3}};
	array<int,3> feedback_chans = {{0,1,2}};
	array<max5134::chan_mask,3> stage_chans = {{ 0x1, 0x2, 0x4 }};

	max1302 psd_adc(psd_adc_dev);
	max1302 fb_adc(fb_adc_dev);
	max1302_inputs inputs(psd_adc, fb_adc, psd_chans, feedback_chans);

	max5134 dac(stage_pos_dac_dev);
	max5134_outputs stage_outputs(dac, stage_chans);

	track(inputs, stage_outputs);
}

