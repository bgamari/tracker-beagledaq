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

template<unsigned int N>
struct test_inputs : input_channels<N> {
	Matrix<float,1,N> get() {
		Matrix<float,1,N> v = Matrix<float,1,N>::Zero();
		return v;
	}
};

template<unsigned int N>
struct max1302_inputs : input_channels<N> {
	max1302& dev;
	array<int,N> channels;

	max1302_inputs(max1302& dev, array<int,N> channels) :
		dev(dev), channels(channels)
	{
		std::vector<max1302::command*> cmds;
		
		for (unsigned int i=0; i<N; i++) {
			cmds.push_back(new max1302::input_config_cmd(channels[i], max1302::EXT_CLOCK));
			cmds.push_back(new max1302::mode_cntrl_cmd(channels[i], max1302::SE_ZERO_PLUS_VREF));
		}
		dev.submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}

	Matrix<float,1,N> get() {
		array<uint16_t,N> int_vals;
		Matrix<float,1,N> values;
		std::vector<max1302::command*> cmds;
		cmds.reserve(N);

		for (unsigned int i=0; i<N; i++)
			cmds.push_back(new max1302::start_conversion_cmd(channels[i], &int_vals[i]));
		dev.submit(cmds);
		for (unsigned int i=0; i<N; i++) {
			values[i] = 1.0*int_vals[i] / 0xffff;
			delete cmds[i];
		}
		return values;
	}
};

template<unsigned int N>
struct test_outputs : output_channels<N> {
	void set(Matrix<float,1,N> values) {
		printf("stage_pos: ");
		for (unsigned int i=0; i<N; i++)
			printf(" %f", values[i]);
		printf("\n");
	}
};

template<unsigned int N>
struct max5134_outputs : output_channels<N> {
	max5134& dev;
	array<max5134::chan_mask,N> channels;

	max5134_outputs(max5134& dev, array<max5134::chan_mask,N> channels) :
		dev(dev), channels(channels) { }

	void set(Matrix<float,1,N> values)
	{
		max5134::chan_mask all_mask;
		std::vector<max5134::command*> cmds;
		cmds.reserve(N+1);
		for (unsigned int i=0; i<N; i++) {
			if (values[i] < 0.0 || values[i] > 1.0)
				fprintf(stderr, "Warning: Clamped output\n");
			cmds.push_back(new max5134::write_cmd(channels[i], values[i]*0xffff));
			all_mask |= channels[i];
		}
		cmds.push_back(new max5134::load_dac_cmd (all_mask));

		dev.submit(cmds);
		this->last_values = values;
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}
};

int main(int argc, char** argv)
{
	using std::tr1::array;
	array<int,4> psd_chans = {{0,1,2,3}};
	array<int,3> fb_chans = {{0,1,2}};
	array<max5134::chan_mask,3> stage_chans = {{ 0x1, 0x2, 0x4 }};

//#define TEST
#ifndef TEST
	max1302 psd_adc(psd_adc_dev);
	max1302 fb_adc(fb_adc_dev);
	max1302_inputs<4> psd_inputs(psd_adc, psd_chans);
	max1302_inputs<3> fb_inputs(fb_adc, fb_chans);

	max5134 dac(stage_pos_dac_dev);
	max5134_outputs<3> stage_outputs(dac, stage_chans);
#else	
	test_inputs<4> psd_inputs;
	test_inputs<3> fb_inputs;
	test_outputs<3> stage_outputs;
#endif

//#define TRACK
#ifdef TRACK
	stage_outputs.set({0.5, 0.5, 0.5});
	fprintf(stderr, "Position bead. Press any key.\n");
	getchar();
	track(psd_inputs, stage_outputs, fb_inputs);
#else
	printf("# psd_x psd_y\tpsd_sum\tfb_x fb_y fb_z\n");
	int n=0;
	while (n < 10000) {
		Vector4f d;
		d = psd_inputs.get();
		for (int i=0; i<4; i++)
			printf("%f ", d[i]);
		printf("\n");
		//usleep(100);
		n++;
	}
#endif
	return 0;
}

