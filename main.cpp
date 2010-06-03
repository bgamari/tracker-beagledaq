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
#include <iostream>
#include <array>
#include <boost/tokenizer.hpp>
#include "bitfield.h"
#include "max5134.h"
#include "max1302.h"
#include "tracker.h"
#include "config.h"

using std::array;
using std::string;

int main(int argc, char** argv)
{
//#define TEST
#ifndef TEST
	max1302 psd_adc(psd_adc_dev);
	max1302 fb_adc(fb_adc_dev);
	max1302_inputs<4> psd_inputs(psd_adc, psd_chans, max1302::SE_MINUS_VREF_PLUS_VREF);
	max1302_inputs<3> fb_inputs(fb_adc, fb_chans, max1302::SE_ZERO_PLUS_VREF);
        max1302_inputs<1> pd_input(fb_adc, pd_chans, max1302::SE_ZERO_PLUS_VREF);

	max5134 dac(stage_pos_dac_dev);
	max5134_outputs<3> stage_outputs(dac, stage_chans);
#else	
	test_inputs<4> psd_inputs;
	test_inputs<3> fb_inputs;
	test_outputs<3> stage_outputs;
#endif

	stage stage(stage_outputs, fb_inputs);
	stage.calibrate();
	stage.move({0.5, 0.5, 0.5});
        tracker tracker(psd_inputs, stage, fb_inputs);
	usleep(10*1000);
	Vector3f fb = fb_inputs.get();
	fprintf(stderr, "Feedback position: %f %f %f\n", fb.x(), fb.y(), fb.z());

	while (true) {
		std::cout << "> ";
		string line;
	        std::getline(std::cin, line);
		boost::tokenizer<> tokens(line);
		boost::tokenizer<>::iterator tok = tokens.begin();

		string cmd = *tok; tok++;
		if (cmd == "set") {
			string param = *tok; tok++;
			string value = *tok;
			parameter* p = find_parameter(tracker.parameters, param);
			if (!p)
				abort();

			*p = value;
		} else if (cmd == "get") {
			string param = *tok;
			parameter* p = find_parameter(tracker.parameters, param);
			if (!p)
				abort();
			std::cout << param << " = " << *p << "\n";
		} else if (cmd == "list") {
			for (auto p=tracker.parameters.begin(); p != tracker.parameters.end(); p++)
				std::cout << (**p).name << " = " << **p << "\n";
		} else
			std::cout << "Invalid command\n";
	}

#define TRACK
#ifdef TRACK
	fprintf(stderr, "Position bead. Press any key.\n");
	getchar();
	Vector4f psd = psd_inputs.get();
	fprintf(stderr, "PSD: pos=(%f, %f); sum=(%f, %f)\n",
			psd[0], psd[1], psd[2], psd[3]);
	track(psd_inputs, stage, fb_inputs);
#else
	printf("# psd_x psd_y sum_x sum_y\tfb_x fb_y fb_z\tpd\n");
	unsigned int n=0;
	while (true) {
		Vector4f psd = psd_inputs.get();
#define SCALE_INPUTS 1
#if SCALE_INPUTS
                psd.x() /= psd[2];
                psd.y() /= psd[3];
#endif
		Vector3f fb = fb_inputs.get();
                Matrix<float,1,1> pd = pd_input.get();
		for (int i=0; i<4; i++) printf("%f ", psd[i]);
		printf("\t");
		for (int i=0; i<3; i++) printf("%f ", fb[i]);
		printf("\t%f\n", pd[0]);

		usleep(1000*10);
		n++;
                Vector3f pos;
                pos << 0.5 + 0.2*sin(0.01*n), 0.5, 0.5;
                //stage.move(pos);
	}
#endif
	tracker.track();
	return 0;
}

