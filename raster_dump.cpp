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

#include "max5134.h"
#include "max1302.h"
#include "stage.h"
#include "config.h"

#include <exception>
#include <vector>
#include <iostream>
#include <boost/program_options.hpp>

template<typename T>
Matrix<T,3,1> create_vector(std::vector<T> v) {
	Matrix<T,3,1> vec;
	if (v.size() != 3)
		throw std::runtime_error("Vector arguments must have three elements\n");
	vec << v[0], v[1], v[2];
	return vec;
}

int main(int argc, char** argv)
{
	unsigned int nsamps = 100;
	unsigned int delay = 0;

	namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("npts,N", po::value<std::vector<unsigned int> >(), "Number of points in scan (comma-separated)")
		("nsamps,n", po::value<unsigned int>(&nsamps), "Number of samples per point")
		("size,s", po::value<std::vector<float> >(), "Scan size")
		("delay,d", po::value<unsigned int>(&delay), "Delay between samples in usec");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	Vector3f center;
	center << 0.5, 0.5, 0.5;
	Vector3i npts = create_vector(vm["npts"].as<std::vector<int> >());
	Vector3f size = create_vector(vm["size"].as<std::vector<float> >());

	max1302 psd_adc(psd_adc_dev);
	max1302 fb_adc(fb_adc_dev);
	max1302_inputs<4> psd_inputs(psd_adc, psd_chans, max1302::SE_MINUS_VREF_PLUS_VREF);
	max1302_inputs<3> fb_inputs(fb_adc, fb_chans, max1302::SE_ZERO_PLUS_VREF);
        max1302_inputs<1> pd_input(fb_adc, pd_chans, max1302::SE_ZERO_PLUS_VREF);

	max5134 dac(stage_pos_dac_dev);
	max5134_outputs<3> stage_outputs(dac, stage_chans);

	stage stage(stage_outputs, fb_inputs);
	stage.calibrate();
	stage.move(center);

	Vector3f start = center - size / 2;
	Vector3f step = size.array() / npts.array().cast<float>();
	raster_route route(start, step, npts);

	multi_collect_cb<4> psd_collect(psd_inputs, nsamps, delay);

	execute_route(stage, route, {&psd_collect});
}

