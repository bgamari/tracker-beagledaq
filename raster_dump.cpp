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

template<unsigned int N>
struct multi_dump_cb : point_callback {
	input_channels<N>& inputs;
	struct point {
		Vector3f position;
		vector<Matrix<float,N,1>> values;
	};
        unsigned int n_pts, delay;

	multi_dump_cb(input_channels<N>& inputs,
                        unsigned int n_pts, unsigned int delay=0) :
                inputs(inputs), n_pts(n_pts), delay(delay) { }

        bool operator()(Vector3f& pos) {
		Eigen::IOFormat fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, " ", " ");
                for (unsigned int i=0; i < n_pts; i++) {
                        Matrix<float,N,1> v = inputs.get();
			std::cout << pos.format(fmt) << "\t" << v.format(fmt) << "\n";
                        usleep(delay);
                }
		return true;
	}
};

int main(int argc, char** argv)
{
	unsigned int nsamps = 100;
	unsigned int delay = 0;
	Matrix<unsigned int,3,1> npts;
	Vector3f size, center;
	center << 0.5, 0.5, 0.5;

	namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "produce help message")
		("n-x", po::value<unsigned int>(&npts.x()), "Number of points in X axis")
		("n-y", po::value<unsigned int>(&npts.y()), "Number of points in Y axis")
		("n-z", po::value<unsigned int>(&npts.z()), "Number of points in Z axis")
		("s-x", po::value<float>(&size.x()), "Size of scan in X axis")
		("s-y", po::value<float>(&size.y()), "Size of scan in Y axis")
		("s-z", po::value<float>(&size.z()), "Size of scan in Z axis")
		("size,s", po::value<std::vector<float> >(), "Scan size")
		("nsamps,n", po::value<unsigned int>(&nsamps), "Number of samples per point")
		("delay,d", po::value<unsigned int>(&delay), "Delay between samples in usec");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		std::cout << desc;
		return 0;
	}

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

	multi_dump_cb<4> psd_collect(psd_inputs, nsamps, delay);

	execute_route(stage, route, {&psd_collect});
}

