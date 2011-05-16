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

#include "channels.h"
#include "stage.h"
#include "config.h"

#include <exception>
#include <vector>
#include <iostream>
#include <boost/program_options.hpp>

int main(int argc, char** argv)
{
	unsigned int nsamps, delay;
	Vector3u npts;
	Vector3f size, center;

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
		("c-x", po::value<float>(&center.x())->default_value(0.5), "Center of scan in X axis")
		("c-y", po::value<float>(&center.y())->default_value(0.5), "Center of scan in Y axis")
		("c-z", po::value<float>(&center.z())->default_value(0.5), "Center of scan in Z axis")
		("nsamps,n", po::value<unsigned int>(&nsamps)->default_value(1), "Number of samples per point")
		("delay,d", po::value<unsigned int>(&delay)->default_value(0), "Delay between samples in usec");

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

	fb_stage stage(stage_outputs, fb_inputs);
	stage.calibrate();
	stage.move(center);

	Vector3f start = center - size / 2;
	Vector3f step = size.array() / npts.array().cast<float>();
	raster_route rt(start, step, npts);
	for (int i=0; rt.has_more(); ++i, ++rt) {
		Vector3f pos = rt.get_pos();
		stage.move(pos);
		usleep(1000);

		Eigen::IOFormat fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, " ", " ");
                for (unsigned int i=0; i < nsamps; i++) {
                        Matrix<float,N,1> v = psd_inputs.get();
			std::cout << pos.format(fmt) << "\t" << v.format(fmt) << "\n";
                        usleep(delay);
                }
	}
}

