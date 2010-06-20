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

#include "utils.h"

#include <boost/format.hpp>

void dump_data(std::string file, vector<collect_cb<3>::point> fb_data,
                vector<collect_cb<4>::point> psd_data, string comment) {
        std::ofstream f(file);
        if (comment.length())
                f << "# " << comment << "\n";
	f << "# pos_x pos_y pos_z\tfb_x fb_y fb_z\tpsd_x psd_y sum_x sum_y\n";
        for (unsigned int i=0; i < psd_data.size(); i++)
		f << boost::format("%f %f %f\t%f %f %f\t%f %f %f %f\n") %
				fb_data[i].position[0] % fb_data[i].position[1] % fb_data[i].position[2] %
                                fb_data[i].values[0] % fb_data[i].values[1] % fb_data[i].values[2] %
				psd_data[i].values[0] % psd_data[i].values[1] % psd_data[i].values[2] % psd_data[i].values[3];
}

