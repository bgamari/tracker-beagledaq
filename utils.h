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

#include "stage.h"

#include <vector>
#include <fstream>

using std::vector;
using std::string;

void dump_data(std::string file, vector<collect_cb<3>::point> fb_data,
                vector<collect_cb<4>::point> psd_data, string comment="");

template<typename Matrix>
void dump_matrix(Matrix A, const char* filename)
{
	Eigen::IOFormat fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "\n");
        std::ofstream os(filename);
        os << A.format(fmt);
        os.close();
}
