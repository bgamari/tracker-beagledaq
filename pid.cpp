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


#include "pid.h"

float pid_loop::get_response() {
        // Proportional
        float P = points.front().y;

        // Integral
        float I = 0;
        for (unsigned int i=1; i < points.size(); i++)
                I += points[i].y * (points[i].x - points[i-1].x);
        if (points.size() > 2) 
                I /= points.front().x - points.back().x;

        // Derivative
        float D = 0;
        if (points.size() > 2)
               D = (points[0].y - points[1].y) / (points[0].x - points[1].x);

        return prop_gain*P + int_gain*I + diff_gain*D;
}

void pid_loop::add_point(float x, float y) {
        points.add(point(x, y));
}

