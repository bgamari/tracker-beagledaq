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
        assert(points.size() > 0);
        float resp = 0;

        // Proportional
        if (prop_gain != 0)
                resp += prop_gain * points.front().y;

        // Integral
        if (int_gain != 0 && points.size() > 2) {
                float I = 0;
                for (unsigned int i=1; i < points.size(); i++)
                        I += points[i].y * (points[i].x - points[i-1].x);
                I /= points.front().x - points.back().x;
                resp += int_gain * I;
        }

        // Derivative
        if (diff_gain != 0 && points.size() > 2)
                resp += diff_gain * (points[0].y - points[1].y) / (points[0].x - points[1].x);
        return resp;
}

void pid_loop::add_point(float x, float y) {
        points.add(point(x, y));
}

