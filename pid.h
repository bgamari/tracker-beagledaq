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

#pragma once

#include "ringbuffer.h"

class pid_loop {
        struct point {
                float x, y;
                point(float x, float y) : x(x), y(y) { }
        };
        ring_buffer<point> points;
public:
        float prop_gain, int_gain, diff_gain;

public:
        pid_loop(float prop_gain=1, float int_gain=0, float diff_gain=0, unsigned int tau=10) :
                points(tau), prop_gain(prop_gain), int_gain(int_gain), diff_gain(diff_gain)
        {
                assert(tau >= 1);
        }

        void set_tau(unsigned int tau) {
                points.resize(tau);
        }

        unsigned int tau() {
                return points.capacity();
        }

        void clear();
        void add_point(float x, float y);
        float get_response();
};

