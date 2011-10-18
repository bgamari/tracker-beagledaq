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

#include <sstream>
#include <string>
#include <vector>

using std::string;

struct parameter {
        string name;
        string description;
        parameter(string name, string description) :
                name(name), description(description) { }
        virtual void operator=(string s) = 0;
        virtual void put(std::ostream& os) const = 0;
};

std::ostream& operator<<(std::ostream& os, const parameter& p);

parameter* find_parameter(std::vector<parameter*> params, string name);

template <typename T>
struct typed_value : parameter {
        typedef T Type;
        T& value;
        typed_value(string name, string description, T& value) :
                parameter(name, description), value(value) { }
        void operator=(string s) {
                std::istringstream(s) >> value;
        }
        void put(std::ostream& os) const {
                os << value;
        }
};

