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


/*
 *
 * Unit tests for bitfield manipulation library.
 *
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <endian.h>
#include "bitfield.h"

#define FAIL(test) fprintf(stderr, "Failed "test" test\n");

int main() {
	bitfield a(32, 0x12345678);
	bitfield b(32, 0xaaaabbbb);
	
	std::cout << (a >> 14).str();
	if ((a >> 14) != bitfield(32, 0x000048d1)) FAIL("bitfield >>");
	if ((a << 14) != bitfield(32, 0x159e0000)) FAIL("bitfield <<");
	
	if ((a & b) != bitfield(32, 0x02201238)) FAIL("bitfield AND");
	if ((a | b) != bitfield(32, 0xbabefffb)) FAIL("bitfield OR");
	
	if (a.resized(40) != bitfield(40, 0x0012345678ULL)) FAIL("bitfield resize");
	if (a.concat(b) != bitfield(64, 0xaaaabbbb12345678ULL)) FAIL("bitfield concat");
}

