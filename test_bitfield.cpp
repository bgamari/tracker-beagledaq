/*
 * test_bitfield.cpp
 *
 * (c) 2009 Ben Gamari
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

