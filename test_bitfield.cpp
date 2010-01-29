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

void test_set_clear_bits() {
	uint8_t b[4] = {0, };
	uint8_t res[4] = { 0x00, 0xe0, 0xf1, 0x00 };
	
	set_bits(b, 13, 11);
	clear_bits(b, 17, 3);
	if (memcmp(b, res, 4)) FAIL("set/clear bits");
	//for (int i=0; i<4; i++) printf("%04x	%02hhx	%02hhx\n", i, b[i], res[i]);
}

void test_assign_bits() {
	uint32_t a = 0xdeadbeef;
	uint8_t b[6] = {0, };
	uint8_t res[6] = { 0x00, 0xf0, 0xee, 0xdb, 0xea, 0x0d };

	assign_bits(&a, b, 12, 12+32);
	if (memcmp(b, res, 6)) FAIL("assign bits");
	//for (int i=0; i<6; i++) printf("%04x	%02hhx\n", i, b[i]);
}

void test_extract_bits() {
	uint8_t a[6] = { 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
	uint32_t b, res = 0xfddbb997;

	extract_bits(a, 11, 11+32, &b);
	if (b != res) FAIL("extract bits");
	//printf("%08x\n", b);
}

void test_bitfield() {
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

int main() {
	test_set_clear_bits();
	test_assign_bits();
	test_extract_bits();

	test_bitfield();
}

