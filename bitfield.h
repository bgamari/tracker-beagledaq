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
 * This library arose out of a need for a mechanism for flexibly manipulating
 * bit-fields. The implementation may not be the fastest, but it is written to
 * be easily comprehensible and checked. To this end, algorithms requiring any
 * consideration of endianess were not used.
 *
 */

#pragma once
#include <algorithm>
#include <boost/format.hpp>


template <typename word_t>
class generic_bitfield {
	unsigned int bits;
	word_t* data;

public:
	generic_bitfield(unsigned int bits) : bits(bits), data(new word_t[words()]) { }

	template<typename T>
	generic_bitfield(unsigned int bits, T d) : bits(bits), data(new word_t[words()]) {
		*((T*) data) = d;
		unsigned int i = bits/sizeof(word_t) + (bits%sizeof(word_t) != 0);
		for (; i<words(); i++)
			data[i] = 0;
	}

	unsigned int words() const {
		 return bits/sizeof(word_t) + (bits%sizeof(word_t) != 0);
	}

	unsigned int size() const {
		return bits;
	}

	bool get(unsigned int bit) const {
		word_t& d = data[bit/sizeof(word_t)];
		word_t mask = 1 << (bit%sizeof(word_t));
		return (d & mask) != 0;
	}

	void set(unsigned int bit, bool value) {
		word_t& d = data[bit/sizeof(word_t)];
		word_t mask = 1 << (bit%sizeof(word_t));
		if (value)
			d |= mask;
		else
			d &= ~mask;
	}

	generic_bitfield operator<<(unsigned int shift) const {
		generic_bitfield res(bits, 0);
		for (unsigned int i=shift; i<bits; i++)
			res.set(i, get(i-shift));
		return res;
	}

	generic_bitfield operator>>(unsigned int shift) const {
		generic_bitfield res(bits, 0);
		for (unsigned int i=0; i<bits-shift; i++)
			res.set(i, get(i+shift));
		return res;
	}

	generic_bitfield operator|(const generic_bitfield& bf) const {
		generic_bitfield res(bits);
		unsigned int b = std::min(words(), bf.words());
		for (unsigned int i=0; i<b; i++)
			res.data[i] = bf.data[i] | data[i];
		return res;
	}

	generic_bitfield operator&(const generic_bitfield& bf) const {
		generic_bitfield res(bits);
		unsigned int b = std::min(words(), bf.words());
		for (unsigned int i=0; i<b; i++)
			res.data[i] = bf.data[i] & data[i];
		return res;
	}

	generic_bitfield& operator|=(const generic_bitfield& bf) {
		unsigned int b = std::min(words(), bf.words());
		for (unsigned int i=0; i<b; i++)
			data[i] |= bf.data[i];
		return *this;
	}

	generic_bitfield& operator&=(const generic_bitfield& bf) {
		unsigned int b = std::min(words(), bf.words());
		for (unsigned int i=0; i<b; i++)
			data[i] &= bf.data[i];
		return *this;
	}

	bool operator==(const generic_bitfield& b) const {
		if (bits != b.bits)
			return false;
		for (unsigned int i=0; i<words(); i++)
			if (data[i] != b.data[i]) return false;
		return true;
	}

	bool operator!=(const generic_bitfield& b) const {
		return !(*this == b);
	}

	generic_bitfield resized(unsigned int size) const {
		generic_bitfield res(size);
		int mask_size = std::min(bits, size);
		for (unsigned int i=0; i < res.words(); i++) {
			uint8_t mask = 0xff;
			if (mask_size-sizeof(word_t)*i < sizeof(word_t))
				mask = (1 << (mask_size - 8*i)) - 1;
			res.data[i] = data[i] & mask;
		}
		return res;
	}

	generic_bitfield concat(const generic_bitfield& b) const {
		generic_bitfield res = resized(bits + b.bits);
		generic_bitfield c = b.resized(bits + b.bits) << bits;
		res |= c;
		return res;
	}

	class reference {
		friend class generic_bitfield;
		generic_bitfield& bf;
		const unsigned int bit;
	private:
		reference(generic_bitfield& bf, unsigned int bit) : bf(bf), bit(bit) { }
	public:
		reference& operator=(bool value) {
			bf.set(bit, value);
			return *this;
		}
		reference& operator=(const reference& ref) {
			bool value = ref.bf.get(ref.bit);
			bf.set(bit, value);
			return *this;
		}
		bool operator~() const {
			return !(*this);
		}
		operator bool() const {
			return bf.get(bit);
		}
		reference& flip() {
			bool value = *this;
			bf.set(bit, !value);
		}
	};
	friend class reference;

	reference operator[](unsigned int bit) {
		return reference(*this, bit);
	};

	std::string str() {
		std::string s;
		s.reserve(3*words() + 1);
		boost::format fmt("%02x ");
		for (unsigned int i=0; i<words(); i++)
			s.append((fmt % data[words()-i-1]).str());
		s.append("\n");
		return s;
	}
};

typedef generic_bitfield<unsigned int> bitfield;

