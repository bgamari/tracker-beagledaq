/*
 * bitfield.h
 *
 * (c) Copyright 2009 Ben Gamari
 * Author: Ben Gamari <bgamari.foss@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 *
 * This library arose out of a need for a mechanism for flexibly manipulating
 * bit-fields. The implementation may not be the fastest, but it is written to
 * be easily comprehensible and checked. To this end, algorithms requiring any
 * consideration of endianess were not used.
 *
 */

#pragma once
#include <algorithm>
#include <boost/format.hpp>

/*
 * set_bits(): Sets a range of bits
 */
template<typename TWord>
void set_bits(TWord* data, unsigned int start_bit, unsigned int length=1)
{
        int word = start_bit / (8*sizeof(TWord));
	int bit = start_bit % (8*sizeof(TWord));
	for (int i=0; i<length; i++) {
		data[word] |= (1 << bit);
		word += (bit == 8*sizeof(TWord)-1);
		bit = (bit == 8*sizeof(TWord)-1) ? 0 : bit+1;
	}
}

/* 
 * clear(): Clears a range of bits
 */
template<typename TWord>
void clear_bits(TWord* data, unsigned int start_bit, unsigned int length=1)
{
        int word = start_bit / (8*sizeof(TWord));
	int bit = start_bit % (8*sizeof(TWord));
	for (int i=0; i<length; i++) {
		data[word] &= ~(1 << bit);
		word += (bit == 8*sizeof(TWord)-1);
		bit = (bit == 8*sizeof(TWord)-1) ? 0 : bit+1;
	}
}

/*
 * assign_bits(): Sets a range of bits of the given value.
 *
 * @src:		The buffer to copy from
 * @dest:		The buffer to copy to
 * @dest_start_bit:	The bit number of the beginning of the assignment range
 * @dest_end_bit:	The bit number of the end of the assignment range
 *
 */
template<typename TSrc, typename TDest>
void assign_bits(const TSrc* src, TDest* dest, unsigned int dest_start_bit, unsigned int dest_end_bit)
{
	int src_word = 0;
	int src_bit = 0;
	int dest_word = dest_start_bit / (8*sizeof(TDest));
	int dest_bit = dest_start_bit % (8*sizeof(TDest));

	int bits = dest_end_bit - dest_start_bit;
	for (int i=0; i<bits; i++) {
		TSrc bit = src[src_word] & (1 << src_bit);
		dest[dest_word] &= ~(1 << dest_bit);
		dest[dest_word] |= ((bit != 0) << dest_bit);

		src_word += (src_bit == 8*sizeof(TSrc)-1);
		src_bit = (src_bit == 8*sizeof(TSrc)-1) ? 0 : src_bit+1;

		dest_word += (dest_bit == 8*sizeof(TDest)-1);
		dest_bit = (dest_bit == 8*sizeof(TDest)-1) ? 0 : dest_bit+1;
	}
}

/*
 * extract_bits(): Extracts a given range of bits
 *
 * @src:		The buffer to copy from
 * @src_start_bit:	The bit number of the beginning of the extraction range
 * @src_end_bit:	The bit number of the end of the extraction range
 * @dest:		The buffer to copy to
 *
 */
template<typename TSrc, typename TDest>
void extract_bits(const TSrc* src, unsigned int src_start_bit, unsigned int src_end_bit, TDest* dest)
{
	int src_word = src_start_bit / (8*sizeof(TSrc));
	int src_bit = src_start_bit % (8*sizeof(TSrc));
	int dest_word = 0;
	int dest_bit = 0;

	int bits = src_end_bit - src_start_bit;
	for (int i=0; i<bits; i++) {
		TSrc bit = src[src_word] & (1 << src_bit);
		dest[dest_word] &= ~(1 << dest_bit);
		dest[dest_word] |= ((bit != 0) << dest_bit);

		src_word += (src_bit == 8*sizeof(TSrc)-1);
		src_bit = (src_bit == 8*sizeof(TSrc)-1) ? 0 : src_bit+1;

		dest_word += (dest_bit == 8*sizeof(TDest)-1);
		dest_bit = (dest_bit == 8*sizeof(TDest)-1) ? 0 : dest_bit+1;
	}
}

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
		for (int i=0; i < res.words(); i++) {
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

