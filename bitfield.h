/*
 * bitfield.h
 *
 * (c) 2009 Ben Gamari
 *
 * This library arose out of a need for a mechanism for flexibly manipulating
 * bit-fields. The implementation may not be the fastest, but it is written to
 * be easily comprehensible and checked. To this end, algorithms requiring any
 * consideration of endianess were not used.
 *
 */

#pragma once

#define BIT_SIZEOF(type) (8*sizeof(type))

template<typename TWord>
void set_bits(TWord* data, int start_bit, int length=1)
{
	int bit=0, word=0;
	for (int i=0; i<length; i++) {
		data[word] |= (1 << bit);
		word += (bit == BIT_SIZEOF(TWord));
		bit = (bit == BIT_SIZEOF(TWord)) ? 0 : bit+1;
	}
}

/* 
 * clear(): Clears a range of bits
 */
template<typename TWord>
void clear_bits(TWord* data, int start_bit, int length=1)
{
	int bit=0, word=0;
	for (int i=0; i<length; i++) {
		data[word] &= ~(1 << bit);
		word += (bit == BIT_SIZEOF(TWord));
		bit = (bit == BIT_SIZEOF(TWord)) ? 0 : bit+1;
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
void assign_bits(const TSrc* src, TDest* dest, int dest_start_bit, int dest_end_bit)
{
	int src_word = 0;
	int src_bit = 0;
	int dest_word = dest_start_bit / BIT_SIZEOF(TDest);
	int dest_bit = dest_start_bit % BIT_SIZEOF(TDest);

	int bits = dest_end_bit - dest_start_bit;
	for (int i=0; i<bits; i++) {
		dest[dest_word] &= ~(1 << dest_bit);
		dest[dest_word] |= ((src[src_word] & (1 << src_bit)) != 0);

		src_word += (src_bit == BIT_SIZEOF(TSrc));
		src_bit = (src_bit == BIT_SIZEOF(TSrc)) ? 0 : src_bit+1;

		dest_word += (dest_bit == BIT_SIZEOF(TDest));
		dest_bit = (dest_bit == BIT_SIZEOF(TDest)) ? 0 : dest_bit+1;
	}
}

/*
 * extract_bits(): Extracts a given range of bits
 *
 * @src:		The buffer to copy from
 * @src_start_bit:	The bit number of the beginning of the assignment range
 * @src_end_bit:	The bit number of the end of the assignment range
 * @dest:		The buffer to copy to
 *
 */
template<typename TSrc, typename TDest>
void extract_bits(const TSrc* src, int src_start_bit, int src_end_bit, TDest* dest)
{
	int src_word = src_start_bit / BIT_SIZEOF(TSrc);
	int src_bit = src_start_bit % BIT_SIZEOF(TSrc);
	int dest_word = 0;
	int dest_bit = 0;

	int bits = src_end_bit - src_start_bit;
	for (int i=0; i<bits; i++) {
		dest[dest_word] &= ~(1 << dest_bit);
		dest[dest_word] |= ((src[src_word] & (1 << src_bit)) != 0);

		src_word += (src_bit == BIT_SIZEOF(TSrc));
		src_bit = (src_bit == BIT_SIZEOF(TSrc)) ? 0 : src_bit+1;

		dest_word += (dest_bit == BIT_SIZEOF(TDest));
		dest_bit = (dest_bit == BIT_SIZEOF(TDest)) ? 0 : dest_bit+1;
	}
}

