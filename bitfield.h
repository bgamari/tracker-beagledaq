
template<typename TWord>
void set_bits(TWord* data, int start_bit, int length=1)
{
	int bit=0, word=0;
	for (int i=0; i<length; i++) {
		data[word] |= (1 << bit);
		word += (bit == sizeof(TWord));
		bit = (bit == sizeof(TWord)) ? 0 : bit+1;
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
		word += (bit == sizeof(TWord));
		bit = (bit == sizeof(TWord)) ? 0 : bit+1;
	}
}

/*
 * assign_bits(): Sets a range of bits of the given value.
 *
 * Endianess is a bitch. After trying to reason through a not-so-iterative way
 * of doing this, I just gave up and punted. Instead we handle each bit
 * individually. Ouch.
 */
template<typename TDest, typename TSrc>
void assign_bits(const TSrc* value, TDest* data, int start_bit, int end_bit)
{
	int bits = end-bit-start_bit
	int src_word = 0, dest_word = 0;
	int src_bit = 0; dest_bit = 0;
	for (int i=0; i<bits; i++) {
		data[dest_word] &= ~(1 << dest_bit);
		data[dest_word] |= ((value[src_word] & (1 << src_bit)) != 0);

		src_word += (src_bit == sizeof(TValue));
		src_bit = (src_bit == sizeof(TValue)) ? 0 : src_bit+1;

		dest_word += (dest_bit == sizeof(TWord));
		dest_bit = (dest_bit == sizeof(TValue)) ? 0 : dest_bit+1;
	}
}


