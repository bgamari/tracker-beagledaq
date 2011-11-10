#include <stdio.h>
#include "ringbuffer.h"

int main()
{
	ring_buffer<int> rb(5);
	for (int i=0; i<15; i++) {
		rb.add(i);
		printf("%d\t", rb.size());
		printf("%d %d\t", rb.front(), rb.back());
		for (int j=0; j<rb.size(); j++)
			printf("%d, ", rb[j]);
		printf("\n");
	}
	return 0;
}

