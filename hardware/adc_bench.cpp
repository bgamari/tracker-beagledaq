#include <stdint.h>
#include <sched.h>
#include <time.h>
#include "max1302.h"

using std::vector;

const unsigned int rep = 10000;
float time_commands(max1302& dev, vector<max1302::command*>& cmds)
{
	struct timespec start, stop;
	clock_gettime(CLOCK_MONOTONIC, &start);
	for (unsigned int i=0; i<rep; i++)
		dev.submit(cmds);
	clock_gettime(CLOCK_MONOTONIC, &stop);
	return (stop.tv_nsec - start.tv_nsec)*1e-9 + (stop.tv_sec - start.tv_sec);
}

int main(int argc, char** argv)
{
	max1302 dev("/dev/spidev3.1");
	std::vector<max1302::command*> cmds;

	sched_param sp;
	sp.sched_priority = 10;
	if (sched_setscheduler(0, SCHED_RR, &sp))
		fprintf(stderr, "Warning: Failed to acquire SCHED_RR\n");

	for (unsigned int i=0; i<4; i++) {
		uint16_t scratch;
		cmds.push_back( new max1302::start_conversion_cmd(i, scratch) );
		float t = time_commands(dev, cmds);
		float rate = 1.0*rep*(i+1)/t; // samp/sec
		printf("%d samples in %f seconds (%f samp/sec = %f usec/samp)\n", rep*(i+1), t, rate, 1e6/rate);
	}
	
	return 0;
}

