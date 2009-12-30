#include "max5590.h"

uint16_t max5590::build_command(enum command cmd, short data)
{
	return ((0xf & cmd) << 12) | (0x0fff & data);
}

void max5590::write_outputs(std::vector<std::pair<int, uint16_t> > channels)
{
	int n = channels.size();
	uint16_t tx[n];

	for (int i=0; i<n; i++) {
		if (channels[i].first > 7)
			throw "invalid channel";

		command cmd = (command) channels[i].first;
		tx[i] = build_command(cmd, channels[i].second);
	}
	write(tx, NULL, n);
}

