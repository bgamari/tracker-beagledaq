#pragma once

#include <cstdint>
#include <tr1/array>
#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

struct input_data {
	Vector2f psd_pos;
	float psd_sum;
	Vector3f fb_pos;
};

struct input_channels {
	virtual input_data get() = 0;
};

struct output_channels {
	virtual void set(Vector3f stage) = 0;
};

void track(input_channels& inputs, output_channels& outputs);

