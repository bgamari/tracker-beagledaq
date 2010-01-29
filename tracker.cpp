#include <utility>
#include <time.h>
#include "tracker.h"

std::array<uint16_t,3> cal_rough_start = {0,0,0};
std::array<uint16_t,3> cal_rough_step = {100,100,100};
std::array<int,3> cal_rough_pts = {1000,1000,1000};

template<int N>
struct point_callback {
	/*
	 * operator()
	 *
	 * Returns: false to abort route. true otherwise.
	 *
	 */
	virtual bool operator()(int n, std::array<uint16_t,N>& outputs) = 0;
};

struct route {
	virtual std::array<uint16_t,3> get_point() = 0;
	virtual void operator++() = 0;
	virtual bool is_end() = 0;
};

struct random_route : route {
	int n_pts;
	const std::array<std::uniform_int,3> rngs;
	std::array<uint16_t,3> a;

	random_route(int n_pts, std::array<std::uniform_int,3> rngs) :
		rngs(rngs), n_pts(n_pts) { }

	void operator++()  {
		n_pts--;
		for (int i=0; i<N; i++)
			a[i] = rngs[i]();
	}

	std::array<uint16_t,N> get_point() { return a; }

	bool is_end() {
		return !(n_pts > 0);
	}
};

struct raster_route : route {
	const std::array<uint16_t,3> start;
	const std::array<uint16_t,3> step;
	const std::array<unsigned int,3> points;

	std::array<int,3> dirs;
	std::array<int,3> pos; // current position in steps

	raster_route(std::array<uint16_t,3> start, std::array<uint16_t,3> step, std::array<int,3> points) :
		n(0), start(start), step(step), points(points), pos({0,0,0}) {  }

	std::array<uint16_t,3> get_point() {
		std::array<uint16_t,N> a;
		for (int i=0; i < 3; i++)
			a[i] = step[i]*pos[i];
		return a;
	}

	void operator++() {
		unsigned int i=0;
		while (true) {
			pos[i] += dirs[i];

			if (pos[i] <= 0 || pos[i] >= points[i]) {
				dirs[i] *= -1;
			else
				break;
		}
	}

	bool is_end() {
		for (int i=0; i<3; i++) {
			if (pos[i] <= points[i])
				return false;
		}
		return true;
	}
};

template <int n_in, int n_out>
struct find_min_max : point_callback<n_out> {
	input_channels<n_in>& inputs;
	input_channels<2>& feedback;

	std::array<int, n_in> min_val, max_val;
	std::array<std::array<uint16_t,n_in>, n_out> min_pos, max_pos;
	std::array<std::array<uint16_t,2>, n_out> min_fb_pos, max_fb_pos;

	find_min_max(input_channels<n_in> signals, input_channels<2> feedback) :
		inputs(inputs), feedback(feedback) { }

	bool operator()(int n, std::array<int,n_out> outputs) {
		std::array<uint16_t,n_in> in = inputs.get();
		std::array<uint16_t,2> fb = feedback.get();
		for (int i=0; i<n_in; i++) {
			if (in[i] > max_val[i]) {
				max_fb_pos[i] = fb;
				max_pos[i] = outputs;
				max_val[i] = in[i];
			}
			if (in[i] < min_val[i]) {
				min_fb_pos[i] = fb;
				min_pos[i] = outputs;
				min_val[i] = in[i];
			}
		}
		usleep(100);
	}
};

static std::array<uint16_t, 3> rough_calibrate(input_channels<4>& in, input_channels<2>& fb_in, output_channels<3>& out)
{
	raster_route<3> route(cal_rough_start, cal_rough_step, cal_rough_pts);
	find_min_max<3> cb(in, fb_in);

	execute_route(out, route, cb);
	std::array<uint16_t, 3> laser_pos = {
		(max_pos[0][0] + min_pos[0][0]) / 2,
		(max_pos[0][1] + min_pos[0][1]) / 2,
		0 };
	
	// Scan in Z direction
	raster_route<1> route();
	find_min_max<3> cb(in, fb_in);
	execute_route(out, route, cb);
	laser_pos[3] = (max_pos[0][0] + min_pos[0][0]) / 2;
	return laser_pos;
}

static matrix<9,3> fine_calibrate(input_channels<4>& in, input_channels<2>& fb_in, output_channels<3>& out)
{
	std::array<uniform_int,3> rngs = {
		uniform_int(-100, +100),
		uniform_int(-100, +100),
		uniform_int(-100, +100)
	};
	random_route route(rngs, n_pts);
	collect_data_cb cb(n_pts); // all inputs

	execute_route(out, route, cb);

}

static matrix<9,3> invert_matrix(matrix)
{
	matrix RRi = (R.transpose() * R).inverse();
	matrix beta = (RRi * R.transpose()) * S;
	return beta.transpose()
}

static void execute_route(
		output_channels<n_out> channels,
		route route,
		point_callback* cb=NULL)
{
	int n=0;
	for (; route.has_more(); route++, n++) {
		channels.set(*i);
		if (cb) cb(n, *i);
	}
}

void feedback(input_channels& inputs, output_channels& outputs)
{
	while (1) {
		inputs = get_inputs();
		calulate_delta();
		set_outputs()
	}
}

void track(input_channels& inputs, output_channels& outputs)
{
	std::vector<uint16_t, 3> laser_pos = rough_calibrate();
	feedback(coeffs);
}

