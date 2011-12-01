#include "config.h"

bool scale_psd_inputs = false;

rough_cal_params rough_params =
	{ xy_range: 0.4 	, z_range: 0.4
	, xy_npts: 40	        , z_npts: 200
	, xy_dwell: 1000	, z_dwell: 1000
	, z_avg_window: 5
	};

fine_cal_params fine_params =
	{ xy_range: 0.008
	, z_range: 0.02
	, npts: 400
	, dwell: 1000
	};

rough_cal_params def_rough_cal_params() {
        return rough_params;
}

fine_cal_params def_fine_cal_params() {
        return fine_params;
}

feedback_params def_feedback_params() {
        feedback_params p;

        p.delay = 2000;
        p.max_delta = 0.05;
        p.show_rate = false;
        p.rate_report_period = 5;

        p.pids[0] = pid_loop(0.6, 1e-3, 0, 10);
        p.pids[1] = pid_loop(0.6, 1e-3, 0, 10);
        p.pids[2] = pid_loop(1.0, 1e-3, 0, 10);

	p.setpoint = Vector3f::Zero();
        return p;
}

