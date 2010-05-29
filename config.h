// Position sensitive device
const char* psd_adc_dev = "/dev/spidev3.0";
std::array<int,4> psd_chans = {{0,1,2,3}};

// Feedback and photodiode
const char* fb_adc_dev = "/dev/spidev3.1";
const std::array<int,3> fb_chans = {{0,1,2}};
const std::array<int,1> pd_chans = {{3}};

// Stage output
static const char* stage_pos_dac_dev = "/dev/spidev4.0";
const std::array<max5134::chan_mask,3> stage_chans = {{ 0x1, 0x2, 0x4 }};

