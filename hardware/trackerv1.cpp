/* tracker - Back-focal plane droplet tracker
 *
 * Copyright © 2010 Ben Gamari
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * Author: Ben Gamari <bgamari@physics.umass.edu>
 */

#include <array>

// ==== Configuration ====
// Position sensitive device
const char* psd_adc_dev = "/dev/spidev3.0";
std::array<int,4> psd_chans = {{0,1,2,3}};

// Feedback and photodiode
const char* stage_adc_dev = "/dev/spidev3.1";
const std::array<int,3> fb_chans = {{0,1,2}};
const std::array<int,1> pd_chans = {{3}};

// Stage output
static const char* stage_dac_dev = "/dev/spidev4.0";
const std::array<max5134::chan_mask,3> stage_chans = {{ 0x1, 0x2, 0x4 }};
// ==== End configuration ====


static max1302* psd_adc;
static max1302* stage_adc;
static max5134* stage_dac;

void tracker_init()
{
	psd_adc = new max1302(psd_adc_dev);
	stage_adc = new max1302(stage_adc_dev);
	stage_dac = new max5134(stage_dac_dev);

	psd_in = new max1302_inputs<4>(*psd_adc, psd_chans, max1302::SE_MINUS_VREF_PLUS_VREF);
	photodiode_in = new max1302_inputs<1>(*psd_adc, pd_chans, max1302::SE_ZERO_PLUS_VREF);
	stage_in = new max1302_inputs<3>(*stage_adc, fb_chans, max1302::SE_ZERO_PLUS_VREF);
	stage_out = new max5134_outputs<3>(*stage_dac, stage_chans);
}

