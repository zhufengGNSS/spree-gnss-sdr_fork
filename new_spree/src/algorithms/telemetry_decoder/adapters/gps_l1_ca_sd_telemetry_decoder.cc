/*!
 * \file gps_l1_ca_telemetry_decoder.cc
 * \brief Implementation of an adapter of a GPS L1 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gps_l1_ca_sd_telemetry_decoder.h"
#include "configuration_interface.h"
#include <glog/logging.h>


GpsL1CaSdTelemetryDecoder::GpsL1CaSdTelemetryDecoder(ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    std::string default_dump_filename = "./navigation.dat";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);

    fs_in = configuration->property("GNSS-SDR.internal_fs_sps",2e6);
    //Spoofing detection
    Spoofing_Detector *spoofing_detector = new Spoofing_Detector(configuration);
    
    // make telemetry decoder object
    telemetry_decoder_ = gps_l1_ca_make_sd_telemetry_decoder_gs(satellite_, dump_, *spoofing_detector, fs_in);  // TODO fix me
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";
    channel_ = 0;
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


GpsL1CaSdTelemetryDecoder::~GpsL1CaSdTelemetryDecoder() = default;


void GpsL1CaSdTelemetryDecoder::set_satellite(const Gnss_Satellite &satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << satellite_;
}


void GpsL1CaSdTelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void GpsL1CaSdTelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}

void GpsL1CaSdTelemetryDecoder::set_state(unsigned int state)
{
    telemetry_decoder_->set_state(state);
}

gr::basic_block_sptr GpsL1CaSdTelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr GpsL1CaSdTelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}