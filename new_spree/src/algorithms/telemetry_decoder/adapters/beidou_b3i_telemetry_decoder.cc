/*!
 * \file beidou_b3i_telemetry_decoder.cc
 * \brief Implementation of an adapter of a Beidou B3I NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#include "beidou_b3i_telemetry_decoder.h"
#include "configuration_interface.h"
#include <glog/logging.h>

BeidouB3iTelemetryDecoder::BeidouB3iTelemetryDecoder(
    ConfigurationInterface *configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams)
    : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    std::string default_dump_filename = "./navigation.dat";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ =
        configuration->property(role + ".dump_filename", default_dump_filename);
    // make telemetry decoder object
    telemetry_decoder_ = beidou_b3i_make_telemetry_decoder_gs(satellite_, dump_);
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


BeidouB3iTelemetryDecoder::~BeidouB3iTelemetryDecoder() = default;

void BeidouB3iTelemetryDecoder::set_state(unsigned int state)
{
}

void BeidouB3iTelemetryDecoder::set_satellite(const Gnss_Satellite &satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << satellite_;
}


void BeidouB3iTelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void BeidouB3iTelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr BeidouB3iTelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr BeidouB3iTelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}