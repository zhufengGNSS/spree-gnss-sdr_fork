/*!
 * \file gps_l1_ca_sd_pvt.cc
 * \brief  Implementation of an adapter of a GPS L1 C/A PVT solver block to a
 * PvtInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gps_l1_ca_sd_pvt.h"
#include <glog/logging.h>
#include "configuration_interface.h"
#include "gps_l1_ca_sd_pvt_cc.h"

using google::LogMessage;

GpsL1CaSdPvt::GpsL1CaSdPvt(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
                role_(role),
                in_streams_(in_streams),
                out_streams_(out_streams),
                queue_(queue)
{
    // dump parameters
    std::string default_dump_filename = "./pvt.dat";
    std::string default_nmea_dump_filename = "./nmea_pvt.nmea";
    std::string default_nmea_dump_devname = "/dev/tty1";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    // moving average depth parameters
    int averaging_depth;
    averaging_depth = configuration->property(role + ".averaging_depth", 10);
    bool flag_averaging;
    flag_averaging = configuration->property(role + ".flag_averaging", false);
    // output rate
    int output_rate_ms;
    output_rate_ms = configuration->property(role + ".output_rate_ms", 500);
    // display rate
    int display_rate_ms;
    display_rate_ms = configuration->property(role + ".display_rate_ms", 500);
    // NMEA Printer settings
    bool flag_nmea_tty_port;
    flag_nmea_tty_port = configuration->property(role + ".flag_nmea_tty_port", false);
    std::string nmea_dump_filename;
    nmea_dump_filename = configuration->property(role + ".nmea_dump_filename", default_nmea_dump_filename);
    std::string nmea_dump_devname;
    nmea_dump_devname = configuration->property(role + ".nmea_dump_devname", default_nmea_dump_devname);

    //spoofing detection
    bool detect_spoofing = configuration->property("Spoofing.spoofing_detection", false);
    bool cno_detection = configuration->property("Spoofing.cno_detection", false);
    bool alt_detection = configuration->property("Spoofing.alt_detection", false);
    bool satpos_detection = configuration->property("Spoofing.satpos_detection", false);
    double max_alt = configuration->property("Spoofing.alt_max", 2e3);
    int cno_count = configuration->property("Spoofing.cno_count", 4);
    double cno_min = configuration->property("Spoofing.cno_min", 1.0);
    DLOG(INFO) << "cno_min: " << cno_min;
    Spoofing_Detector *spoofing_detector = new Spoofing_Detector(detect_spoofing, cno_detection, cno_count, cno_min, alt_detection,  max_alt, satpos_detection);

    // make PVT object
    pvt_ = gps_l1_ca_make_sd_pvt_cc(in_streams_, queue_, dump_, dump_filename_, averaging_depth, flag_averaging, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname, *spoofing_detector);
    DLOG(INFO) << "pvt(" << pvt_->unique_id() << ")";


}


GpsL1CaSdPvt::~GpsL1CaSdPvt()
{}


void GpsL1CaSdPvt::connect(gr::top_block_sptr top_block)
{
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void GpsL1CaSdPvt::disconnect(gr::top_block_sptr top_block)
{
    // Nothing to disconnect
}

gr::basic_block_sptr GpsL1CaSdPvt::get_left_block()
{
    return pvt_;
}


gr::basic_block_sptr GpsL1CaSdPvt::get_right_block()
{
    return pvt_;
}

