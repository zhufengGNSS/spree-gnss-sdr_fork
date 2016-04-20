/*!
* \file test_main.cc
* \brief This file implements all system tests.
* \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
*
*
* -------------------------------------------------------------------------
*
* Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
*
* GNSS-SDR is a software defined Global Navigation
* Satellite Systems receiver
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
*
* -------------------------------------------------------------------------
*/

#include <cmath>
#include <iostream>
#include <queue>
#include <memory>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gnuradio/msg_queue.h>
#include <gtest/gtest.h>
#include "concurrent_queue.h"
#include "concurrent_map.h"
#include "concurrent_map_str.h"
#include "control_thread.h"
#include "gps_navigation_message.h"

#include "gps_ephemeris.h"
#include "gps_cnav_ephemeris.h"
#include "gps_almanac.h"
#include "gps_iono.h"
#include "gps_cnav_iono.h"
#include "gps_utc_model.h"

#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"

#include "sbas_ephemeris.h"
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"
#include "sbas_satellite_correction.h"
#include "sbas_time.h"
#include "spoofing_message.h"



using google::LogMessage;

DECLARE_string(log_dir);

#include "arithmetic/complex_carrier_test.cc"
#include "arithmetic/conjugate_test.cc"
#include "arithmetic/magnitude_squared_test.cc"
#include "arithmetic/multiply_test.cc"
#include "arithmetic/code_generation_test.cc"
#include "arithmetic/tracking_loop_filter_test.cc"
#include "arithmetic/fft_length_test.cc"
#include "configuration/file_configuration_test.cc"
#include "configuration/in_memory_configuration_test.cc"
#include "control_thread/control_message_factory_test.cc"
#include "control_thread/control_thread_test.cc"
#include "flowgraph/pass_through_test.cc"
#include "flowgraph/gnss_flowgraph_test.cc"
#include "formats/string_converter_test.cc"
#include "formats/rtcm_test.cc"
#include "gnss_block/gnss_block_factory_test.cc"
#include "gnss_block/rtcm_printer_test.cc"
#include "gnss_block/file_signal_source_test.cc"
#include "gnss_block/fir_filter_test.cc"
#include "gnss_block/gps_l1_ca_pcps_acquisition_test.cc"
#include "gnss_block/gps_l2_m_pcps_acquisition_test.cc"
#include "gnss_block/gps_l1_ca_pcps_acquisition_gsoc2013_test.cc"
//#include "gnss_block/gps_l1_ca_pcps_multithread_acquisition_gsoc2013_test.cc"
#if OPENCL_BLOCKS_TEST
#include "gnss_block/gps_l1_ca_pcps_opencl_acquisition_gsoc2013_test.cc"
#endif
#include "gnss_block/gps_l1_ca_pcps_quicksync_acquisition_gsoc2014_test.cc"
#include "gnss_block/gps_l1_ca_pcps_tong_acquisition_gsoc2013_test.cc"
#include "gnss_block/galileo_e1_pcps_ambiguous_acquisition_test.cc"
#include "gnss_block/galileo_e1_pcps_ambiguous_acquisition_gsoc_test.cc"
#include "gnss_block/galileo_e1_pcps_ambiguous_acquisition_gsoc2013_test.cc"
#include "gnss_block/galileo_e1_pcps_8ms_ambiguous_acquisition_gsoc2013_test.cc"
#include "gnss_block/galileo_e1_pcps_tong_ambiguous_acquisition_gsoc2013_test.cc"
#include "gnss_block/galileo_e1_pcps_cccwsr_ambiguous_acquisition_gsoc2013_test.cc"
#include "gnss_block/galileo_e1_pcps_quicksync_ambiguous_acquisition_gsoc2014_test.cc"
#include "gnss_block/galileo_e1_dll_pll_veml_tracking_test.cc"
#include "gnuradio_block/gnss_sdr_valve_test.cc"
#include "gnuradio_block/direct_resampler_conditioner_cc_test.cc"
#include "gnss_block/galileo_e5a_pcps_acquisition_gsoc2014_gensource_test.cc"
#include "gnss_block/galileo_e5a_tracking_test.cc"
#include "gnss_block/gps_l2_m_dll_pll_tracking_test.cc"


// For GPS NAVIGATION (L1)

concurrent_queue<Gps_Acq_Assist> global_gps_acq_assist_queue;
concurrent_map<Gps_Acq_Assist> global_gps_acq_assist_map;

//For spoofing detection
struct GPS_time_t{
    int week;
    double TOW;
    double timestamp;    
    int subframe_id;
};

struct sEph{
    Gps_Ephemeris ephemeris;
    double time;
    bool changed;
};

concurrent_map<GPS_time_t> global_gps_time;
concurrent_map<sEph> global_sEph_map;
concurrent_map<double> global_last_gps_time;
concurrent_map<std::map<std::string, int>> global_code_phase;
//concurrent_map<bool> global_channel_status;
concurrent_map<int> global_channel_status;  //status 0: newly acquired, status 1: received the first 3 subframes, 
                                            //status 2: reset by PVT because no spoofing detected 
struct Subframe{
    std::string subframe;
    unsigned int id;
    unsigned int PRN;
    double timestamp;    
};

concurrent_map<Subframe> global_subframe_map;
concurrent_map<std::map<unsigned int, unsigned int>> global_subframe_check;
concurrent_queue<Spoofing_Message> global_spoofing_queue;


int main(int argc, char **argv)
{
    std::cout << "Running GNSS-SDR Tests..." << std::endl;
    int res = 0;
    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    try
    {
            res = RUN_ALL_TESTS();
    }
    catch(...)
    {
            LOG(WARNING) << "Unexpected catch";
    }
    google::ShutDownCommandLineFlags();
    return res;
}
