/*!
 * \file spoofing_detector.h
 * \brief module to detect spoofing 
 *
 * \author Hildur Ólafsdóttir, 2014, ohildur@gmail.com 
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_SPOOFING_DETECTOR_H_
#define	GNSS_SDR_SPOOFING_DETECTOR_H_


#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <vector>
#include <set>
#include "concurrent_map.h"
#include "gps_ephemeris.h"
#include <string>
#include "gnss_synchro.h"
#include <boost/circular_buffer.hpp>
#include "gps_iono.h"
#include "gps_almanac.h"
#include "gps_utc_model.h"

struct Satpos{
    double x;
    double y;
    double z;
    double time;
};

/*!
 * \brief provides spoofing detection 
 *
 */
class Spoofing_Detector
{
public:
    /*!
     * \brief Default constructor.
     */
    Spoofing_Detector();
    /*!
    * \brief Constructor called from telemetry 
    */
    Spoofing_Detector(bool detect_spoofing, double max_rx_discrepancy, double max_tow_discrepancy);
    /*!
    * \brief Constructor called from PVT 
    */
    Spoofing_Detector(bool detect_spoofing, bool cno_detection, int cno_count, double cno_min, 
                    bool alt_detection, double max_alt, bool satpos_detection, int snr_moving_avg_window);

    void check_new_TOW(double current_time_ms, int new_week, double new_TOW);
    void check_middle_earth(double sqrtA);
    std::map<unsigned int, Satpos> Satpos_map;
    void check_position(double lat, double lng, double alt);
    void check_satpos(unsigned int sat, double time, double x, double y, double z); 
    void check_GPS_time();
    void check_ap_subframe(unsigned int uid, unsigned int PRN, unsigned int subframe_id);
    void check_RX_time(unsigned int PRN, unsigned int subframe_id);
    bool checked_subframes(unsigned int id1, unsigned int id2);
    double check_SNR(std::list<unsigned int> channels, Gnss_Synchro **in, int sample_counter);
    void check_external_ephemeris(Gps_Ephemeris internal, int PRN);

    bool d_detect_spoofing = false; 
    bool d_cno_detection = false; 
    bool d_alt_detection = false; 
    bool d_satpos_detection = false; 
    double d_max_rx_discrepancy = 1e-6;
    double d_max_tow_discrepancy = 20e-3;
    double d_max_alt = 2e3;
    double d_cno_min = 1.0;
    int d_cno_count = 4;
    int d_snr_moving_avg_window = 1000;
    boost::circular_buffer<double> stdev_cb;


    /*!
     * \brief Default destructor.
     */
    ~Spoofing_Detector();

private:
    void spoofing_detected(std::string description, int spoofing_case); 
    double StdDeviation(std::vector<double> v);
    bool compare_ephemeris(Gps_Ephemeris a, Gps_Ephemeris b);
    bool compare_utc(Gps_Utc_Model a, Gps_Utc_Model b);
    bool compare_iono(Gps_Iono a, Gps_Iono b);
//    bool compare_almanac(Gps_Alamanc a, Gps_Almanac b);
    std::map<int,Gps_Ephemeris> lookup_external_ephemeris(int source);
};

#endif
