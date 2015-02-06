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
#include <set>
#include "concurrent_map.h"
#include "gps_ephemeris.h"
#include <string>
#include "gnss_synchro.h"
using namespace std; 
struct rx_time_t {
    int sat_id;
    double time;
    int subframe;
};

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
    Spoofing_Detector(bool detect_spoofing, double max_discrepancy, double max_alt);

    void RX_TX_ephemeris_check(std::list<unsigned int>channels, Gnss_Synchro **in, std::map<int, Gps_Ephemeris> ephemeris_map);
    void check_new_TOW(double current_time_ms, int new_week, double new_TOW);
    void check_middle_earth(double sqrtA);
    std::map<unsigned int, unsigned int> used_channel; 
    std::map<unsigned int, bool> checked_rx;
    std::map<unsigned int, bool> checked_channel_rx;
    std::map<unsigned int, Satpos> Satpos_map;
    //std::map<unsigned int, bool> checked_ephemeris;
    std::map<string, std::map<int, bool>> checked_subframe;
    bool checked(unsigned int PRN, unsigned int channel);
    void check_position(double lat, double lng, double alt);
    void check_satpos(unsigned int sat, double time, double x, double y, double z); 
    bool check_spoofing(std::list<unsigned int>channels, Gnss_Synchro **in);
    void check_GPS_time();

    bool d_detect_spoofing; 
    double d_max_discrepancy;
    double  d_max_alt;

    /*!
     * \brief Default destructor.
     */
    ~Spoofing_Detector();

private:
    set<unsigned int> new_subframe;
    bool check_RX(std::set<int> multiples, std::map<int, std::map<std::string, rx_time_t>> rx_times);
    void check_sat_RX(std::set<int> multiples, std::map<int, std::map<std::string,rx_time_t>> rx_times);
    bool check_subframes(std::map<int, std::list<int>> ids_of_PRN);
    void spoofing_detected(std::string description, int spoofing_case); 
};

#endif
