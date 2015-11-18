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
#include "gps_ref_time.h"
#include "gnss_sdr_supl_client.h"
#include "configuration_interface.h"
#include "gps_navigation_message.h"
#include "gps_ephemeris.h"

struct sEph{
    double Crs;
    double Crc;
    double time;
};

struct Satpos{
    double x;
    double y;
    double z;
    double time;
};
struct Subframe{
    std::string subframe;
    unsigned int subframe_id;
    unsigned int PRN;
    double timestamp;
};

struct SatBuff{
    int PRN;
    boost::circular_buffer<double> SNR_cb; 
    boost::circular_buffer<double> delta_cb; 
    boost::circular_buffer<double> RT_cb;
    double last_snr;
    double last_rt;
    double last_delta;
    int count = 0;

    void init(int cb_window){
        SNR_cb = boost::circular_buffer<double> (cb_window); 
        delta_cb = boost::circular_buffer<double> (cb_window); 
        RT_cb = boost::circular_buffer<double> (cb_window); 
    };

    void add(float CN0, float RT, float Delta){
        count++;

        double max_diff = 0.001;
        double snr = CN0; 
        double rt = RT; 
        double delta = Delta; 

        //if the difference between any two sample is more than 0.001
        //do not add value 
        if( last_snr && abs(snr - last_snr) < max_diff)
            {
                SNR_cb.push_back(snr);
            }

        if( last_rt && abs(rt - last_rt) < max_diff)
            {
                RT_cb.push_back(rt);
            }

        if( last_delta && abs(delta - last_delta) < max_diff)
            {
                delta_cb.push_back(delta);
            }

        last_snr = snr; 
        last_delta = delta; 
        last_rt = rt; 
    }
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
    Spoofing_Detector(ConfigurationInterface* configuration);

    void New_subframe(int subframe_ID, int PRN, int channel, int uid, Gps_Navigation_Message nav, double time);
    std::map<unsigned int, Satpos> Satpos_map;
    void check_position(double lat, double lng, double alt);
    void check_satpos(unsigned int sat, double time, double x, double y, double z); 
    double check_SNR(std::list<unsigned int> channels, Gnss_Synchro **in, int sample_counter);
    void check_external_utc(Gps_Utc_Model time_internal);
    void check_external_iono(Gps_Iono internal);
    // APT 
    bool d_APT;
    int d_APT_ch_per_sat;
    double d_APT_max_rx_discrepancy;

    //PPE 
    bool d_PPE;
    int d_PPE_window_size;
    boost::circular_buffer<double> ppe_cb;

    double d_PPE_sampling;
    double  d_CN0_threshold;
    double d_RT_threshold;
    double d_Delta_threshold;

    //NAVI configuration
    bool d_NAVI_TOW;
    double d_NAVI_TOW_max_discrepancy;
    bool d_NAVI_inter_satellite;
    bool d_NAVI_external;
    bool d_NAVI_alt;
    double d_NAVI_max_alt;
    sEph d_eph;
    double d_Crc;
    double d_Crs;

    std::map<int, boost::circular_buffer<double>> satellite_SNR;
    double get_SNR_corr(std::list<unsigned int> channels, Gnss_Synchro **in, int sample_counter);
    double get_corr(boost::circular_buffer<double> a, boost::circular_buffer<double> b);

    std::map<int, SatBuff> sat_buffs;
    void PPE_moving_var(std::list<unsigned int> channels, Gnss_Synchro **in, int sample_counter);
    void calc_mean_var(int sample_counter);
    void calc_max_var(int sample_counter);
    int snr_sum = 0;
    int delta_sum = 0;
    int rt_sum = 0; 
    int count = 0;
    
    //supl
    gnss_sdr_supl_client supl_client_;

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
    bool compare_subframes(Subframe subframeA, Subframe subframeB, int idA, int idB);
    bool compare_almanac(Gps_Almanac a, Gps_Almanac b);
    void lookup_external_nav_data(int source, int type);
    void set_supl_client();
    void check_new_TOW(double current_time_ms, int new_week, double new_TOW);
    void check_middle_earth(double sqrtA);
    void check_GPS_time();
    void check_inter_satellite_subframe(unsigned int uid, unsigned int subframe_id);
    void check_RX_time(unsigned int PRN, unsigned int subframe_id);
    void check_external_almanac(std::map<int,Gps_Almanac> internal);
    void check_external_gps_time(int internal_week, int internal_TOW);
    void check_external_ephemeris(Gps_Ephemeris internal, int PRN);
    void check_and_update_ephemeris(Gps_Ephemeris eph, double time);
};

#endif
