/*!
 * \file spoofing_detector.h
 * \brief module which performs several spoofing detection techniques.
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


#include "spoofing_detector.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "GPS_L1_CA.h"
#include "control_message_factory.h"
#include "spoofing_message.h"
#include "concurrent_map.h"
#include "concurrent_map_str.h"
#include "concurrent_queue.h"
#include <cmath>
#include <numeric>
#include <iomanip>
#include "gnss_sdr_supl_client.h"

const int seconds_per_week = 604800; 

/*!
 *  Contains all spoofing alarms.
 */
extern concurrent_queue<Spoofing_Message> global_spoofing_queue;

/*!
 *   Contains the last received GPS time
 *   key 0 contains the week.
 *   key 1 contains the TOW (time of week).
 *   key 2 contains the timestamp of when the GPS time was received.
 */
extern concurrent_map<double> global_last_gps_time;

extern concurrent_map<Subframe> global_subframe_map;

/*!
 *   Contains the GPS time, that is the GPS week and the time of week (TOW).
 *   In addition it contains the timestamp of when this information was received 
 *   and the id of the subframe that this information arrived in.
 */
struct GPS_time_t{
    int week;
    double TOW;
    double timestamp;
    int subframe_id;
};

/*!
 *   Contains the latest received GPS time of all currently tracked channels. 
 */
extern concurrent_map<GPS_time_t> global_gps_time;
/*!
 *  For each unique peak that is being tracked this maps it to all other peaks
 *  that it has been compared to i.e., has been tested for spoofing against. 
 */
extern concurrent_map<std::map<unsigned int, unsigned int>> global_subframe_check;


using google::LogMessage;

Spoofing_Detector::Spoofing_Detector()
{
    

}

/*!
 *  Constructor that is called from the telemetry decoder.
 *  d_ap_detection: whether to test for auxilary peaks.
 *  d_internal_satellite_check:
 *      whether to run spoofing test that test for inconsistencies
 *      amongst signals from different satellites. 
 *  d_external_nav_check: 
 *      whether to test the navigational messages against external sources.
 *  d_max_rx_discrepancy: 
 *      the maximum inconsistency that doens't raise a spoofing alarm between the
 *      reception time of two peaks that belong to the same satellite PRN.
 *  d_max_tow_discrepancy: the maximum difference between the duration between the arrival
 *      of two navigational messages and the difference in their GPS time.
 */
Spoofing_Detector::Spoofing_Detector(bool ap_detection, bool inter_satellite_check, bool external_nav_check, double max_rx_discrepancy, double max_tow_discrepancy)
{
    d_ap_detection = ap_detection;
    d_inter_satellite_check = inter_satellite_check;
    d_external_nav_check = external_nav_check;
    d_max_rx_discrepancy = max_rx_discrepancy/1e6; //[ns] -> [ms]
    DLOG(INFO) << "max dis " << d_max_rx_discrepancy;
    d_max_tow_discrepancy = max_tow_discrepancy/1e3; //[ms] -> [s]
}

/*!
 *  Constructor that is called from the PVT block.
 *  d_ap_detection: whether to test for auxilary peaks.
 *  d_cno_detection:
 *      whether to check if the standard deviation of the Carrier-to-Noise (CN0) estimator
 *      between satellites is too low.       
 *  d_cno_count: 
 *      The minumum number of satellites tracked befor the d_cno_detection is performed.
 *  d_cno_min:
 *      The minimum standard deviation of the Carrier-to-Noise (CN0) estimator that
 *      does not raise a spoofing alarm.
 *  d_alt_detection:
 *      Whether to compare the estimated height of the receiver to the maximum expected value
 *      and whether it is negative.
 *  d_max_alt:
 *      the maximum expected value of the receiver height.
 *  d_satpos_detection:
 *      whether to test if the calculated position of the satellite is moving to fast.
 *  d_snr_moving_avg_window:
 *      the window size of the moving average used to calculate the  standard deviation 
 *      of the Carrier-to-Noise (CN0) estimator.
 */
Spoofing_Detector::Spoofing_Detector(bool ap_detection, bool cno_detection, int cno_count, double cno_min, 
                                    bool alt_detection, double max_alt, bool satpos_detection, int snr_moving_avg_window, 
                                    int snr_delta_rt_cb_window)
{
    d_ap_detection = ap_detection;
    d_cno_detection = cno_detection;
    d_cno_count = cno_count;
    d_cno_min = cno_min;
    d_alt_detection = alt_detection;
    d_max_alt = max_alt;
    d_satpos_detection = satpos_detection;
    d_snr_moving_avg_window = snr_moving_avg_window;
    d_snr_delta_rt_cb_window = snr_delta_rt_cb_window;
    stdev_cb = boost::circular_buffer<double> (d_snr_moving_avg_window);
}

Spoofing_Detector::~Spoofing_Detector()
{
}

/*!
 *  Logs the occurance of a spoofing alarm.
 */
void Spoofing_Detector::spoofing_detected(std::string description, int spoofing_case)
{
    DLOG(INFO) << "SPOOFING DETECTED " << description;
    std::cout << "SPOOFING DETECTED " << description << std::endl;
    Spoofing_Message msg;
    msg.spoofing_case = spoofing_case;
    msg.description = description;
    global_spoofing_queue.push(msg);
}

/*! 
 *  Check that the estimated receiver position has normal values, that is is non negative and 
 *  below the configurable value alt 
 */
void Spoofing_Detector::check_position(double lat, double lng, double alt) 
{
    if(alt < 0)
        {
            std::string description = "Height of position is negative";
            spoofing_detected(description, 4);
        }
    else if(alt > d_max_alt)
        {
            std::stringstream s;
            s << "Height of position is above " <<alt << " km";
            std::string description = s.str();
            spoofing_detected(description, 4);
        }
}

/*!
 *  check that new ephemeris TOW (from a certain satellite) is consistent with the latest received TOW
 *  and the time duration between them. If the differnce in these values is above the maximum allowed
 *  value a spoofing alarm is raised.
 */
void Spoofing_Detector::check_new_TOW(double current_timestamp_ms, int new_week, double new_TOW)
{
    if( new_TOW == 0 || new_week == 0)
        return;

    std::map<int, double> old_GPS_time;
    old_GPS_time = global_last_gps_time.get_map_copy();
    int old_gps_time, new_gps_time;
    double duration;
    if(old_GPS_time.size() > 2)
    {
        double old_timestamp_ms = old_GPS_time.at(2);

        old_gps_time = old_GPS_time.at(0)*seconds_per_week+old_GPS_time.at(1);
        new_gps_time = new_week*seconds_per_week+new_TOW;
        //duration = round((current_timestamp_ms-old_timestamp_ms)/1000.0);
        duration = (current_timestamp_ms-old_timestamp_ms)/1000.0;
        //DLOG(INFO) << "check new TOW " << duration << " " << std::abs(new_time-old_time);

        if( std::abs(std::abs(new_gps_time-old_gps_time)-duration) > d_max_tow_discrepancy) 
            {
                    
                if(old_gps_time < new_gps_time)
                    {
                        std::stringstream s;
                        s << " received new ephemeris TOW that is later than last received one and incorrect";
                        s << " difference: " << new_gps_time-old_gps_time;
                        s << " duration: " << duration << std::endl;
                        s << " gps times : " << new_gps_time << " " << old_gps_time; 
                        s << " times : " << current_timestamp_ms << " " << old_timestamp_ms; 
                        s << " tow: " << new_TOW << " " << old_GPS_time.at(1);
                        s << " week: " << new_week << " " << old_GPS_time.at(0);
                        spoofing_detected(s.str(), 3);
                    }
                else
                    {
                        std::stringstream s;
                        s << " received new ephemeris TOW that is earlier than last received one and incorrect";
                        s << " difference: " << new_gps_time-old_gps_time;
                        s << " duration: " << duration << std::endl;
                        s << " gps times : " << new_gps_time << " " << old_gps_time; 
                        s << " times : " << current_timestamp_ms << " " << old_timestamp_ms; 
                        s << " tow: " << new_TOW << " " << old_GPS_time.at(1);
                        s << " week: " << new_week << " " << old_GPS_time.at(0);
                        spoofing_detected(s.str(), 3);
                    }
            }
    }

    global_last_gps_time.write(0, new_week);
    global_last_gps_time.write(1, new_TOW);
    global_last_gps_time.write(2, current_timestamp_ms);
}

/*!
 *  Check for middle of earth attack
 */
void Spoofing_Detector::check_middle_earth(double sqrtA)
{ 
    if(sqrtA == 0)
    {
        std::string s = "middle of the earth attack";
        spoofing_detected(s, 5);
    }
}  

/*!
 *  Checks whether the change in the estimated satellite position is changing faster then made
 *  possible given the satellites speed given that the receiver is moving at a "normal" speed.
 */
void Spoofing_Detector::check_satpos(unsigned int sat, double time, double x, double y, double z) 
{
    Satpos p;
    if(Satpos_map.count(sat))
        {
            p = Satpos_map.at(sat);
            double sat_speed = 1400e3/(60*60); // [m/s]
            double distance = sqrt(pow(p.x-x, 2)+pow(p.y-y, 2)+pow(p.z-z, 2));
            double time_diff = std::abs(time-p.time)/1000.0;
            //what to set as the max difference in position
            if(distance != 0  && ((distance - time_diff*sat_speed) > 500 || (distance - time_diff*sat_speed) < 10e3))
                {
                    std::stringstream s;
                    s << "New satellite position for sat: " << sat << " is further away from last reported position." << std::endl;
                    s << "  Distance: " << distance/1e3 << " [km] " << " time difference: " << time_diff << std::endl;
                    s << "  New pos: (" << p.x << ", " << p.y << ", " << p.z << ") old pos: (" << x << ", " << y << ", " << z << ")";
                    spoofing_detected(s.str(), 5);

                }
        }

    p.x = x;
    p.y = y;
    p.y = z;
    p.time = time;
    Satpos_map[sat] = p;
}

/*!
 *  Checks whether all tracked satellite signals are reporting the same GPS time.
 */
void Spoofing_Detector::check_GPS_time()
{
    std::map<int, GPS_time_t> gps_times = global_gps_time.get_map_copy();
    std::set<int> GPS_TOW;
    int GPS_week, TOW;
    std::set<int> subframe_IDs;
    double smallest = 0; 
    double largest = 0;
    GPS_time_t gps_time;
    //check that the GPS week is consistent between all satellites
    for(std::map<int, GPS_time_t>::iterator it = gps_times.begin(); it != gps_times.end(); ++it)
        {
            gps_time = it->second;
            GPS_week = gps_time.week; 
            if(GPS_week == 0)
                continue;

            if(gps_time.timestamp > largest)
                largest = gps_time.timestamp;
            if(gps_time.timestamp < smallest)
                smallest  = gps_time.timestamp;

            TOW = GPS_week*604800+gps_time.TOW;
            GPS_TOW.insert(TOW);
            subframe_IDs.insert(gps_time.subframe_id);
            DLOG(INFO) << "ts " << gps_time.timestamp << " TOW: " << TOW << " subframe: " << gps_time.subframe_id << " " << it->first;
        }

    /*if(std::abs(largest-smallest) > 29e3) // 29 s 
    {
        DLOG(INFO) << "Not all satellites have received the latest subframe, don't compare GPS time " << subframe_IDs.size() << " " <<std::abs(largest-smallest);
        return;
    }*/

    if(GPS_TOW.size() > subframe_IDs.size())
    {
        std::string s =  "satellites GPS TOW are not synced";
        spoofing_detected(s, 4);

        for(std::set<int>::iterator it = GPS_TOW.begin(); it != GPS_TOW.end(); ++it)
            {
                DLOG(INFO) << "TOW " << *it; 
            }
        for(std::set<int>::iterator it = subframe_IDs.begin(); it != subframe_IDs.end(); ++it)
            {
                DLOG(INFO) << "subframe " << *it; 
            }
    }
}

/*!
 *  Spoofing detection based on: calculating the variance of SNR, delta and RT over a given window for each satellite
 *  and calulate the mean of this. 
 */
//TODO: find better name
void Spoofing_Detector::SNR_delta_RT_calc(std::list<unsigned int> channels, Gnss_Synchro **in, int sample_counter)
{
    std::vector<unsigned int> PRNs;
    unsigned int PRN, i;
    for(std::list<unsigned int>::iterator it = channels.begin(); it != channels.end(); ++it)
    {
        i = *it;
        PRN  = in[i][0].PRN;
        PRNs.push_back(PRN);

        //we have a buffer with previous SNR samples
        if(!sat_buffs.count(PRN)) 
            {
                SatBuff satbuff;
                satbuff.init(d_snr_delta_rt_cb_window);
                satbuff.PRN = PRN;
                sat_buffs[PRN] = satbuff; 
            }
        sat_buffs.at(PRN).add(in[i][0]);
    }

    //remove satellites form the buffers if they are no longer being tracked
    std::list<unsigned int> sats_to_be_removed;
    for(std::map<int, SatBuff>::iterator it = sat_buffs.begin(); it != sat_buffs.end(); it++)
        {
            if( std::find(PRNs.begin(), PRNs.end(), it->first) == PRNs.end())
                {
                    sats_to_be_removed.push_back(it->first);
                }
        }

    for(std::list<unsigned int>::iterator it = sats_to_be_removed.begin(); it != sats_to_be_removed.end(); ++it)
    {
        sat_buffs.erase(*it); 
    }
    
    //caluclate the mean of the variances
    calc_mean_var(sample_counter);

}

double get_mean(boost::circular_buffer<double> v)
{
    double sum = 0;

    for(boost::circular_buffer<double>::iterator it = v.begin(); it != v.end(); it++)
        {
            sum += *it;
        }

    return (sum / v.size());
}

double get_cov(boost::circular_buffer<double> a, boost::circular_buffer<double> b)
{
    double mean_a = get_mean(a);
    double mean_b = get_mean(b);
    
    double sum = 0;
    if(a.size() != b.size())
        {
            DLOG(ERROR) << "vectors are not same length, can't calculate convariance";
            return 0;
        }
    if( a.size() == 0)
        {
            DLOG(INFO) << "vectors are empty, can't calculate convariance";
            return 0;
        }

    for(unsigned int i = 0; i < a.size(); i++)
        {
            sum += ( a[i]-mean_a ) * ( b[i]-mean_b ); 
        }
    return sum / a.size();
}

double get_var(boost::circular_buffer<double> a)
{
    double mean_a = get_mean(a);
    
    double sum = 0;
    if( a.size() == 0)
        {
            DLOG(INFO) << "vectors are empty, can't calculate convariance";
            return 0;
        }

    for(unsigned int i = 0; i < a.size(); i++)
        {
            sum += ( a[i]-mean_a ) * ( a[i]-mean_a ); 
        }
    return sum / a.size();
}

/*!
 *
 */
void Spoofing_Detector::calc_mean_var(int sample_counter)
{
    std::vector<double> snr_vars;
    std::vector<double> delta_vars;
    std::vector<double> rt_vars;
    double snr_var = 0;
    double delta_var = 0;
    double rt_var = 0;
    for(std::map<int, SatBuff>::iterator it = sat_buffs.begin(); it != sat_buffs.end(); it++)
        {
            SatBuff sb = it->second;
            if( sb.count < d_snr_delta_rt_cb_window )
                continue;

            snr_var = get_var(sb.SNR_cb);
            delta_var = get_var(sb.delta_cb);
            rt_var = get_var(sb.RT_cb); 
            
            snr_vars.push_back(snr_var);
            delta_vars.push_back(delta_var);
            rt_vars.push_back(rt_var);
        }

    if( snr_vars.size() == 0)
        return;

    double snr_mean = std::accumulate(snr_vars.begin(), snr_vars.end(), 0.0) / snr_vars.size(); 
    double delta_mean = std::accumulate(delta_vars.begin(), delta_vars.end(), 0.0) / delta_vars.size(); 
    double rt_mean = std::accumulate(rt_vars.begin(), rt_vars.end(), 0.0) / delta_vars.size(); 
    DLOG(INFO) << "SNR, delta, rt mean: " << snr_mean << " " << delta_mean << " " << rt_mean;

    if( snr_mean > 8 )
        {
            std::stringstream s;
            s << " the SNR indicates a spoofing attack"; 
            s << " SNR: " << snr_mean;
            s << ", " << sample_counter; 
            spoofing_detected(s.str(), 10);
        }
    if(  rt_mean > 0.7)
        {
            std::stringstream s;
            s << " the RT of the signal indicates a spoofing attack"; 
            s << " RT: " << rt_mean;
            s << ", " << sample_counter; 
            spoofing_detected(s.str(), 10);
        }

    if( snr_mean > 2) 
        snr_sum +=1; 
    if( rt_mean > 0.04) 
        rt_sum +=1; 
    if( delta_mean > 0.03) 
        delta_sum +=1; 
    count += 1;

    int window = 20;
    int threshold = 10;
    if( count >= window)
        {
            int sp = 0;
            if( snr_sum > threshold)
                sp +=1;
            if( rt_sum > threshold)
                sp +=1;
            if( delta_sum > threshold)
                sp +=1;

            if( sp >= 2) 
                {
                    std::stringstream s;
                    s << " the SNR and/or vestigial parameters suggest spoofing attack"; 
                    s << ", " << sample_counter; 
                    spoofing_detected(s.str(), 10);
                }
            
            count = 0;
            snr_sum = 0;
            rt_sum = 0;
            delta_sum = 0;
            
        }
}

double Spoofing_Detector::StdDeviation(std::vector<double> v)
{
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();
    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
        std::bind2nd(std::minus<double>(), mean));

    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size());
    return stdev;
}


double get_stdDev(std::vector<double> v, double mean)
{
    double sum = 0;

    for(std::vector<double>::iterator it = v.begin(); it != v.end(); it++)
        {
            sum += (*it-mean)*(*it-mean); 
        }
    return std::sqrt(sum / v.size());

}



double Spoofing_Detector::get_SNR_corr(std::list<unsigned int> channels, Gnss_Synchro **in, int sample_counter)
{
    int window_size = 1e3;
    std::vector<unsigned int> PRNs;
    unsigned int PRN, i;
    for(std::list<unsigned int>::iterator it = channels.begin(); it != channels.end(); ++it)
    {
        i = *it;
        PRN  = in[i][0].PRN;
        PRNs.push_back(PRN);
        //we have a buffer with previous SNR samples
        if(!satellite_SNR.count(PRN)) 
            {
                satellite_SNR[PRN] =  boost::circular_buffer<double> (window_size); 
            }
        satellite_SNR.at(PRN).push_back(in[i][0].CN0_dB_hz);
    }

    std::list<unsigned int> sats_to_be_removed;
    //remove satellites form the buffers if they are no longer being tracked
    for(std::map<int, boost::circular_buffer<double>>::iterator it = satellite_SNR.begin(); it != satellite_SNR.end(); it++)
        {
            if( std::find(PRNs.begin(), PRNs.end(), it->first) == PRNs.end())
                {
                    sats_to_be_removed.push_back(it->first);
                }
        }

    for(std::list<unsigned int>::iterator it = sats_to_be_removed.begin(); it != sats_to_be_removed.end(); ++it)
    {
        satellite_SNR.erase(*it); 
    }

    double p_corr;
    int PRN_a, PRN_b;
    double corr_sum = 0;
    for(unsigned int i = 0; i < PRNs.size(); i++) 
    {
        PRN_a  = PRNs.at(i); 
        for(unsigned int j = i+1; j < PRNs.size(); j++) 
        {
            PRN_b = PRNs.at(j);
            p_corr = get_corr(satellite_SNR.at(PRN_a), satellite_SNR.at(PRN_b));
            corr_sum += p_corr;
        } 
    }

    if(corr_sum > 3)
    {
        std::stringstream s;
        s << " the SNR correlation is above expected values, "; 
        s << " SNR: " << corr_sum; 
        s << ", " << sample_counter; 
        spoofing_detected(s.str(), 10);
    }

}

double Spoofing_Detector::get_corr(boost::circular_buffer<double> a, boost::circular_buffer<double> b)
{
    unsigned int window_size = 1e3;
    if(a.size() < window_size || b.size() < window_size)
        {
            //DLOG(INFO) << "don't have enough SNR values to calculate correlation";
            return 0; 
        }
    
    double corr = get_cov(a, b) / (get_cov(a, a) * get_cov(b, b)); 
    return corr;

}

double Spoofing_Detector::check_SNR(std::list<unsigned int> channels, Gnss_Synchro **in, int sample_counter)
{
    if(channels.size() < d_cno_count)
        {
            return 0;
        }

    std::vector<double> SNRs;
    unsigned int i;
    for(std::list<unsigned int>::iterator it = channels.begin(); it != channels.end(); ++it)
    {
        i = *it;
        SNRs.push_back(in[i][0].CN0_dB_hz);
    }

    double stdev = StdDeviation(SNRs); 
    double mv_avg;
    
    stdev_cb.push_back(stdev);
    if(stdev_cb.size() >= 1000)
        {
            double sum = std::accumulate(stdev_cb.begin(), stdev_cb.end(), 0);
            mv_avg = sum/stdev_cb.size();
            if(mv_avg < d_cno_min)
                {
                    std::stringstream s;
                    s << " the SNR stdev is below expected values, "; 
                    s << " SNR: " << mv_avg; 
                    s << ", " << sample_counter; 
                    spoofing_detected(s.str(), 10);
                }
        }
    
    //calculate pearson correlation
    //double p_corr = get_cov(SNR

   /* 
    if(stdev < d_cno_min)
        {
            std::stringstream s;
            s << " the SNR stdev is below expected values, "; 
            s << " SNR: " << stdev; 
            s << ", " << sample_counter; 
            spoofing_detected(s.str(), 10);
        }*/
    return stdev;
}

/*!
 *  Check whether the reception time of two different peaks of the same satellite
 *  are within the configurable value d_maximum_deviation
 */
void Spoofing_Detector::check_RX_time(unsigned int PRN, unsigned int subframe_id)
{
    std::map<int, Subframe> subframes = global_subframe_map.get_map_copy();
     
    Subframe smallest;
    Subframe largest;
    Subframe subframe;

    for (std::map<int, Subframe>::iterator it = subframes.begin(); it!= subframes.end(); ++it)
    {
        subframe = it->second;
        
        if(subframe.PRN != PRN) 
            continue;
        smallest = subframe;
        largest = subframe;
        break;
    }
    
    for (std::map<int, Subframe>::iterator it = subframes.begin(); it!= subframes.end(); ++it)
    {
        subframe = it->second;
        
        if(subframe.PRN != PRN)
            continue;

        //DLOG(INFO) << "id: " << it->first << " subframe: " << subframe.subframe_id << " timestamp " << std::setprecision(10)<< subframe.timestamp;
    
        if(smallest.timestamp > subframe.timestamp) 
        {
            smallest = subframe;
        }

        if(largest.timestamp < subframe.timestamp) 
        {
            largest = subframe;
        }
    }
    
    //the earliest and latest reception times
    double largest_t = largest.timestamp;
    double smallest_t = smallest.timestamp;
    bool spoofed = false;
    int diff = 0;

    if(std::abs(largest_t-smallest_t) > d_max_rx_discrepancy)
        {
            if(largest.subframe_id != smallest.subframe_id)
                {
                    diff = largest.subframe_id-smallest.subframe_id;
                    if(std::abs(largest_t-smallest_t)/std::fmod(diff, 5) > 6001) 
                        {
                            spoofed = true;
                        }
                }
            else
                {
                    spoofed = true;
                }
        }

    if(spoofed)
        {
            double distance = std::abs(largest_t-smallest_t)*GPS_C_m_s/1e3; 
            std::stringstream s;
            s << " for satellite " << PRN;
            s << std::setprecision(16) << " RX times not consistent " << smallest_t << " "<< largest_t << " " << largest_t-smallest_t <<  std::endl;
            s << std::setprecision(5) << "subframes: " << largest.subframe_id << " " << smallest.subframe_id << std::endl;
            s << "time difference: " << std::abs(largest_t-smallest_t)*1e6 << " [ns]" << std::endl;
            s << std::setprecision(16) << "distance: " << distance <<" [m]";
            std::cout << "RX " <<  PRN << std::setprecision(15) << smallest_t << " "<< largest_t << " " << std::abs(largest_t-smallest_t)*1e6 << " " << distance << std::endl;
            spoofing_detected(s.str(), 1);
        }
}

/*!
 *  Check if two subframes contain the same values, if not raise a spoofing alarm.
 *  Return true if the subframes were compared but false if they were not.
 */
bool Spoofing_Detector::compare_subframes(Subframe subframeA, Subframe subframeB, int idA, int idB)
{
        DLOG(INFO) << "check subframe "<< subframeA.subframe_id << std::endl
        << subframeA.subframe << std::endl
        << subframeB.subframe;

        //one of the ephemeris data has not been updated.
        if(subframeA.timestamp == 0 ||  subframeB.timestamp == 0)
            {
                DLOG(INFO) << "Subframes timestamps are zero";
                return 0;
            }
/*
        //one of the ephemeris data has not been updated.
        if(std::abs(subframeA.timestamp -subframeB.timestamp) > 3000)
            {
                DLOG(INFO) << "Subframes timestamps differ more than one" << std::endl
                << subframeA.timestamp << " " << subframeB.timestamp << std::endl
                << subframeA.subframe_id << " " << subframeB.subframe_id << std::endl
                << subframeA.subframe << std::endl << subframeB.subframe << std::endl;
                return 0;
            }
  */          
        if(subframeA.subframe != subframeB.subframe && subframeA.subframe != "" && subframeB.subframe != "")
            {
                std::stringstream s;
                s << "Ephemeris data not consistent " << idA << " " << idB;
                s << std::endl << "subframe id: " << subframeA.subframe_id;
                s << std::endl << "timestamps: " << subframeA.timestamp << " " << subframeB.timestamp; 
                s << std::endl << "subframes: ";
                s << std::endl << subframeA.subframe << std::endl << subframeB.subframe;

                spoofing_detected(s.str(), 2);

            }
        else
            {
                DLOG(INFO) << " subframes: " << std::endl
                << subframeA.timestamp << " " << subframeB.timestamp << std::endl
                << subframeA.subframe_id << " " << subframeB.subframe_id << std::endl
                << subframeA.subframe << std::endl << subframeB.subframe << std::endl;
            }
    return 1;
}

/*!
 *  checks if the subframe retrieved (with subframe id: subframe_id) from different peaks from 
 *  the same satellite (same PRN)  are consistent
 */
void Spoofing_Detector::check_ap_subframe(unsigned int uid, unsigned int PRN, unsigned int subframe_id)
{
    DLOG(INFO) << "check subframe " << subframe_id << " for " << uid;
    Subframe subframeA, subframeB;
    unsigned int idA, idB;
    std::map<int, Subframe> subframes = global_subframe_map.get_map_copy();
    if(subframes.count(uid))
        {
            subframeA = subframes.at(uid);
            idA = uid;
        }
    else
        {
            DLOG(INFO) << "check subframe - but subframe for sat " << uid << " subframe: " << subframe_id << " not in subframe map"; 
            return;
        }

    for (std::map<int, Subframe>::iterator it = subframes.begin(); it!= subframes.end(); ++it)
    {
        subframeB = it->second;
        idB = it->first;
        DLOG(INFO) << "subframeB " << subframeB.subframe_id << " " << idB << " " << subframeB.PRN;
        DLOG(INFO) <<  (subframeB.PRN != PRN)  << " " << (subframeB.subframe_id != subframe_id) << " " << (idB == idA);
        if(subframeB.PRN != PRN || subframeB.subframe_id != subframe_id || idB == idA)
            continue;

        if( !compare_subframes(subframeA, subframeB, idA, idB))
            continue;
        
        //log if these two signals have been checked against each other
        if(subframe_id != 4 && subframe_id != 5)
            {
                std::string sid;
                if(idA > idB)
                    {
                        sid = std::to_string(idA)+"-"+std::to_string(idB); 
                    }
                else
                    {
                        sid = std::to_string(idB)+"-"+std::to_string(idA); 
                    }
                unsigned int sum = 0; 
                DLOG(INFO) << "sid: " << sid;
                DLOG(INFO) << "id: " << stoi(sid);
                std::map<unsigned int, unsigned int> idAm;
                if(!global_subframe_check.read(idA, idAm)) 
                    {
                        sum = 0;
                    }
                else
                    {
                        if(idAm.count(idB))
                            sum = idAm.at(idB);
                        else
                            sum = 0;
                    }   
                ++sum;
                idAm[idB] = sum;
                global_subframe_check.add(idA, idAm); 

                sum = 0;
                std::map<unsigned int, unsigned int> idBm;
                if(!global_subframe_check.read(idB, idBm)) 
                    {
                        sum = 0;
                    }
                else
                    {
                        if(idBm.count(idA))
                            sum = idBm.at(idA);
                        else
                            sum = 0;
                    }   
                ++sum;
                idBm[idA] = sum;
                global_subframe_check.add(idB, idBm); 
            }
    }
}

/*
 *  Check that the peak of a satellite that is designated id1 has been
 *  checked against the peak id2 (that is checked that they have the same subframe
 *  for subframes 1,2,3)
*/
bool Spoofing_Detector::checked_subframes(unsigned int id1, unsigned int id2)
{
    std::map<unsigned int, unsigned int> check;
    //id1 is not in the checked map and has thus not been checked
    //against any other signal
    if(!global_subframe_check.read(id1, check))
        {
            return false; 
        }
    //id1 has not been checked against id2
    else if (!check.count(id2))
        {
            return false; 
        }
    //id1 and id2 have been checked against one and other
    //but not for all of the subframes
    else if(check.at(id2) < 3)
        {
            return false;
        }
    else
        {
            return true;
        }
}

/*!
 *  Check if the shared subframes 4 and 5 are the same from all satellites
 */
void Spoofing_Detector::check_inter_satellite_subframe(unsigned int uid, unsigned int subframe_id)
{
    DLOG(INFO) << "check subframe " << subframe_id << " for " << uid;
    Subframe subframeA, subframeB;
    unsigned int idA, idB;
    std::map<int, Subframe> subframes = global_subframe_map.get_map_copy();
    if(subframes.count(uid))
        {
            subframeA = subframes.at(uid);
            idA = uid;
        }
    else
        {
            DLOG(INFO) << "check subframe - but subframe for sat " << uid << " subframe: " << subframe_id << " not in subframe map"; 
            return;
        }

    for (std::map<int, Subframe>::iterator it = subframes.begin(); it!= subframes.end(); ++it)
    {
        subframeB = it->second;
        idB = it->first;
        DLOG(INFO) << "subframeB " << subframeB.subframe_id << " " << idB << " " << subframeB.PRN;
        DLOG(INFO) <<  (subframeB.subframe_id != subframe_id) << " " << (idB == idA);
        if(subframeB.subframe_id != subframe_id || idB == idA)
            continue;
        
        compare_subframes(subframeA, subframeB, idA, idB);
    }
}

/*!
 *  Check whether the ephemeris data received from the satellites is consistent with
 *  ephemeris data received from an external source
 */
void Spoofing_Detector::check_external_ephemeris(Gps_Ephemeris eph_internal, int PRN)
{
    lookup_external_nav_data(1,1);
    std::map<int,Gps_Ephemeris> external;
    external = supl_client_.gps_ephemeris_map;    
        
    if(external.count( PRN) )
        {
            //create strings from the the ephemeris object for easy comparison
            Gps_Ephemeris eph_external = external.at( PRN );  
            bool the_same = compare_ephemeris(eph_internal, eph_external);

            if( !the_same )
                {
                    std::cout << "External ephemeris not consistent with ephemeris records from satellite " << PRN << std::endl;
                    LOG(INFO) << "External ephemeris not consistent with ephemeris records from satellite " << PRN; 
                    std::stringstream s;
                    s << "External ephemeris not consistent with ephemeris records from satellite " << PRN;
                    spoofing_detected(s.str(), 0);

                }
            else
                {
                    LOG(INFO) << "External ephemeris are consistent with ephemeris records from satellite " << PRN;
                }
        }
    else
        {
            LOG(INFO) << "No external ephemeris record for satellite " << PRN;
        }
}

/*! 
 *  check whether the UTC Model data received from the satellites is consistent with
 *  UTC model data received from an external source
 */
void Spoofing_Detector::check_external_utc(Gps_Utc_Model internal)
{
    lookup_external_nav_data(1,0);
    Gps_Utc_Model external;
    external = supl_client_.gps_utc;
        
    if( external.valid && internal.valid )
        {
            //create strings from the the ephemeris object for easy comparison
            bool the_same = compare_utc(internal, external);

            if( !the_same )
                {
                    std::cout << "External utc model not consistent with records from satellites " << std::endl;
                    LOG(INFO) << "Externautc utc model not consistent with records from satellites "; 

                    std::stringstream s;
                    s << "Externautc utc model not consistent with records from satellites "; 
                    spoofing_detected(s.str(), 0);
                }
            else
                {
                    LOG(INFO) << "External utc model are consistent with records from satellites ";
                }
        }
    else
        {
            LOG(INFO) << "No external utc model record for satellites ";
        }
}

/*!
 *  Check whether the Iono Model data received from the satellites is consistent with
 *  Iono model data received from an external source.
 */
void Spoofing_Detector::check_external_iono(Gps_Iono internal)
{
    lookup_external_nav_data(1,0);
    Gps_Iono external;
    external = supl_client_.gps_iono;
        
    if( external.valid && internal.valid )
        {
            //create strings from the the ephemeris object for easy comparison
            bool the_same = compare_iono(internal, external);

            if( !the_same )
                {
                    std::cout << "External iono data not consistent with records from satellites " << std::endl;
                    LOG(INFO) << "Externautc iono data not consistent with records from satellites "; 
                    std::stringstream s;
                    s << "Externautc iono data not consistent with records from satellites "; 
                    spoofing_detected(s.str(), 0);
                }
            else
                {
                    LOG(INFO) << "External iono data are consistent with records from satellites ";
                }
        }
    else
        {
            LOG(INFO) << "No external iono data record for satellites ";
        }
}

/*! 
 *  Check whether the Gps time received from the satellites is consistent with
 *  Gps_Ref_Time model data received from an external source
 */
void Spoofing_Detector::check_external_gps_time(int internal_week, int internal_TOW)
{
    lookup_external_nav_data(1,0);
    Gps_Ref_Time external;
    external = supl_client_.gps_time;

    int internal_time = internal_week*seconds_per_week+internal_TOW;
    
    if( external.valid )
        {
            int external_time = external.d_Week*seconds_per_week+external.d_TOW; 
            //create strings from the the ephemeris object for easy comparison
            bool the_same = external_time == internal_time; 

            if( !the_same )
                {
                    std::cout << "External gps time not consistent with records from satellites " << std::endl;
                    LOG(INFO) << "Externautc gps time not consistent with records from satellites "; 
                    std::stringstream s;
                    s << "Externautc gps time not consistent with records from satellites "; 
                    spoofing_detected(s.str(), 0);
                }
            else
                {
                    LOG(INFO) << "External gps time is consistent with records from satellites ";
                }
        }
    else
        {
            LOG(INFO) << "No external gps time record for satellites ";
        }
}

/*!
 *  Check whether the Almanac data received from the satellites is consistent with
 *  Almanac data received from an external source
 */
void Spoofing_Detector::check_external_almanac(std::map<int, Gps_Almanac> internal_map)
{
    lookup_external_nav_data(1,0);
    std::map< int, Gps_Almanac> external_map;
    external_map = supl_client_.gps_almanac_map;
    
    int PRN;
    Gps_Almanac internal;
    for(std::map<int, Gps_Almanac>::iterator it = internal_map.begin(); it != internal_map.end(); it++)
        { 
            PRN = it->first;
            internal = it->second; 
            if(external_map.count( PRN) )
                {
                    //create strings from the the ephemeris object for easy comparison
                    Gps_Almanac external= external_map.at( PRN );  
                    bool the_same = compare_almanac(internal, external);

                    if( !the_same )
                        {
                            std::cout << "External almanac data not consistent with records from satellite " << PRN << std::endl;
                            LOG(INFO) << "Externautc almanac data not consistent with records from satellite " << PRN; 
                            std::stringstream s;
                            s << "Externautc almanac data not consistent with records from satellite " << PRN; 
                            spoofing_detected(s.str(), 0);
                        }
                    else
                        {
                            LOG(INFO) << "External almanac data is consistent with records from satellite " << PRN;
                        }
                }
            else
                {
                    LOG(INFO) << "No external almanac data record for satellite " << PRN;
                }
        }
}

/*!
 *  Loopup external ephemeris, ionospheric, almanac data or utc model
 *  source states from where this information is retrieved (e.g. 1:supl server, 2:file)
 *  type specifies the type of data that is asked for (1: ephemeris, 0: iono, alma, utc)
 */
void Spoofing_Detector::lookup_external_nav_data(int source, int type)
{
    //std::string d_default_eph_server = "supl.google.com";
    std::string d_default_eph_server = "supl.nokia.com";
    supl_client_.server_name = d_default_eph_server;
    supl_client_.server_port = 7275;
    int supl_mcc = 244;
    int supl_mns = 5; 
    int supl_lac = 0x59e2;
    int supl_ci = 0x31b0;

    std::map<int,Gps_Ephemeris> result;
    switch( source )
    {
    case 1:
        // Request ephemeris from SUPL server
        int error;
        supl_client_.request = type;
        error = supl_client_.get_assistance(supl_mcc, supl_mns, supl_lac, supl_ci);
        if (error == 0)
        {
            if( type == 1 )
                {
                    std::cout << "SUPL: Try to read GPS ephemeris from SUPL server.." << std::endl;
                    //Save ephemeris to XML file
                    std::string eph_xml_filename = "../data/ephemeris.xml"; 
                    if (supl_client_.save_ephemeris_map_xml(eph_xml_filename, supl_client_.gps_ephemeris_map) == true)
                    {
                        std::cout << "SUPL: XML Ephemeris file created" << std::endl;
                    }
                    else
                    {
                        std::cout << "SUPL: Failed to create XML Ephemeris file" << std::endl;
                    }
                }
            else if( type == 0)
                {
                    std::cout << "SUPL: Try to iono, alamanc and utc model from SUPL server.." << std::endl;
                    //Save iono, utc model and almanac to XML file
                    std::string utc_xml_filename = "../data/utc.xml"; 
                    std::map<int, Gps_Utc_Model> utc_map;
                    utc_map[0] = supl_client_.gps_utc;
                    if (supl_client_.save_utc_map_xml(utc_xml_filename, utc_map) == true)
                    {
                        std::cout << "SUPL: XML Utc model file created" << std::endl;
                    }
                    else
                    {
                        std::cout << "SUPL: Failed to create XML Utc model file" << std::endl;
                    }

                    std::string iono_xml_filename = "../data/iono.xml"; 
                    std::map<int, Gps_Iono> iono_map;
                    iono_map[0] = supl_client_.gps_iono;
                    if (supl_client_.save_iono_map_xml(iono_xml_filename, iono_map) == true)
                    {
                        std::cout << "SUPL: XML iono model file created" << std::endl;
                    }
                    else
                    {
                        std::cout << "SUPL: Failed to create XML iono model file" << std::endl;
                    }

                    std::string ref_time_xml_filename = "../data/ref_time.xml"; 
                    std::map<int, Gps_Ref_Time> ref_time_map;
                    ref_time_map[0] = supl_client_.gps_time;
                    if (supl_client_.save_ref_time_map_xml(ref_time_xml_filename, ref_time_map) == true)
                        {
                            LOG(INFO) << "SUPL: Successfully saved Ref Time XML file";
                        }
                    else
                        {
                            LOG(INFO) << "SUPL: Error while trying to save ref time XML file";
                        }
                }
        }
        else
        {
            std::cout << "ERROR: SUPL client returned " << error << std::endl;
            std::cout << "Please check internet connection and SUPL server configuration" << error << std::endl;
        }
        break;
            
    case 2:
        std::string eph_xml_filename = "gps_ephemeris.xml"; 
        std::string utc_xml_filename = "gps_utc.xml"; 
        std::string iono_xml_filename = "gps_iono.xml"; 

        // Try to read ephemeris from XML
        if (supl_client_.load_ephemeris_xml(eph_xml_filename) == true)
            {
                LOG(INFO) << "SUPL: Read XML Ephemeris data";
            }
        else
            {
                LOG(INFO) << "SUPL: couldn't read Ephemeris data XML";
            }

        // Try to read utc model from XML
        if (supl_client_.load_utc_xml(utc_xml_filename) == true)
            {
                LOG(INFO) << "SUPL: Read XML UTC model";
            }
        else
            {
                LOG(INFO) << "SUPL: couldn't read UTC model XML";
            }

        // Try to read Iono model from XML
        if (supl_client_.load_iono_xml(iono_xml_filename) == true)
            {
                LOG(INFO) << "SUPL: Read XML IONO model";
            }
        else
            {
                LOG(INFO) << "SUPL: couldn't read IONO model XML";
            }

        break;
    }

}

/*!
 * Compare two sets of ephemeris data.
 */
bool Spoofing_Detector::compare_ephemeris(Gps_Ephemeris a, Gps_Ephemeris b)
{
    if( a.i_satellite_PRN != b.i_satellite_PRN)
        {
            DLOG(INFO) << "Comparing ephemeris of two different satellites";
            return true;
        }
    bool the_same = true; 

    if( a.i_peak != b.i_peak )
        {
            the_same = false;
            DLOG(INFO) << "i_peak not the same: " << a.i_peak << " " << b.i_peak;
        }
    if( a.d_TOW != b.d_TOW )
        {
            the_same = false;
            DLOG(INFO) << "d_TOW not the same: " << a.d_TOW << " " << b.d_TOW;
        }
    if( a.d_Crs != b.d_Crs )
        {
            the_same = false;
            DLOG(INFO) << "d_Crs not the same: " << a.d_Crs << " " << b.d_Crs;
        }
    if( a.d_Delta_n != b.d_Delta_n )
        {
            the_same = false;
            DLOG(INFO) << "d_Delta_n not the same: " << a.d_Delta_n << " " << b.d_Delta_n;
        }
    if( a.d_M_0 != b.d_M_0 )
        {
            the_same = false;
            DLOG(INFO) << "d_M_0 not the same: " << a.d_M_0 << " " << b.d_M_0;
        }
    if( a.d_Cuc != b.d_Cuc )
        {
            the_same = false;
            DLOG(INFO) << "d_Cuc not the same: " << a.d_Cuc << " " << b.d_Cuc;
        }
    if( a.d_e_eccentricity != b.d_e_eccentricity )
        {
            the_same = false;
            DLOG(INFO) << "d_e_eccentricity not the same: " << a.d_e_eccentricity << " " << b.d_e_eccentricity;
        }
    if( a.d_Cus != b.d_Cus )
        {
            the_same = false;
            DLOG(INFO) << "d_Cus not the same: " << a.d_Cus << " " << b.d_Cus;
        }
    if( a.d_sqrt_A != b.d_sqrt_A )
        {
            the_same = false;
            DLOG(INFO) << "d_sqrt_A not the same: " << a.d_sqrt_A << " " << b.d_sqrt_A;
        }
    if( a.d_Toe != b.d_Toe )
        {
            the_same = false;
            DLOG(INFO) << "d_Toe not the same: " << a.d_Toe << " " << b.d_Toe;
        }
    if( a.d_Toc != b.d_Toc )
        {
            the_same = false;
            DLOG(INFO) << "d_Toc not the same: " << a.d_Toc << " " << b.d_Toc;
        }
    if( a.d_Cic != b.d_Cic )
        {
            the_same = false;
            DLOG(INFO) << "d_Cic not the same: " << a.d_Cic << " " << b.d_Cic;
        }
    if( a.d_OMEGA0 != b.d_OMEGA0 )
        {
            the_same = false;
            DLOG(INFO) << "d_OMEGA0 not the same: " << a.d_OMEGA0 << " " << b.d_OMEGA0;
        }
    if( a.d_Cis != b.d_Cis )
        {
            the_same = false;
            DLOG(INFO) << "d_Cis not the same: " << a.d_Cis << " " << b.d_Cis;
        }
    if( a.d_i_0 != b.d_i_0 )
        {
            the_same = false;
            DLOG(INFO) << "d_i_0 not the same: " << a.d_i_0 << " " << b.d_i_0;
        }
    if( a.d_Crc != b.d_Crc )
        {
            the_same = false;
            DLOG(INFO) << "d_Crc not the same: " << a.d_Crc << " " << b.d_Crc;
        }
    if( a.d_OMEGA != b.d_OMEGA )
        {
            the_same = false;
            DLOG(INFO) << "d_OMEGA not the same: " << a.d_OMEGA << " " << b.d_OMEGA;
        }
    if( a.d_OMEGA_DOT != b.d_OMEGA_DOT )
        {
            the_same = false;
            DLOG(INFO) << "d_OMEGA_DOT not the same: " << a.d_OMEGA_DOT << " " << b.d_OMEGA_DOT;
        }
    if( a.d_IDOT != b.d_IDOT )
        {
            the_same = false;
            DLOG(INFO) << "d_IDOT not the same: " << a.d_IDOT << " " << b.d_IDOT;
        }
    if( a.i_code_on_L2 != b.i_code_on_L2 )
        {
            the_same = false;
            DLOG(INFO) << "i_code_on_L2 not the same: " << a.i_code_on_L2 << " " << b.i_code_on_L2;
        }
    if( a.i_GPS_week != b.i_GPS_week )
        {
            the_same = false;
            DLOG(INFO) << "i_GPS_week not the same: " << a.i_GPS_week << " " << b.i_GPS_week;
        }
    if( a.b_L2_P_data_flag != b.b_L2_P_data_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_L2_P_data_flag not the same: " << a.b_L2_P_data_flag << " " << b.b_L2_P_data_flag;
        }
    if( a.i_SV_accuracy != b.i_SV_accuracy )
        {
            the_same = false;
            DLOG(INFO) << "i_SV_accuracy not the same: " << a.i_SV_accuracy << " " << b.i_SV_accuracy;
        }
    if( a.i_SV_health != b.i_SV_health )
        {
            the_same = false;
            DLOG(INFO) << "i_SV_health not the same: " << a.i_SV_health << " " << b.i_SV_health;
        }
    if( a.d_TGD != b.d_TGD )
        {
            the_same = false;
            DLOG(INFO) << "d_TGD not the same: " << a.d_TGD << " " << b.d_TGD;
        }
    if( a.d_IODC != b.d_IODC )
        {
            the_same = false;
            DLOG(INFO) << "d_IODC not the same: " << a.d_IODC << " " << b.d_IODC;
        }
    if( a.i_AODO != b.i_AODO )
        {
            the_same = false;
            DLOG(INFO) << "i_AODO not the same: " << a.i_AODO << " " << b.i_AODO;
        }
    if( a.b_fit_interval_flag != b.b_fit_interval_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_fit_interval_flag not the same: " << a.b_fit_interval_flag << " " << b.b_fit_interval_flag;
        }
    if( a.d_spare1 != b.d_spare1 )
        {
            the_same = false;
            DLOG(INFO) << "d_spare1 not the same: " << a.d_spare1 << " " << b.d_spare1;
        }
    if( a.d_spare2 != b.d_spare2 )
        {
            the_same = false;
            DLOG(INFO) << "d_spare2 not the same: " << a.d_spare2 << " " << b.d_spare2;
        }
    if( a.d_A_f0 != b.d_A_f0 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f0 not the same: " << a.d_A_f0 << " " << b.d_A_f0;
        }
    if( a.d_A_f1 != b.d_A_f1 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f1 not the same: " << a.d_A_f1 << " " << b.d_A_f1;
        }
    if( a.d_A_f2 != b.d_A_f2 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f2 not the same: " << a.d_A_f2 << " " << b.d_A_f2;
        }
    if( a.b_integrity_status_flag != b.b_integrity_status_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_integrity_status_flag not the same: " << a.b_integrity_status_flag << " " << b.b_integrity_status_flag;
        }
    if( a.b_alert_flag != b.b_alert_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_alert_flag not the same: " << a.b_alert_flag << " " << b.b_alert_flag;
        }
    if( a.b_antispoofing_flag != b.b_antispoofing_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_antispoofing_flag not the same: " << a.b_antispoofing_flag << " " << b.b_antispoofing_flag;
        }
    if( a.d_spare1 != b.d_spare1 )
        {
            the_same = false;
            DLOG(INFO) << "d_spare1 not the same: " << a.d_spare1 << " " << b.d_spare1;
        }
    if( a.d_spare2 != b.d_spare2 )
        {
            the_same = false;
            DLOG(INFO) << "d_spare2 not the same: " << a.d_spare2 << " " << b.d_spare2;
        }
    if( a.d_A_f0 != b.d_A_f0 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f0 not the same: " << a.d_A_f0 << " " << b.d_A_f0;
        }
    if( a.d_A_f1 != b.d_A_f1 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f1 not the same: " << a.d_A_f1 << " " << b.d_A_f1;
        }
    if( a.d_A_f2 != b.d_A_f2 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f2 not the same: " << a.d_A_f2 << " " << b.d_A_f2;
        }
    if( a.b_integrity_status_flag != b.b_integrity_status_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_integrity_status_flag not the same: " << a.b_integrity_status_flag << " " << b.b_integrity_status_flag;
        }
    if( a.b_alert_flag != b.b_alert_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_alert_flag not the same: " << a.b_alert_flag << " " << b.b_alert_flag;
        }
    if( a.b_antispoofing_flag != b.b_antispoofing_flag )
        {
            the_same = false;
            DLOG(INFO) << "b_antispoofing_flag not the same: " << a.b_antispoofing_flag << " " << b.b_antispoofing_flag;
        }

    return the_same;
}


/*!
 * Compare two sets of almanac data.
 */
bool Spoofing_Detector::compare_almanac(Gps_Almanac a, Gps_Almanac b)
{
    if( a.i_satellite_PRN != b.i_satellite_PRN)
        {
            DLOG(INFO) << "Comparing almanac data of two different satellites";
            return true;
        }

    bool the_same = true; 
    if( a.d_Delta_i != b.d_Delta_i )
        {
            the_same = false;
            DLOG(INFO) << "d_Delta_i not the same: " << a.d_Delta_i << " " << b.d_Delta_i;
        }
    if( a.d_Toa != b.d_Toa )
        {
            the_same = false;
            DLOG(INFO) << "d_Toa not the same: " << a.d_Toa << " " << b.d_Toa;
        }
    if( a.d_M_0 != b.d_M_0 )
        {
            the_same = false;
            DLOG(INFO) << "d_M_0 not the same: " << a.d_M_0 << " " << b.d_M_0;
        }
    if( a.d_e_eccentricity != b.d_e_eccentricity )
        {
            the_same = false;
            DLOG(INFO) << "d_e_eccentricity not the same: " << a.d_e_eccentricity << " " << b.d_e_eccentricity;
        }
    if( a.d_sqrt_A != b.d_sqrt_A )
        {
            the_same = false;
            DLOG(INFO) << "d_sqrt_A not the same: " << a.d_sqrt_A << " " << b.d_sqrt_A;
        }
    if( a.d_OMEGA0 != b.d_OMEGA0 )
        {
            the_same = false;
            DLOG(INFO) << "d_OMEGA0 not the same: " << a.d_OMEGA0 << " " << b.d_OMEGA0;
        }
    if( a.d_OMEGA != b.d_OMEGA )
        {
            the_same = false;
            DLOG(INFO) << "d_OMEGA not the same: " << a.d_OMEGA << " " << b.d_OMEGA;
        }
    if( a.d_OMEGA_DOT != b.d_OMEGA_DOT )
        {
            the_same = false;
            DLOG(INFO) << "d_OMEGA_DOT not the same: " << a.d_OMEGA_DOT << " " << b.d_OMEGA_DOT;
        }
    /*
    if( a.i_SV_health != b.i_SV_health )
        {
            the_same = false;
            DLOG(INFO) << "i_SV_health not the same: " << a.i_SV_health << " " << b.i_SV_health;
        }
    */
    if( a.d_A_f0 != b.d_A_f0 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f0 not the same: " << a.d_A_f0 << " " << b.d_A_f0;
        }
    if( a.d_A_f1 != b.d_A_f1 )
        {
            the_same = false;
            DLOG(INFO) << "d_A_f1 not the same: " << a.d_A_f1 << " " << b.d_A_f1;
        }

    return the_same;
}

/*!
 * Compare two Gps Iono models
 */
bool Spoofing_Detector::compare_iono(Gps_Iono a, Gps_Iono b)
{
    bool the_same = true; 
    if( a.d_alpha0 != b.d_alpha0 )
        {
            the_same = false;
            DLOG(INFO) << "d_alpha0 not the same: " << a.d_alpha0 << " " << b.d_alpha0;
        }
    if( a.d_alpha1 != b.d_alpha1 )
        {
            the_same = false;
            DLOG(INFO) << "d_alpha1 not the same: " << a.d_alpha1 << " " << b.d_alpha1;
        }
    if( a.d_alpha2 != b.d_alpha2 )
        {
            the_same = false;
            DLOG(INFO) << "d_alpha2 not the same: " << a.d_alpha2 << " " << b.d_alpha2;
        }
    if( a.d_alpha3 != b.d_alpha3 )
        {
            the_same = false;
            DLOG(INFO) << "d_alpha3 not the same: " << a.d_alpha3 << " " << b.d_alpha3;
        }
    if( a.d_beta0 != b.d_beta0 )
        {
            the_same = false;
            DLOG(INFO) << "d_beta0 not the same: " << a.d_beta0 << " " << b.d_beta0;
        }
    if( a.d_beta1 != b.d_beta1 )
        {
            the_same = false;
            DLOG(INFO) << "d_beta1 not the same: " << a.d_beta1 << " " << b.d_beta1;
        }
    if( a.d_beta2 != b.d_beta2 )
        {
            the_same = false;
            DLOG(INFO) << "d_beta2 not the same: " << a.d_beta2 << " " << b.d_beta2;
        }
    if( a.d_beta3 != b.d_beta3 )
        {
            the_same = false;
            DLOG(INFO) << "d_beta3 not the same: " << a.d_beta3 << " " << b.d_beta3;
        }
    if( a.valid != b.valid )
        {
            the_same = false;
            DLOG(INFO) << "valid not the same: " << a.valid << " " << b.valid;
        }
    return the_same;
}

/*!
 * Compare two Gps Utc models
 */
bool Spoofing_Detector::compare_utc(Gps_Utc_Model a, Gps_Utc_Model b)
{
    bool the_same = true;
    if( a.valid != b.valid )
        {
            the_same = false;
            DLOG(INFO) << "valid not the same: " << a.valid << " " << b.valid;
        }
    if( a.d_A1 != b.d_A1 )
        {
            the_same = false;
            DLOG(INFO) << "d_A1 not the same: " << a.d_A1 << " " << b.d_A1;
        }
    if( a.d_A0 != b.d_A0 )
        {
            the_same = false;
            DLOG(INFO) << "d_A0 not the same: " << a.d_A0 << " " << b.d_A0;
        }
    if( a.d_t_OT != b.d_t_OT )
        {
            the_same = false;
            DLOG(INFO) << "d_t_OT not the same: " << a.d_t_OT << " " << b.d_t_OT;
        }
    if( a.i_WN_T != b.i_WN_T )
        {
            the_same = false;
            DLOG(INFO) << "i_WN_T not the same: " << a.i_WN_T << " " << b.i_WN_T;
        }
    if( a.d_DeltaT_LS != b.d_DeltaT_LS )
        {
            the_same = false;
            DLOG(INFO) << "d_DeltaT_LS not the same: " << a.d_DeltaT_LS << " " << b.d_DeltaT_LS;
        }
    if( a.i_WN_LSF != b.i_WN_LSF )
        {
            the_same = false;
            DLOG(INFO) << "i_WN_LSF not the same: " << a.i_WN_LSF << " " << b.i_WN_LSF;
        }
    if( a.i_DN != b.i_DN )
        {
            the_same = false;
            DLOG(INFO) << "i_DN not the same: " << a.i_DN << " " << b.i_DN;
        }
    if( a.d_DeltaT_LSF != b.d_DeltaT_LSF )
        {
            the_same = false;
            DLOG(INFO) << "d_DeltaT_LSF not the same: " << a.d_DeltaT_LSF << " " << b.d_DeltaT_LSF;
        } 
    return the_same;
}

