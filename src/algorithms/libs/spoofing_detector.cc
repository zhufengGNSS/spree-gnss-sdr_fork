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



extern concurrent_queue<Spoofing_Message> global_spoofing_queue;
extern concurrent_map<double> global_last_gps_time;

struct Subframe{
    std::string subframe;
    unsigned int subframe_id;
    unsigned int PRN;
    double timestamp;
};
extern concurrent_map<Subframe> global_subframe_map;

struct GPS_time_t{
    int week;
    double TOW;
    double timestamp;
    int subframe_id;
};

extern concurrent_map<GPS_time_t> global_gps_time;
extern concurrent_map<std::map<unsigned int, unsigned int>> global_subframe_check;

const int seconds_per_week = 604800; 

using google::LogMessage;

Spoofing_Detector::Spoofing_Detector()
{
//    stdev_cb = boost::circular_buffer<double> (d_snr_moving_avg_window);
    //stdev_cb = boost::circular_buffer<double> (1000);
}

Spoofing_Detector::Spoofing_Detector(bool detect_spoofing, double max_rx_discrepancy, double max_tow_discrepancy)
{
    d_detect_spoofing = detect_spoofing;
    d_max_rx_discrepancy = max_rx_discrepancy/1e6; //[ns] -> [ms]
    d_max_tow_discrepancy = max_tow_discrepancy/1e3; //[ms] -> [s]
    //stdev_cb = boost::circular_buffer<double> (d_snr_moving_avg_window);
    //stdev_cb = boost::circular_buffer<double> (1000);
}

Spoofing_Detector::Spoofing_Detector(bool detect_spoofing, bool cno_detection, int cno_count, double cno_min, 
                                    bool alt_detection, double max_alt, bool satpos_detection, int snr_moving_avg_window)
{
    d_detect_spoofing = detect_spoofing;
    d_cno_detection = cno_detection;
    d_cno_count = cno_count;
    d_cno_min = cno_min;
    d_alt_detection = alt_detection;
    d_max_alt = max_alt;
    d_satpos_detection = satpos_detection;
    d_snr_moving_avg_window = snr_moving_avg_window;
    //stdev_cb = boost::circular_buffer<double> (d_snr_moving_avg_window);
    //stdev_cb = boost::circular_buffer<double> (1000);
}

Spoofing_Detector::~Spoofing_Detector()
{
}

void Spoofing_Detector::spoofing_detected(std::string description, int spoofing_case)
{
    DLOG(INFO) << "SPOOFING DETECTED " << description;
//    std::cout << "SPOOFING DETECTED " << description << std::endl;
    Spoofing_Message msg;
    msg.spoofing_case = spoofing_case;
    msg.description = description;
    global_spoofing_queue.push(msg);
}

// check that the position has normal values, that is is non negative and 
//below the configurable value alt 
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

// check that new ephemeris TOW (from a certain satellite) is consistent with the latest received TOW
// and the time duration between them 
void Spoofing_Detector::check_new_TOW(double current_timestamp_ms, int new_week, double new_TOW)
{
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
                        spoofing_detected(s.str(), 3);
                    }
            }
    }
    global_last_gps_time.write(0, new_week);
    global_last_gps_time.write(1, new_TOW);
    global_last_gps_time.write(2, current_timestamp_ms);
}

//Check for middle of earth attack
void Spoofing_Detector::check_middle_earth(double sqrtA)
{ 
    if(sqrtA == 0)
    {
        std::string s = "middle of the earth attack";
        spoofing_detected(s, 5);
    }
}  


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

            //DLOG(INFO) << "ts " << gps_time.timestamp;
            TOW = GPS_week*604800+gps_time.TOW;
            GPS_TOW.insert(TOW);
            subframe_IDs.insert(gps_time.subframe_id);
        }

    if(subframe_IDs.size() > 1 || std::abs(largest-smallest) > 30000) // 30 s 
    {
        DLOG(INFO) << "Not all satellites have received the latest subframe, don't compare GPS time " << subframe_IDs.size() << " " <<std::abs(largest-smallest);
    }
    else if(GPS_TOW.size() >1)
    {
        std::string s =  "satellites GPS TOW are not synced";
        spoofing_detected(s, 4);

        for(std::set<int>::iterator it = GPS_TOW.begin(); it != GPS_TOW.end(); ++it)
            {
                //DLOG(INFO) << "TOW " << *it; 
            }
        for(std::set<int>::iterator it = subframe_IDs.begin(); it != subframe_IDs.end(); ++it)
            {
                //DLOG(INFO) << "subframe " << *it; 
            }
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
   /* 
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
    */
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

//Check whether the reception time of two different peaks of the same satellite
//are within the configurable value d_maximum_deviation
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

        DLOG(INFO) << "id: " << it->first << " subframe: " << subframe.subframe_id << " timestamp " << subframe.timestamp;
    
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
                    spoofed = false;
                /*
                    diff = largest.id-smallest.id;
                    if(diff < 0 && diff != -4)
                        {
                            spoofed = true;
                        }
                    else if(diff > 1)
                        {
                            spoofed = true;
                        }
                    else if(std::abs(largest_t-smallest_t) > 6001) 
                        {
                            spoofed = true;
                        }
                */
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
            s << std::setprecision(10) << " RX times not consistent " << smallest_t << " "<< largest_t<< std::endl;
            s << std::setprecision(5) << "subframes: " << largest.subframe_id << " " << smallest.subframe_id << std::endl;
            s << "time difference: " << std::abs(largest_t-smallest_t)*1e6 << " [ns]" << std::endl;
            s << "distance: " << distance <<" [m]";
            spoofing_detected(s.str(), 1);
        }
}

//checks if the subframe retrieved (with subframe id: subframe_id) from different peaks from 
//the same satellite (same PRN)  are consistent
void Spoofing_Detector::check_ap_subframe(unsigned int uid, unsigned int PRN, unsigned int subframe_id)
{
    DLOG(INFO) << "check subframe " << subframe_id << " for " << uid;
    Subframe subframeA, subframeB;
    unsigned int id1, id2;
    std::map<int, Subframe> subframes = global_subframe_map.get_map_copy();
    if(subframes.count(uid))
        {
            subframeA = subframes.at(uid);
            id1 = uid;
        }
    else
        {
            DLOG(INFO) << "check subframe - but subframe for sat " << uid << " subframe: " << subframe_id << " not in subframe map"; 
            return;
        }

    for (std::map<int, Subframe>::iterator it = subframes.begin(); it!= subframes.end(); ++it)
    {
        subframeB = it->second;
        id2 = it->first;
        DLOG(INFO) << "subframeB " << subframeB.subframe_id << " " << id2 << " " << subframeB.PRN;
        DLOG(INFO) <<  (subframeB.PRN != PRN)  << " " << (subframeB.subframe_id != subframe_id) << " " << (id2 == id1);
        if(subframeB.PRN != PRN || subframeB.subframe_id != subframe_id || id2 == id1)
            continue;

        DLOG(INFO) << "check subframe "<< subframe_id << std::endl
        << subframeA.subframe << std::endl
        << subframeB.subframe;

        //one of the ephemeris data has not been updated.
        if(subframeA.timestamp == 0 ||  subframeB.timestamp == 0)
            {
                DLOG(INFO) << "Subframes timestamps are zero";
                continue;
            }

        //one of the ephemeris data has not been updated.
        if(std::abs(subframeA.timestamp -subframeB.timestamp) > 1)
            {
                DLOG(INFO) << "Subframes timestamps differ more than one" << std::endl
                << subframeA.timestamp << " " << subframeB.timestamp << std::endl
                << subframeA.subframe_id << " " << subframeB.subframe_id << std::endl
                << subframeA.subframe << std::endl << subframeB.subframe << std::endl;
                continue;
            }
            
        if(subframeA.subframe != subframeB.subframe && subframeA.subframe != "" && subframeB.subframe != "")
            {
                std::stringstream s;
                s << "Ephemeris data not consistent " << id1 << " " << id2;
                s << std::endl << "subframe id: " << subframe_id;
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
        
        //log if these two signals have been checked against each other
        if(subframe_id != 4 && subframe_id != 5)
            {
                std::string sid;
                if(id1 > id2)
                    {
                        sid = std::to_string(id1)+"-"+std::to_string(id2); 
                    }
                else
                    {
                        sid = std::to_string(id2)+"-"+std::to_string(id1); 
                    }
                unsigned int sum = 0; 
                DLOG(INFO) << "sid: " << sid;
                DLOG(INFO) << "id: " << stoi(sid);
                std::map<unsigned int, unsigned int> id1m;
                if(!global_subframe_check.read(id1, id1m)) 
                    {
                        sum = 0;
                    }
                else
                    {
                        if(id1m.count(id2))
                            sum = id1m.at(id2);
                        else
                            sum = 0;
                    }   
                ++sum;
                id1m[id2] = sum;
                global_subframe_check.add(id1, id1m); 

                sum = 0;
                std::map<unsigned int, unsigned int> id2m;
                if(!global_subframe_check.read(id2, id2m)) 
                    {
                        sum = 0;
                    }
                else
                    {
                        if(id2m.count(id1))
                            sum = id2m.at(id1);
                        else
                            sum = 0;
                    }   
                ++sum;
                id2m[id1] = sum;
                global_subframe_check.add(id2, id2m); 
            }
    }
}

//check that the peak of a satellite that is designated id1 has been
//checked against the peak id2 (that is checked that they have the same subframe
// for subframes 1,2,3)
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

// check whether the ephemeris data received from the satellites is consistent with
// ephemeris data received from an external source
void Spoofing_Detector::check_external_ephemeris(Gps_Ephemeris eph_internal, int PRN)
{
    std::map<int,Gps_Ephemeris> external;
    external = lookup_external_ephemeris(1);
        
    if(external.count( PRN) )
        {
            //create strings from the the ephemeris object for easy comparison
            Gps_Ephemeris eph_external = external.at( PRN );  
            bool the_same = compare_ephemeris(eph_internal, eph_external);

            if( !the_same )
                {
                    std::cout << "External ephemeris not consistent with ephemeris records from satellite " << PRN << std::endl;
                    LOG(INFO) << "External ephemeris not consistent with ephemeris records from satellite " << PRN; 

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



std::map<int,Gps_Ephemeris> Spoofing_Detector::lookup_external_ephemeris(int source)
{
    std::map<int,Gps_Ephemeris> result;
    gnss_sdr_supl_client supl_client_ephemeris_;
    std::string default_eph_server = "supl.google.com";
    supl_client_ephemeris_.server_name = default_eph_server; 
    int supl_mcc = 244;
    int supl_mns = 5; 
    int supl_lac = 0x59e2;
    int supl_ci = 0x31b0;
    switch( source )
    {
    case 1:
        // Request ephemeris from SUPL server
        int error;
        supl_client_ephemeris_.request = 1;
        std::cout << "SUPL: Try to read GPS ephemeris from SUPL server.." << std::endl;
        error = supl_client_ephemeris_.get_assistance(supl_mcc, supl_mns, supl_lac, supl_ci);
        if (error == 0)
        {
            //Save ephemeris to XML file
            std::string eph_xml_filename = "../data/ephemeris.xml"; 
            if (supl_client_ephemeris_.save_ephemeris_map_xml(eph_xml_filename, supl_client_ephemeris_.gps_ephemeris_map) == true)
            {
                std::cout << "SUPL: XML Ephemeris file created" << std::endl;
            }
            else
            {
                std::cout << "SUPL: Failed to create XML Ephemeris file" << std::endl;
            }

            result = supl_client_ephemeris_.gps_ephemeris_map;
        }
        else
        {
            std::cout << "ERROR: SUPL client for Ephemeris returned " << error << std::endl;
            std::cout << "Please check internet connection and SUPL server configuration" << error << std::endl;
        }
        break;
            
    case 2:
        
        std::string eph_xml_filename = "gps_ephemeris.xml"; 
        if (supl_client_ephemeris_.load_ephemeris_xml(eph_xml_filename) == true)
            {
                result = supl_client_ephemeris_.gps_ephemeris_map;
            }
        break;
    }

    return result;
}

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
}

