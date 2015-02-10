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
#include "concurrent_queue.h"
#include <math.h> 
#include <iomanip>

extern concurrent_queue<Spoofing_Message> global_spoofing_queue;
extern concurrent_map<double> global_last_gps_time;
using namespace std; 
struct Subframe{
    std::string subframe;
    unsigned int id;
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
extern concurrent_map<unsigned int> global_subframe_check;

using google::LogMessage;

Spoofing_Detector::Spoofing_Detector()
{
    d_detect_spoofing = false;
    d_max_discrepancy = 0; 
    d_max_alt = 0;
}
Spoofing_Detector::Spoofing_Detector(bool detect_spoofing, double max_discrepancy, double max_alt)
{
    d_detect_spoofing = detect_spoofing;
    d_max_discrepancy = max_discrepancy;
    d_max_alt = max_alt;
}

Spoofing_Detector::~Spoofing_Detector()
{

}

void Spoofing_Detector::spoofing_detected(std::string description, int spoofing_case)
{
    DLOG(INFO) << "SPOOFING DETECTED " << description;
    std::cout << "SPOOFING DETECTED " << description << std::endl;
    Spoofing_Message msg;
    msg.spoofing_case = spoofing_case;
    msg.description = description;
    global_spoofing_queue.push(msg);
}

// check that the position has norma values 
void Spoofing_Detector::check_position(double lat, double lng, double alt) 
{
    if(alt < 0)
        {
            string description = "Height of position is negative";
            spoofing_detected(description, 4);
        }
    else if(alt > d_max_alt)
        {
            string description = "Height of position is above 2 km";
            spoofing_detected(description, 4);
        }
}

// check that new ephemeris TOW is consitent with the latest received TOW
// and the time duration between them
void Spoofing_Detector::check_new_TOW(double current_time_ms, int new_week, double new_TOW)
{
    std::map<int, double> old_GPS_time;
    old_GPS_time = global_last_gps_time.get_map_copy();
    int old_time, new_time;
    double duration;
    if(old_GPS_time.size() > 2)
    {

        old_time = old_GPS_time.at(0)*604800+old_GPS_time.at(1); // 604800=7*24*60*60
        new_time = new_week*604800+new_TOW;
        duration = round((current_time_ms-old_GPS_time.at(2))/1000.0);
        //DLOG(INFO) << "check new TOW " << duration << " " << std::abs(new_time-old_time);

        //TODO: set configurable max deviation, now is 1 second
        if( std::abs(std::abs(new_time-old_time)-duration) > 1 && (round(current_time_ms) != 0 || round(old_GPS_time.at(2)) != 0) )
            {
                    
                if(old_time < new_time)
                    {
                        stringstream s;
                        s << " received new ephemeris TOW that is later than last received one and incorrect";
                        s << " difference: " << new_time-old_time;
                        s << " duration: " << duration << std::endl;
                        s << " gps times : " << new_time << " " << old_time; 
                        s << " times : " << round(current_time_ms) << " " << round(old_GPS_time.at(2)); 
                        spoofing_detected(s.str(), 3);
                    }
                else
                    {
                        stringstream s;
                        s << " received new ephemeris TOW that is earlier than last received one and incorrect";
                        s << " difference: " << new_time-old_time;
                        s << " duration: " << duration << std::endl;
                        s << " gps times : " << new_time << " " << old_time;
                        s << " times : " << round(current_time_ms) << " " << round(old_GPS_time.at(2)); 
                        spoofing_detected(s.str(), 3);
                    }
            }
    }
    global_last_gps_time.write(0, new_week);
    global_last_gps_time.write(1, new_TOW);
    global_last_gps_time.write(2, current_time_ms);
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
                    stringstream s;
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
    set<int> GPS_TOW;
    int GPS_week, TOW;
    set<int> subframe_IDs;
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

    if(subframe_IDs.size() > 1 || abs(largest-smallest) > 30000)
    {
        DLOG(INFO) << "Not all satellites have received the latest subframe, don't compare GPS time " << subframe_IDs.size() << " " <<abs(largest-smallest);
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

void Spoofing_Detector::check_RX(unsigned int PRN, unsigned int subframe_id)
{
    std::map<int, Subframe> subframes = global_subframe_map.get_map_copy();
    
    Subframe smallest = subframes.begin()->second;
    Subframe largest = subframes.begin()->second;
    Subframe subframe;
    
    for (std::map<int, Subframe>::iterator it = subframes.begin(); it!= subframes.end(); ++it)
    {
        subframe = it->second;
        
        if(subframe.PRN != PRN)
            continue;
    
        if(smallest.timestamp > subframe.timestamp) 
        {
            smallest = subframe;
        }

        if(largest.timestamp < subframe.timestamp) 
        {
            largest = subframe;
        }
    }
    
    double largest_t = largest.timestamp;
    double smallest_t = smallest.timestamp;
    bool spoofed = false;
    int diff = 0;

    if(std::abs(largest_t-smallest_t) > d_max_discrepancy)
        {
            if(largest.id != smallest.id)
                {
                    diff = largest.id-smallest.id;
                    if(diff < 0 && diff != -4)
                        {
                            spoofed = true;
                        }
                    else if(diff > 1)
                        {
                            spoofed = true;
                        }
                    else if(std::abs(largest_t-smallest_t) > 6000) 
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
            int c = 299792458; //[m/s] 
            double distance = std::abs(largest_t-smallest_t)*c/1e3; 
            stringstream s;
            s << " for satellite " << PRN;
            s << setprecision(10) << " RX times not consistent " << smallest_t << " "<< largest_t<< std::endl;
            s << setprecision(5) << "subframes: " << largest.id << " " << smallest.id << std::endl;
            s << "time difference: " << std::abs(largest_t-smallest_t)*1e6 << " [ns]" << std::endl;
            s << "distance: " << distance <<" [m]";
            spoofing_detected(s.str(), 1);
        }
}

void Spoofing_Detector::check_subframe(unsigned int uid, unsigned int PRN, unsigned int subframe_id)
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
        DLOG(INFO) << "subframeB " << subframeB.id << " " << id2 << " " << subframeB.PRN;
        DLOG(INFO) <<  (subframeB.PRN != PRN)  << " " << (subframeB.id != subframe_id) << " " << (id2 == id1);
        if(subframeB.PRN != PRN || subframeB.id != subframe_id || id2 == id1)
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
            
        if(subframeA.subframe != subframeB.subframe && subframeA.subframe != "" && subframeB.subframe != "")
            {
                stringstream s;
                s << "Ephemeris data not consistent " << id1 << " " << id2;
                s << std::endl << "subframe id: " << subframe_id;
                s << std::endl << "timestamps: " << subframeA.timestamp << " " << subframeB.timestamp; 
                s << std::endl << "subframes: ";
                s << std::endl << subframeA.subframe << std::endl << subframeB.subframe;

                spoofing_detected(s.str(), 2);

            }
        else
            {
                DLOG(INFO) << " subframes: " << endl
                << subframeA.timestamp << " " << subframeB.timestamp << std::endl
                << subframeA.id << " " << subframeB.id << std::endl
                << subframeA.subframe << std::endl << subframeB.subframe << std::endl;
            }
        
        //log if these two signals have been checked against each other
        if(subframe_id != 4 && subframe_id != 5)
            {
                std::string sid;
                if(id1 > id2)
                    {
                        sid = to_string(id1)+"-"+to_string(id2); 
                    }
                else
                    {
                        sid = to_string(id2)+"-"+to_string(id1); 
                    }
                unsigned int sum = 0; 
                unsigned int id = (unsigned int)stoi(sid);
                global_subframe_check.read(id, sum); 
                ++sum;
                global_subframe_check.add(id, sum); 
            }
    }
}


bool Spoofing_Detector::checked_subframes(unsigned int id1, unsigned int id2)
{
    unsigned int sum = 0;
    std::string sid;
    if(id1 > id2)
    {
        sid = to_string(id1)+"-"+to_string(id2); 
    }
    else
    {
        sid = to_string(id2)+"-"+to_string(id1); 
    }
    unsigned int id = (unsigned int)stoi(sid);
    global_subframe_check.read(id, sum); 
    if(sum < 3)
        {
            return false;
        }
    else
        {
            return true;
        }
}
