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

extern concurrent_queue<Spoofing_Message> global_spoofing_queue;
extern concurrent_map<double> global_last_gps_time;
using namespace std; 
struct Subframe{
    std::string subframe;
    int id;
    double timestamp;
};
extern concurrent_map<Subframe> global_channel_to_subframe;

struct GPS_time_t{
    int week;
    double TOW;
    double timestamp;
    int subframe_id;
};

extern concurrent_map<GPS_time_t> global_gps_time;

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

void Spoofing_Detector::check_sat_RX(std::set<int> satellites, std::map<int, std::map<string,rx_time_t>> rx_times)
{
    // If the received time for any of the signals of tsatellite is larger
    // than 1 second TODO: make configurable 
    double smallest = 0;
    double largest = 0;
    for(std::set<int>::iterator it = satellites.begin(); it != satellites.end(); ++it)
        {
            if(rx_times.count(*it))
                {
                    
                    if(rx_times.at(*it).at("smallest").time > largest)
                        largest = rx_times.at(*it).at("smallest").time;
        
                    if(rx_times.at(*it).at("smallest").time < smallest)
                        largest = rx_times.at(*it).at("smallest").time;

                }
        }
        if(std::abs(largest-smallest) > 1)
        {
            DLOG(INFO) << "SPOOFING rx times of satellites are more than a second apart " << smallest << " "<< largest;
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
            if(distance != 0  && abs(distance - time_diff*sat_speed) > 500)
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

bool Spoofing_Detector::check_RX(std::set<int> satellites, std::map<int, std::map<string, rx_time_t>> rx_times)
{
    // If the received time for any of the signals of the same satellite is larger
    // than the maximum allowed discrepancy assume one is a spoofed signal
    double largest, smallest;
    int subframe1, subframe2, diff;
    bool spoofed_rx = false;
    bool checked = false;
    for(std::set<int>::iterator it = satellites.begin(); it != satellites.end(); ++it)
        {
            checked = true;
            spoofed_rx = false;
            if(rx_times.count(*it))
                {
                    checked_rx[*it] = true;
                    largest =  rx_times.at(*it).at("largest").time;
                    smallest =  rx_times.at(*it).at("smallest").time;
                    //DLOG(INFO) << "check rx for "<< *it << "times "<< largest << " " << smallest ;
                    if(std::abs(largest-smallest) > d_max_discrepancy)
                        {
                            subframe1 = rx_times.at(*it).at("smallest").subframe;
                            subframe2 = rx_times.at(*it).at("largest").subframe;
                            //DLOG(INFO) << "subframes " << subframe1 << " " << subframe2; 
                            if(subframe2 != subframe1)
                                {
                                    //DLOG(INFO) << "subframes " << subframe1 << " " << subframe2; 
                                    diff = subframe2-subframe1;
                                    //DLOG(INFO) << "diff " << diff;
                                    if(diff < 0 && diff != -4)
                                        {
                                            spoofed_rx = true;
                                        }
                                    else if(diff > 1)
                                        {
                                            spoofed_rx = true;
                                        }
                                    else if(std::abs(largest-smallest) > 6000) 
                                        {
                                            spoofed_rx = true;
                                        }
                                    else
                                        {
                                            checked_rx[*it] = false;
                                            checked = false;
                                        }
                                }
                            else
                                spoofed_rx = true;
                        }
                }

                if(spoofed_rx)
                    {
                        stringstream s;
                        s << " for sat: " << *it;
                        s << " RX times not consistent " << smallest << " "<< largest << " "<< diff;
                        spoofing_detected(s.str(), 1);
                    }
        }
    return checked;
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

bool Spoofing_Detector::check_subframes(std::map<int, std::list<int>> ids_of_PRN)
{
    int id1, id2;
    Subframe subframeA, subframeB;
    std::map<int, Subframe> subframes = global_channel_to_subframe.get_map_copy();
    bool checked = false;
    for (std::map<int,list<int>>::iterator it=ids_of_PRN.begin(); it!=ids_of_PRN.end(); ++it)
    {
        std::list<int> ids = it->second;  
        //DLOG(INFO) << "check subframes for sat " << it->first << " " <<ids.size(); 
        while(!ids.empty())
            {
                checked  = true;
                id1 = ids.front();
                ids.pop_front();
                if(subframes.count(id1))
                    {
                        subframeA = subframes.at(id1); 
                        break;
                    }
            }
        
        while(!ids.empty())
            {
                id2 = ids.front();
                //DLOG(INFO) << "Check subframes for " << id1 << " " << id2;
                ids.pop_front();
                if(!subframes.count(id2))
                    continue;
                
                subframeB = subframes.at(id2); 
                DLOG(INFO) << "check subframe "<< subframeA.id << std::endl
                       << subframeA.subframe << std::endl
                       << subframeB.subframe;
                //one of the ephemeris data has not been updated.
                if(subframeA.id != subframeB.id || subframeA.timestamp == 0 ||  subframeB.timestamp == 0)
                    {
                        DLOG(INFO) << "Subframes are not updated"
                                   << " " << subframeA.id
                                   << " " << subframeB.id
                                   << " " << subframeA.timestamp
                                   << " " << subframeB.timestamp;
                        checked = false;
                        continue;
                    }
                
                if(subframeA.subframe != subframeB.subframe && subframeA.subframe != "" && subframeB.subframe != "")
                    {
                        stringstream s;
                        s << "Ephemeris data not consistent " << id1 << " " << id2;
                        s << std::endl << subframeA.timestamp << " " << subframeB.timestamp; 
                        s << std::endl << subframeA.id<< " " << subframeB.id; 
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
                std::string id;
                if(id1 > id2)
                    {
                        id = to_string(id1)+"-"+to_string(id2); 
                    }
                else
                    {
                        id = to_string(id2)+"-"+to_string(id1); 
                    }
                checked_subframe[id][subframeA.id] = true;
            }
    }
    return checked;
}

bool Spoofing_Detector::checked(unsigned int PRN, unsigned int channel)
{
    if(!used_channel.count(PRN))
        {
            //DLOG(INFO) << "PRN is not in used channel";
            return false; 
        }

    unsigned int u_channel = used_channel.at(PRN);
    unsigned int id1 = channel; 
    unsigned int id2 = used_channel.at(PRN);
    //DLOG(INFO) << "checked ? " << channel << " " << u_channel;
    if(!checked_rx.count(PRN))
        {
            //DLOG(INFO) << "The satellite has not been checked yet " << u_channel;
            return false;
        }
    else
        {
            //there was no comparison of the RX times for satellite PRN
            if(!checked_rx.at(PRN))
                {
                    //DLOG(INFO) << "No RX checked has been performed";
                    return false;
                }

            //Either the main channel or the auxiliary channel was not checked during the RX check
            if(checked_channel_rx.count(u_channel) && checked_channel_rx.count(channel)) 
                {
                    if(!(checked_channel_rx.at(u_channel) && checked_channel_rx.at(channel))) 
                        {
                            //DLOG(INFO) << "No RX checked has been performed on the channels 1";
                            return false;
                        }
                }
            else
                {   
                    //DLOG(INFO) << "No RX checked has been performed on the channels 2";
                    return false;
                }
        }
    std::string id;
    if(id1 > id2)
        {
            id = to_string(id1)+"-"+to_string(id2); 
        }
    else
        {
            id = to_string(id2)+"-"+to_string(id1); 
        }

    if(!(checked_subframe.count(id)))
        {
            //DLOG(INFO) << "subframes not checked ";
            return false;
        }
    else
        {
            if(!(checked_subframe.at(id).count(1) && checked_subframe.at(id).at(1)))
                {
                    //DLOG(INFO) << "subframe 1 not checked for channel " <<channel;
                    return false;
                }
            if(!(checked_subframe.at(id).count(2) && checked_subframe.at(id).at(2)))
                {
                    //DLOG(INFO) << "subframe 2 not checked for channel " <<channel;
                    return false;
                }
            if(!(checked_subframe.at(id).count(3) && checked_subframe.at(id).at(3)))
                {
                    //DLOG(INFO) << "subframe 3 not checked for channel " <<channel;
                    return false;
                }
        }
    //Since the
    checked_subframe.erase(id);
    return true;
}


bool Spoofing_Detector::check_spoofing(std::list<unsigned int> channels, Gnss_Synchro **in)
{
    bool check = false; 
    unsigned int i = 0;
    for(std::list<unsigned int>::iterator it = channels.begin(); it != channels.end(); ++it)
        {
            i = *it;
            if( in[i][0].new_subframe == true)
                {
                    check = true;
                    DLOG(INFO) << "new subframe " << in[i][0].PRN;
                    new_subframe.insert(i);
                }
            else if(find(new_subframe.begin(), new_subframe.end(), i) != new_subframe.end())
                {
                    check = true;
                    DLOG(INFO) << "new subframe not found " << in[i][0].PRN;
                    for(set<unsigned int>::iterator it = new_subframe.begin(); it != new_subframe.end(); ++it)
                        {
                            DLOG(INFO) << *it;
                        }
                }
        }

    return check;
}


void Spoofing_Detector::RX_TX_ephemeris_check(std::list<unsigned int> channels, 
                                                                Gnss_Synchro **in, 
                                                                std::map<int, 
                                                                Gps_Ephemeris> ephemeris_map )
{
    std::map<int, std::map<string, rx_time_t >> rx_times;
    std::set<int> satellites;
    std::map<int, std::list<int>> ids_of_PRN;
    double rx_time;
    unsigned int PRN;
    unsigned int i = 0;

    for(std::list<unsigned int>::iterator it = channels.begin(); it != channels.end(); ++it)
        {
            i = *it;
            DLOG(INFO) << "check spoofing channel " << i;
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    PRN = in[i][0].PRN;
                    std::string tmp = std::to_string(PRN);
                    satellites.insert(PRN);
                    tmp += "0"+std::to_string(in[i][0].peak)+"0"+std::to_string(i);
                    int sat_id = std::stoi(tmp);

                    rx_time = in[i][0].rx_of_subframe;
                    if(rx_time == 0)
                        continue;

            
                    DLOG(INFO) << "spoofing check " << sat_id;
        
                    rx_time_t p;
                    p.sat_id = sat_id;
                    p.time = rx_time;
                    p.subframe = in[i][0].subframe;
                    
                    if(!rx_times.count(PRN))
                        {
                          //DLOG(INFO) << "received first rx for sat " << PRN;
                          std::map<string, rx_time_t> tmp;
                          tmp["smallest"] = p; 
                          tmp["largest"] = p; 
                          rx_times[PRN] = tmp;

                        }
                    else
                        {
                            std::map<string, rx_time_t> rx = rx_times.at(PRN);
                            if(rx.at("smallest").time > rx_time)// && 
                                {
                                    rx_times.at(PRN)["smallest"] = p;
                                }
                            else if(rx.at("largest").time < rx_time)
                                {
                                    rx_times.at(PRN)["largest"] = p;
                                }
                        }

                    if(!ids_of_PRN.count(in[i][0].PRN))
                        {
                            std::list<int> tmp;
                            tmp.push_back(sat_id);
                            ids_of_PRN[in[i][0].PRN] = tmp;
                        }
                    else
                        {
                            ids_of_PRN.at(in[i][0].PRN).push_back(sat_id); 
                       }
                    //DLOG(INFO) << "checked channel rx " << sat_id;
                checked_channel_rx[sat_id] = true;
                }

            //DLOG(INFO) << "sat " << PRN << " smallest " << rx_times.at(PRN).at("smallest").second << " largest " <<rx_times.at(PRN).at("largest").second;
        }
    bool c1 = false; 
    bool c2 = false;
    c1 = check_RX(satellites, rx_times);
    c2 = check_subframes(ids_of_PRN);
    if(c1 && c2)
        {
            new_subframe.clear();
            DLOG(INFO) << "c1 and c2 " << c1 << " " << c2;
        }
    else
        {
            DLOG(INFO) << "c1 and c2 " << c1 << " " << c2;
        }
        
}
