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

}

Spoofing_Detector::~Spoofing_Detector()
{

}
bool Spoofing_Detector::is_spoofed()
{
    return spoofed;
}

void Spoofing_Detector::spoofing_detected(std::string description, int spoofing_case)
{
    DLOG(INFO) << "SPOOFING DETECTED " << description;
    spoofed = true;
    Spoofing_Message msg;
    msg.spoofing_case = spoofing_case;
    msg.description = description;
    global_spoofing_queue.push(msg);
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
        DLOG(INFO) << "check new TOW " << duration << " " << std::abs(new_time-old_time);

        //TODO: set configurable max deviation, now is 1 second
        if( std::abs(std::abs(new_time-old_time)-duration) > 0 && (round(current_time_ms) != 0 || round(old_GPS_time.at(2)) != 0) )
            {
                Spoofing_Message msg;
                msg.spoofing_case = 3; 

                string description = "New ephemeris TOW is not consistent with last received TOW and the time elapsed  from its reception";
                msg.description = description;
                spoofed = true;
                    
                if(old_time < new_time)
                    {
                        stringstream s;
                        s << " received new ephemeris TOW that is later than last received one and incorrect";
                        s << " difference: " << new_time-old_time;
                        s << " duration: " << duration;
                        s << " gps times : " << new_time << " " << old_time; 
                        s << " times : " << round(current_time_ms) << " " << round(old_GPS_time.at(2)); 
                        spoofing_detected(s.str(), 3);
                        
                        std::cout << "SPOOFING DETECTED" 
                        << " received new ephemeris TOW that is later than last received one and incorrect"
                        << " difference: " << new_time-old_time
                        << " duration: " << duration
                        << " gps times : " << new_time << " " << old_time 
                        << " times : " << round(current_time_ms) << " " << round(old_GPS_time.at(2)) << std::endl; 
                    }
                else
                    {
                        stringstream s;
                        s << " received new ephemeris TOW that is earlier than last received one and incorrect";
                        s << " difference: " << new_time-old_time;
                        s << " duration: " << duration;
                        s << " gps times : " << new_time << " " << old_time;
                        s << " times : " << round(current_time_ms) << " " << round(old_GPS_time.at(2)); 
                        spoofing_detected(s.str(), 3);

                        std::cout << "SPOOFING DETECTED" 
                        << " received new ephemeris TOW that is earlier than last received one and incorrect"
                        << " difference: " << new_time-old_time
                        << " duration: " << duration
                        << " gps times : " << new_time << " " << old_time 
                        << " times : " << round(current_time_ms) << " " << round(old_GPS_time.at(2)) << std::endl; 
                
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
        std::cout  << "SPOOFING middle of the earth attack" << std::endl;
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
            spoofed = true;
        }
}

void Spoofing_Detector::check_RX(std::set<int> satellites, std::map<int, std::map<string, rx_time_t>> rx_times, double max_discrepancy)
{
    // If the received time for any of the signals of the same satellite is larger
    // than the maximum allowed discrepancy assume one is a spoofed signal
    double largest, smallest;
    int subframe1, subframe2, diff;
    bool spoofed_rx = false;
    for(std::set<int>::iterator it = satellites.begin(); it != satellites.end(); ++it)
        {
            spoofed_rx = false;
            if(rx_times.count(*it))
                {
                    checked_rx[*it] = true;
                    largest =  rx_times.at(*it).at("largest").time;
                    smallest =  rx_times.at(*it).at("smallest").time;
                    DLOG(INFO) << "check rx for "<< *it << "times "<< largest << " " << smallest ;
                    if(std::abs(largest-smallest) > max_discrepancy)
                        {
                            subframe1 = rx_times.at(*it).at("smallest").subframe;
                            subframe2 = rx_times.at(*it).at("largest").subframe;
                            DLOG(INFO) << "subframes " << subframe1 << " " << subframe2; 
                            if(subframe2 != subframe1)
                                {
                                    DLOG(INFO) << "subframes " << subframe1 << " " << subframe2; 
                                    diff = subframe2-subframe1;
                                    DLOG(INFO) << "diff " << diff;
                                    if(diff < 0 && diff != -4)
                                        spoofed_rx = true;
                                    else if(diff > 1)
                                        spoofed_rx = true;
                                    else if(std::abs(largest-smallest) > 6000) 
                                        spoofed_rx = true;
                                    else
                                        checked_rx[*it] = false;
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

                     //   std::cout << "SPOOFING DETECTED for sat: " << *it
                     //       << " RX times not consistent " << smallest << " "<< largest << std::endl;
                    }
        }
}

void Spoofing_Detector::check_GPS_time(std::set<int> unique_ids)
{
    std::map<int, GPS_time_t> gps_times = global_gps_time.get_map_copy();
    set<int> GPS_TOW;
    int GPS_week, TOW;
    set<int> subframe_IDs;
    double smallest = 0; 
    double largest = 0;
    GPS_time_t gps_time;
    //check that the GPS week is consistent between all satellites
    for(std::set<int>::iterator it = unique_ids.begin(); it != unique_ids.end(); ++it)
        {
            if(gps_times.count(*it))
            {
                gps_time = gps_times.at(*it);
                GPS_week = gps_time.week; 
                if(GPS_week == 0)
                    continue;
                if(gps_time.timestamp > largest)
                    largest = gps_time.timestamp;
                if(gps_time.timestamp < smallest)
                    smallest  = gps_time.timestamp;

                DLOG(INFO) << "ts " << gps_time.timestamp;
                TOW = GPS_week*604800+gps_time.TOW;
                GPS_TOW.insert(TOW);
                subframe_IDs.insert(gps_time.subframe_id);
            }

        }

    if(subframe_IDs.size() > 1 || abs(largest-smallest) > 30000)
    {
        DLOG(INFO) << "Not all satellites have received the latest subframe, don't compare GPS time " << subframe_IDs.size() << " " <<abs(largest-smallest);
    }
    else if(GPS_TOW.size() >1)
    {
        std::cout << "SPOOFING DETECTED satellites GPS TOW are not synced" << std::endl;
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

void Spoofing_Detector::check_subframes(std::map<int, std::list<int>> ids_of_PRN)
{
    int id1, id2;
    Subframe subframeA, subframeB;
    std::map<int, Subframe> subframes = global_channel_to_subframe.get_map_copy();
    for (std::map<int,list<int>>::iterator it=ids_of_PRN.begin(); it!=ids_of_PRN.end(); ++it)
    {
        std::list<int> ids = it->second;  
        DLOG(INFO) << "check subframes for sat " << it->first << " " <<ids.size(); 
        while(!ids.empty())
            {
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
                DLOG(INFO) << "Check subframes for " << id1 << " " << id2;
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
                        DLOG(INFO) << "Subframe are not updated"
                                   << " " << subframeA.id
                                   << " " << subframeB.id
                                   << " " << subframeA.timestamp
                                   << " " << subframeB.timestamp;
                        continue;
                    }
                
                if(subframeA.subframe != subframeB.subframe && subframeA.subframe != "" && subframeB.subframe != "")
                    {

                        std::cout << "SPOOFING DETECTED ephemeris data not consistent "<< it->first << " " << id1 << " " << id2 << std::endl;
                        std::cout << subframeA.timestamp << " " << subframeB.timestamp << std::endl;
                        std::cout << subframeA.id << " " << subframeB.id << std::endl;
                        std::cout << subframeA.subframe << std::endl << subframeB.subframe << std::endl;

                        stringstream s;
                        s << "Ephemeris data not consistent" << id1 << " " << id2;
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
}

bool Spoofing_Detector::checked(unsigned int PRN, unsigned int channel)
{
    if(!used_channel.count(PRN))
        {
            DLOG(INFO) << "PRN is not in used channel";
            return false; 
        }

    unsigned int u_channel = used_channel.at(PRN);
    unsigned int id1 = channel; 
    unsigned int id2 = used_channel.at(PRN);
    DLOG(INFO) << "checked ? " << channel << " " << u_channel;
    if(!checked_rx.count(PRN))
        {
            DLOG(INFO) << "The satellite has not been checked yet " << u_channel;
            return false;
        }
    else
        {
            //there was no comparison of the RX times for satellite PRN
            if(!checked_rx.at(PRN))
                {
                    DLOG(INFO) << "No RX checked has been performed";
                    return false;
                }

            //Either the main channel or the auxiliary channel was not checked during the RX check
            if(checked_channel_rx.count(u_channel) && checked_channel_rx.count(channel)) 
                {
                    if(!(checked_channel_rx.at(u_channel) && checked_channel_rx.at(channel))) 
                        {
                            DLOG(INFO) << "No RX checked has been performed on the channels 1";
                            return false;
                        }
                }
            else
                {   
                    DLOG(INFO) << "No RX checked has been performed on the channels 2";
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
            DLOG(INFO) << "subframes not checked ";
            return false;
        }
    else
        {
            if(!(checked_subframe.at(id).count(1) && checked_subframe.at(id).at(1)))
                {
                    DLOG(INFO) << "subframe 1 not checked for channel " <<channel;
                    return false;
                }
            if(!(checked_subframe.at(id).count(2) && checked_subframe.at(id).at(2)))
                {
                    DLOG(INFO) << "subframe 2 not checked for channel " <<channel;
                    return false;
                }
            if(!(checked_subframe.at(id).count(3) && checked_subframe.at(id).at(3)))
                {
                    DLOG(INFO) << "subframe 3 not checked for channel " <<channel;
                    return false;
                }
        }
    //Since the
    checked_subframe.erase(id);
    return true;
}


std::list<unsigned int> Spoofing_Detector::RX_TX_ephemeris_check(std::list<unsigned int> channels, 
                                                                Gnss_Synchro **in, 
                                                                std::map<int, 
                                                                Gps_Ephemeris> ephemeris_map, 
                                                                double max_discrepancy)
{
    used_channel.clear();
    std::map<int, std::map<string, rx_time_t >> rx_times;
    std::map<int, unsigned int> peak;
    std::set<int> unique_ids;
    std::set<int> satellites;
    std::map<int, std::list<int>> ids_of_PRN;
    std::map<int, int> satId_to_channel;  //unique ids of auxiliary acqusitions to  channel
    set<double> tx_times;
    double rx_time;
    unsigned int PRN;
    //DLOG(INFO) << "spoofing detection1";
    unsigned int i = 0;
    for(std::list<unsigned int>::iterator it = channels.begin(); it != channels.end(); ++it)
        {
            i = *it;
            if (in[i][0].Flag_valid_pseudorange == true)
                {
                    PRN = in[i][0].PRN;
                    std::string tmp = std::to_string(PRN);
                    satellites.insert(PRN);
                    tmp += "0"+std::to_string(in[i][0].peak)+"0"+std::to_string(i);
                    int sat_id = std::stoi(tmp);
                    unique_ids.insert(sat_id);

                    satId_to_channel[sat_id] = i;
                    DLOG(INFO) << "spoofing detection2";
                    rx_time = in[i][0].rx_of_subframe;
                    if(rx_time == 0)
                        continue;
        
                    //rx_time = in[i][0].rx_of_subframe;
                    DLOG(INFO) << "sat " << sat_id << " rx: " << rx_time;

                    rx_time_t p;
                    p.sat_id = sat_id;
                    p.time = rx_time;
                    p.subframe = in[i][0].subframe;
                    
                    if(!rx_times.count(PRN))
                        {
                          DLOG(INFO) << "received first rx for sat " << PRN;
                          std::map<string, rx_time_t> tmp;
                          tmp["smallest"] = p; 
                          tmp["largest"] = p; 
                          rx_times[PRN] = tmp;

                          peak[PRN] = in[i][0].peak; 
                        }
                    else
                        {
                            std::map<string, rx_time_t> rx = rx_times.at(PRN);
                            if(rx.at("smallest").time > rx_time)// && 
                            //    !(rx.at("smallest").second == rx_time && in[i][0].peak> peak.at(PRN))) 
                                {
                                    rx_times.at(PRN)["smallest"] = p;
                                    peak.at(PRN) = in[i][0].peak;
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
                    DLOG(INFO) << "checked channel rx " << sat_id;
                    checked_channel_rx[sat_id] = true;
                }

            //DLOG(INFO) << "sat " << PRN << " smallest " << rx_times.at(PRN).at("smallest").second << " largest " <<rx_times.at(PRN).at("largest").second;
        }

    check_RX(satellites, rx_times, max_discrepancy);
    //check_sat_RX(satellites, rx_times);
    //check_GPS_time(satellites, ephemeris_map);
    check_GPS_time(unique_ids);
    check_subframes(ids_of_PRN);
   
    //return the channles of those signals with the smallest RX time
    int id;
    std::list<unsigned int> first_arriving_channels;
    for(std::set<int>::iterator it = satellites.begin(); it != satellites.end(); ++it) 
        {
           if(!rx_times.count(*it))
                continue;
           id = rx_times.at(*it).at("smallest").sat_id; 
           DLOG(INFO) << "smallest: " << rx_times.at(*it).at("smallest").sat_id; 
           first_arriving_channels.push_back(satId_to_channel.at(id));
           used_channel[*it] = id; 
        }

    return channels;
}
