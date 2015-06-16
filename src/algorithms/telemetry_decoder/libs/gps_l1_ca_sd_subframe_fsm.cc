/*!
 * \file gps_l1_ca_sd_subframe_fsm.cc
 * \brief  Implementation of a GPS NAV message word-to-subframe decoder state machine
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_l1_ca_sd_subframe_fsm.h"
#include <list>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
extern concurrent_map<bool> global_channel_status;

extern concurrent_map<Subframe> global_subframe_map;

struct RX_time{
    unsigned int subframe_id;
    unsigned int PRN;
    double timestamp;
};
extern concurrent_map<RX_time> global_RX_map;

struct GPS_time_t{
    int week;
    double TOW;
    double timestamp;
    int subframe_id;
};

extern concurrent_map<GPS_time_t> global_gps_time;


//************ GPS WORD TO SUBFRAME DECODER STATE MACHINE **********

struct Ev_gps_word_valid : sc::event<Ev_gps_word_valid> {};
struct Ev_gps_word_invalid : sc::event<Ev_gps_word_invalid>{};
struct Ev_gps_word_preamble : sc::event<Ev_gps_word_preamble>{};


struct gps_sd_subframe_fsm_S0: public sc::state<gps_sd_subframe_fsm_S0, GpsL1CaSdSubframeFsm>
{
public:
    // sc::transition(event,next_status)
    typedef sc::transition< Ev_gps_word_preamble, gps_sd_subframe_fsm_S1 > reactions;
    gps_sd_subframe_fsm_S0(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S0 "<<std::endl;
    }
};




struct gps_sd_subframe_fsm_S1: public sc::state<gps_sd_subframe_fsm_S1, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S2 > > reactions;

    gps_sd_subframe_fsm_S1(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S1 "<<std::endl;
    }
};




struct gps_sd_subframe_fsm_S2: public sc::state<gps_sd_subframe_fsm_S2, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S3 > > reactions;

    gps_sd_subframe_fsm_S2(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S2 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(0);
    }
};




struct gps_sd_subframe_fsm_S3: public sc::state<gps_sd_subframe_fsm_S3, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S4 > > reactions;

    gps_sd_subframe_fsm_S3(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S3 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(1);
    }
};




struct gps_sd_subframe_fsm_S4: public sc::state<gps_sd_subframe_fsm_S4, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S5 > > reactions;

    gps_sd_subframe_fsm_S4(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S4 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(2);
    }
};




struct gps_sd_subframe_fsm_S5: public sc::state<gps_sd_subframe_fsm_S5, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S6 > > reactions;

    gps_sd_subframe_fsm_S5(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S5 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(3);
    }
};





struct gps_sd_subframe_fsm_S6: public sc::state<gps_sd_subframe_fsm_S6, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S7 > > reactions;

    gps_sd_subframe_fsm_S6(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S6 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(4);
    }
};



struct gps_sd_subframe_fsm_S7: public sc::state<gps_sd_subframe_fsm_S7, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S8 > > reactions;

    gps_sd_subframe_fsm_S7(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S7 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(5);
    }
};



struct gps_sd_subframe_fsm_S8: public sc::state<gps_sd_subframe_fsm_S8, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S9 > > reactions;

    gps_sd_subframe_fsm_S8(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S8 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(6);
    }
};




struct gps_sd_subframe_fsm_S9: public sc::state<gps_sd_subframe_fsm_S9, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S10 > > reactions;

    gps_sd_subframe_fsm_S9(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S9 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(7);
    }
};



struct gps_sd_subframe_fsm_S10: public sc::state<gps_sd_subframe_fsm_S10, GpsL1CaSdSubframeFsm>
{
public:
    typedef mpl::list<sc::transition< Ev_gps_word_invalid, gps_sd_subframe_fsm_S0 >,
            sc::transition< Ev_gps_word_valid, gps_sd_subframe_fsm_S11 > > reactions;

    gps_sd_subframe_fsm_S10(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Enter S10 "<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(8);
    }
};



struct gps_sd_subframe_fsm_S11: public sc::state<gps_sd_subframe_fsm_S11, GpsL1CaSdSubframeFsm>
{
public:
    typedef sc::transition< Ev_gps_word_preamble, gps_sd_subframe_fsm_S1 > reactions;

    gps_sd_subframe_fsm_S11(my_context ctx): my_base( ctx )
    {
        //std::cout<<"Completed GPS Subframe!"<<std::endl;
        context< GpsL1CaSdSubframeFsm >().gps_word_to_subframe(9);
        context< GpsL1CaSdSubframeFsm >().gps_subframe_to_nav_msg(); //decode the subframe
        // DECODE SUBFRAME
        //std::cout<<"Enter S11"<<std::endl;
    }
};




GpsL1CaSdSubframeFsm::GpsL1CaSdSubframeFsm()
{
    d_nav.reset();
    initiate(); //start the FSM
}



void GpsL1CaSdSubframeFsm::gps_word_to_subframe(int position)
{
    // insert the word in the correct position of the subframe
    std::memcpy(&d_subframe[position*GPS_WORD_LENGTH], &d_GPS_frame_4bytes, sizeof(char)*GPS_WORD_LENGTH);
}



void GpsL1CaSdSubframeFsm::gps_subframe_to_nav_msg()
{
    int subframe_ID;
    new_subframe = true;   
    // NEW GPS SUBFRAME HAS ARRIVED!
    subframe_ID = d_nav.subframe_decoder(this->d_subframe); //decode the subframe
   
    std::string tmp = std::to_string(i_satellite_PRN)+ "0" + std::to_string(i_peak)+"0"+std::to_string(i_channel_ID);
    d_nav.unique_id = std::stoi(tmp);
    if( subframe_ID < 1 || subframe_ID > 5)
        return;

    d_nav.i_satellite_PRN = i_satellite_PRN;

    std::cout << "NAV Message: received subframe "
        << subframe_ID << " from satellite "
        << Gnss_Satellite(std::string("GPS"), i_satellite_PRN) 
        << " at time: " << this->d_preamble_time_ms
        << " in channel: " << i_channel_ID 
        << " sat id"  << i_satellite_PRN << "0" << i_peak<< "0" << i_channel_ID<< std::endl << std::endl
        <<  "subframe: " << d_nav.get_subframe(subframe_ID) << std::endl << std::endl;

    global_channel_status.add(d_nav.i_channel_ID, 1); 
    if( inter_satellite_check || ap_detection )
        {
            //Spoofing detection - subframes
            Subframe subframe;
            subframe.timestamp = this->d_preamble_time_ms;
            subframe.subframe_id = subframe_ID; 
            subframe.PRN = i_satellite_PRN; 
            subframe.subframe = d_nav.get_subframe(subframe_ID);

            global_subframe_map.add((int)d_nav.unique_id, subframe);
        }

    if( ap_detection)
        {
            spoofing_detector.check_ap_subframe(d_nav.unique_id, i_satellite_PRN, subframe_ID);
            spoofing_detector.check_RX_time(i_satellite_PRN, subframe_ID);
            d_nav.i_peak = i_peak; 
        }

    GPS_time_t gps_time;
    if( inter_satellite_check )
        {
            //Spoofing detection - GPS time
            std::map<int, GPS_time_t> gps_times = global_gps_time.get_map_copy();
            if(gps_times.count(d_nav.unique_id))
            {
                gps_time = gps_times.at(d_nav.unique_id);
            }
            else
            {
                gps_time.week = 0;
            }

            gps_time.subframe_id = subframe_ID;
            gps_time.timestamp = this->d_preamble_time_ms;
            gps_time.TOW = d_nav.get_TOW();
        }

    d_nav.i_satellite_PRN = i_satellite_PRN;
    d_nav.i_channel_ID = i_channel_ID;

    switch (subframe_ID)
    {
    case 1:
        if( inter_satellite_check )
            {
                gps_time.week = d_nav.get_week();

                // check that new ephemeris TOW is consitent with the latest received TOW
                // and the time duration between them
                spoofing_detector.check_new_TOW(this->d_preamble_time_ms, d_nav.get_week(), d_nav.get_TOW());
            }
        if( external_nav_check )
            {
                spoofing_detector.check_external_gps_time(d_nav.get_week(), d_nav.get_TOW()); 
            }
        break;
    case 2:
        if(detect_spoofing)
        {
            spoofing_detector.check_middle_earth(d_nav.get_sqrtA());
        }
        break;
    case 3: //we have a new set of ephemeris data for the current SV
        if (d_nav.satellite_validation() == true)
            {
                // get ephemeris object for this SV (mandatory)
                Gps_Ephemeris ephemeris = d_nav.get_ephemeris();

                ephemeris.timestamp = this->d_preamble_time_ms;
                d_ephemeris_queue->push(ephemeris);
                if( external_nav_check )
                {
                    spoofing_detector.check_external_ephemeris(ephemeris, i_satellite_PRN); 
                }
/*
                std::map<int, Gps_Almanac> almanac_map = d_nav.get_almanac();
                if( almanac_map.count( i_satellite_PRN) )
                    { 
                        Gps_Almanac almanac = almanac_map.at( i_satellite_PRN);
                        std::cout << "satellite: " <<  i_satellite_PRN << std::endl
                            << "d_Delta_i " <<  almanac.d_Delta_i << std::endl
                            << "d_Toa " <<  almanac.d_Toa << std::endl
                            << "d_M_0 " <<  almanac.d_M_0 << std::endl
                            << "d_e_eccentricity " <<  almanac.d_e_eccentricity << std::endl
                            << "d_sqrt_A " <<  almanac.d_sqrt_A << std::endl
                            << "d_OMEGA0 " <<  almanac.d_OMEGA0 << std::endl
                            << "d_OMEGA " <<  almanac.d_OMEGA << std::endl
                            << "d_OMEGA_DOT " <<  almanac.d_OMEGA_DOT << std::endl
                            << "i_SV_health " <<  almanac.i_SV_health << std::endl
                            << "d_A_f0 " <<  almanac.d_A_f0 << std::endl
                            << "d_A_f1 " <<  almanac.d_A_f1 << std::endl;

                        std::cout << std::endl
                            << "d_M_0 " <<  ephemeris.d_M_0 << std::endl
                            << "d_e_eccentricity " <<  ephemeris.d_e_eccentricity << std::endl
                            << "d_sqrt_A " <<  ephemeris.d_sqrt_A << std::endl
                            << "d_OMEGA0 " <<  ephemeris.d_OMEGA0 << std::endl
                            << "d_OMEGA " <<  ephemeris.d_OMEGA << std::endl
                            << "d_OMEGA_DOT " <<  ephemeris.d_OMEGA_DOT << std::endl
                            << "i_SV_health " <<  ephemeris.i_SV_health << std::endl
                            << "d_A_f0 " <<  ephemeris.d_A_f0 << std::endl
                            << "d_A_f1 " <<  ephemeris.d_A_f1 << std::endl;
                    }
*/
            }
        break;
    case 4: // Possible IONOSPHERE and UTC model update (page 18)
        if (d_nav.flag_iono_valid == true)
            {
                Gps_Iono iono = d_nav.get_iono(); //notice that the read operation will clear the valid flag
                d_iono_queue->push(iono);
                if( external_nav_check )
                {
                    spoofing_detector.check_external_iono(iono); 
                }
            }
        if (d_nav.flag_utc_model_valid == true)
            {
                Gps_Utc_Model utc_model = d_nav.get_utc_model(); //notice that the read operation will clear the valid flag
                d_utc_model_queue->push(utc_model);

                if( external_nav_check )
                {
                    spoofing_detector.check_external_utc(utc_model); 
                }
            }
        if( inter_satellite_check )
            {
                spoofing_detector.check_inter_satellite_subframe(d_nav.unique_id, subframe_ID);
            }

        if( external_nav_check )
            {
                spoofing_detector.check_external_almanac(d_nav.get_almanac()); 
            }
        break;
    case 5:
        // get almanac (if available)
        {
            if( inter_satellite_check )
                {
                    //spoofing_detector.check_inter_satellite_subframe(d_nav.unique_id, subframe_ID);
                }

            if( external_nav_check )
                {
                    spoofing_detector.check_external_almanac(d_nav.get_almanac()); 
                }
        }
        break;
    default:
        break;
    }

    if( inter_satellite_check )
        {
            global_gps_time.add((int)d_nav.unique_id, gps_time);
            spoofing_detector.check_GPS_time();
        }
}



void GpsL1CaSdSubframeFsm::Event_gps_word_valid()
{
    this->process_event(Ev_gps_word_valid());
    //std::cout << "word received " 
    //          << " from satellite "
    //          << Gnss_Satellite(std::string("GPS"), i_satellite_PRN) 
    //          << " at time: " << this->d_preamble_time_ms<< std::endl;
}



void GpsL1CaSdSubframeFsm::Event_gps_word_invalid()
{
    this->process_event(Ev_gps_word_invalid());
}



void GpsL1CaSdSubframeFsm::Event_gps_word_preamble()
{
    this->process_event(Ev_gps_word_preamble());
}

