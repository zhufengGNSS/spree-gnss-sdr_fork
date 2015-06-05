/*!
 * \file gps_l1_ca_sd_pvt_cc.cc
 * \brief Implementation of a Position Velocity and Time computation block for GPS L1 C/A
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_l1_ca_sd_pvt_cc.h"
#include <algorithm>
#include <bitset>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "control_message_factory.h"
#include "gnss_synchro.h"
#include "concurrent_map.h"
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"
#include "spoofing_message.h"

using google::LogMessage;

extern concurrent_map<Gps_Ephemeris> global_gps_ephemeris_map;
extern concurrent_map<Gps_Iono> global_gps_iono_map;
extern concurrent_map<Gps_Utc_Model> global_gps_utc_model_map;

extern concurrent_queue<Sbas_Raw_Msg> global_sbas_raw_msg_queue;
extern concurrent_map<Sbas_Ionosphere_Correction> global_sbas_iono_map;
extern concurrent_map<Sbas_Satellite_Correction> global_sbas_sat_corr_map;
extern concurrent_map<Sbas_Ephemeris> global_sbas_ephemeris_map;
extern concurrent_queue<Spoofing_Message> global_spoofing_queue;
extern concurrent_map<int> global_channel_status;
extern concurrent_map<std::map<unsigned int, unsigned int>> global_subframe_check;
extern concurrent_map<Subframe> global_subframe_map;

struct GPS_time_t{
    int week;
    double TOW;
    int subframe_id;
};

extern concurrent_map<GPS_time_t> global_gps_time;

gps_l1_ca_sd_pvt_cc_sptr
gps_l1_ca_make_sd_pvt_cc(unsigned int nchannels, boost::shared_ptr<gr::msg_queue> queue, bool dump, std::string dump_filename, int averaging_depth, bool flag_averaging, int output_rate_ms, int display_rate_ms, bool flag_nmea_tty_port, std::string nmea_dump_filename, std::string nmea_dump_devname, Spoofing_Detector spoofing_detector, std::string flog_filename)
{
    return gps_l1_ca_sd_pvt_cc_sptr(new gps_l1_ca_sd_pvt_cc(nchannels, queue, dump, dump_filename, averaging_depth, flag_averaging, output_rate_ms, display_rate_ms, flag_nmea_tty_port, nmea_dump_filename, nmea_dump_devname, spoofing_detector, flog_filename));
}


gps_l1_ca_sd_pvt_cc::gps_l1_ca_sd_pvt_cc(unsigned int nchannels,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump, std::string dump_filename,
        int averaging_depth,
        bool flag_averaging,
        int output_rate_ms,
        int display_rate_ms,
        bool flag_nmea_tty_port,
        std::string nmea_dump_filename,
        std::string nmea_dump_devname,
        Spoofing_Detector spoofing_detector,
        std::string flog_filename) :
             gr::block("gps_l1_ca_sd_pvt_cc", gr::io_signature::make(nchannels, nchannels,  sizeof(Gnss_Synchro)),
             gr::io_signature::make(1, 1, sizeof(gr_complex)) )
{
    d_output_rate_ms = output_rate_ms;
    d_display_rate_ms = display_rate_ms;
    d_queue = queue;
    d_dump = dump;
    d_nchannels = nchannels;
    d_dump_filename = dump_filename;
    std::string dump_ls_pvt_filename = dump_filename;

    //initialize kml_printer
    std::string kml_dump_filename;
    kml_dump_filename = d_dump_filename;
    kml_dump_filename.append(".kml");
    d_kml_dump = std::make_shared<Kml_Printer>();
    d_kml_dump->set_headers(kml_dump_filename);

    //initialize nmea_printer
    d_nmea_printer = std::make_shared<Nmea_Printer>(nmea_dump_filename, flag_nmea_tty_port, nmea_dump_devname);


    d_dump_filename.append("_raw.dat");
    dump_ls_pvt_filename.append("_ls_pvt.dat");
    d_averaging_depth = averaging_depth;
    d_flag_averaging = flag_averaging;

    d_ls_pvt = new gps_l1_ca_ls_pvt(nchannels,dump_ls_pvt_filename,d_dump);
    d_ls_pvt->set_averaging_depth(d_averaging_depth);

    d_sample_counter = 0;
    d_last_sample_nav_output = 0;
    d_rx_time = 0.0;

    //spoofing
    d_spoofing_detector = spoofing_detector;
    d_detect_spoofing = spoofing_detector.d_ap_detection;
    d_cno_detection = d_spoofing_detector.d_cno_detection;
    d_alt_detection = d_spoofing_detector.d_alt_detection;
    d_satpos_detection = d_spoofing_detector.d_satpos_detection;

    //flog
    d_flog_filename = flog_filename;

    b_rinex_header_writen = false;
    b_rinex_sbs_header_writen = false;
    rp = new Rinex_Printer();

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "PVT dump enabled Log file: " << d_dump_filename.c_str();
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(INFO) << "Exception opening PVT dump file " << e.what();
                    }
                }


            std::string d_dump_snr_filename = "../data/stdev";
            if (d_dump_snr_file.is_open() == false)
                {
                    try
                    {
                            d_dump_snr_filename.append(".dat");
                            d_dump_snr_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_snr_file.open(d_dump_snr_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "SNR stdDevdump enabled, " << " Log file: " << d_dump_snr_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << " Exception opening SNR stdDev dump file " << e.what() << std::endl;
                    }
                }
            }

    //create all logging files
    for(int i = 1; i != 33; i++)
        {
            std::string tmp = d_flog_filename;
            tmp.append(boost::lexical_cast<std::string>(i));
            tmp.append(".dat");
            std::ofstream *flog_file = new std::ofstream(tmp.c_str(), std::ios::out | std::ios::binary);
            flog_file->exceptions (std::ifstream::failbit | std::ifstream::badbit );
            flog_files_map[i] = flog_file;
        }

}



gps_l1_ca_sd_pvt_cc::~gps_l1_ca_sd_pvt_cc()
{
    d_dump_file.close();
    d_dump_snr_file.close();
}

int gps_l1_ca_sd_pvt_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,	gr_vector_void_star &output_items)
{
    d_sample_counter++;

    std::map<int, Gnss_Synchro> gnss_pseudoranges_map;
    std::map<int, int> channel_status = global_channel_status.get_map_copy(); 

    Gnss_Synchro **in = (Gnss_Synchro **)  &input_items[0]; //Get the input pointer

    std::list<unsigned int> channels; 
    std::list<unsigned int> channels_used; 
    std::map<unsigned int, unsigned int> channel_added;
    for(unsigned int i = 0; i<d_nchannels; ++i)
        {
            if (in[i][0].Flag_valid_pseudorange)
                {
                    if(channel_status.count(i) && channel_status.at(i) != 2)
                        {
                            channels.push_back(i);

                            //use the channel that is tracking the  highest peak
                            if(channel_added.count(in[i][0].PRN))
                                {
                                    if(channel_added.at(in[i][0].PRN) != 1 && in[i][0].peak < channel_added.at(in[i][0].PRN))
                                        {
                                            channels_used.remove(channel_added.at(in[i][0].PRN));
                                            channels_used.push_back(i);
                                            channel_added[in[i][0].PRN] = i; 
                                        }
                                }
                            else
                                {
                                    channels_used.push_back(i);
                                    channel_added[in[i][0].PRN] = i; 
                                }
                        }
                }
        }

   
    if((d_sample_counter % d_output_rate_ms) == 0)
        {
            //double corr  = d_spoofing_detector.get_SNR_corr(channels_used, in, d_sample_counter);
        }

    if(d_cno_detection) 
        {
            double stdev = d_spoofing_detector.check_SNR(channels_used, in, d_sample_counter);
             //log the standard deviation
            //d_dump_snr_file.write((char*)&d_sample_counter, sizeof(unsigned long int));
            //d_dump_snr_file.write((char*)&stdev, sizeof(double));
        }

    //only sample every 1000 sample
    if((d_sample_counter % 1000) == 0)
        {
            d_spoofing_detector.SNR_delta_RT_calc(channels_used, in, d_sample_counter);
        }

    unsigned int i = 0;
    std::map<unsigned int, unsigned int> PRN_to_uid;
    std::ofstream *flog_file;
    for(std::list<unsigned int>::iterator it = channels_used.begin(); it != channels_used.end(); ++it)
        {
            i = *it; 
            std::string tmp = std::to_string(in[i][0].PRN);
            tmp += "0"+std::to_string(in[i][0].peak)+"0"+std::to_string(i);
            int unique_id = std::stoi(tmp);
            PRN_to_uid[in[i][0].PRN] = unique_id; 

            if(d_detect_spoofing)
                {
                    gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(unique_id, in[i][0])); // store valid pseudoranges in a map
                }
            else
                {
                    gnss_pseudoranges_map.insert(std::pair<int,Gnss_Synchro>(in[i][0].PRN, in[i][0])); // store valid pseudoranges in a map
                }
            d_rx_time = in[i][0].d_TOW_at_current_symbol; // all the channels have the same RX timestamp (common RX time pseudoranges)

            if( flog_files_map.count(in[i][0].PRN))
                {
                    flog_file = flog_files_map.at(in[i][0].PRN); 
                    if (flog_file->is_open())
                        {
                            flog_file->write((char*)&in[i][0].PRN, sizeof(float));
                            flog_file->write((char*)&d_sample_counter, sizeof(unsigned long int));
                            flog_file->write((char*)&in[i][0].Tracking_timestamp_secs, sizeof(double));
                            flog_file->write((char*)&in[i][0].CN0_dB_hz, sizeof(double));
                            flog_file->write((char*)&in[i][0].Carrier_Doppler_hz, sizeof(double));
                            flog_file->write((char*)&in[i][0].delta, sizeof(float));
                            flog_file->write((char*)&in[i][0].RT, sizeof(float));
                            flog_file->write((char*)&in[i][0].Extra_RT, sizeof(float));
                            flog_file->write((char*)&in[i][0].ELP, sizeof(float));
                            flog_file->write((char*)&in[i][0].MD, sizeof(float));

                        }
                    else
                        {
                            flog_file->open(d_flog_filename.c_str(), std::ios::out | std::ios::binary);
                            flog_file->write((char*)&in[i][0].PRN, sizeof(float));
                            flog_file->write((char*)&d_sample_counter, sizeof(unsigned long int));
                            flog_file->write((char*)&in[i][0].Tracking_timestamp_secs, sizeof(double));
                            flog_file->write((char*)&in[i][0].CN0_dB_hz, sizeof(double));
                            flog_file->write((char*)&in[i][0].Carrier_Doppler_hz, sizeof(double));
                            flog_file->write((char*)&in[i][0].delta, sizeof(float));
                            flog_file->write((char*)&in[i][0].RT, sizeof(float));
                            flog_file->write((char*)&in[i][0].Extra_RT, sizeof(float));
                            flog_file->write((char*)&in[i][0].ELP, sizeof(float));
                            flog_file->write((char*)&in[i][0].MD, sizeof(float));
                        }
                }
        }


    bool spoofed = false;
    
    Spoofing_Message spm;
    spoofed = global_spoofing_queue.try_pop(spm);
    if(spoofed)
        {
           while( !global_spoofing_queue.empty())
                global_spoofing_queue.try_pop(spm);
        }

    //cancel tracking on auxiliary channels if no spoofing has been detected.
    if(d_detect_spoofing && !spoofed) 
        {
            ControlMessageFactory* cmf = new ControlMessageFactory();
            unsigned int i = 0;
            for(std::list<unsigned int>::iterator it = channels.begin(); it != channels.end(); ++it)
                {
                    i = *it; 
                    if (in[i][0].Flag_valid_pseudorange && (channel_status.count(i) && channel_status.at(i) == 1))
                        {
                            if (std::find(channels_used.begin(), channels_used.end(), i) == channels_used.end())
                                {
                                    if(!PRN_to_uid.count(in[i][0].PRN))
                                        continue;

                                    std::string tmp = std::to_string(in[i][0].PRN);
                                    tmp += "0"+std::to_string(in[i][0].peak)+"0"+std::to_string(i);
                                    int uid = std::stoi(tmp);
                                    int uid_u = PRN_to_uid.at(in[i][0].PRN); //uid of channel with the same sat that is used in the PVT
                                     
                                    
                                    if(!d_spoofing_detector.checked_subframes(uid, uid_u))
                                        {
                                            continue;
                                        }
                                    global_subframe_map.remove(uid);
                                    global_gps_time.remove(uid);
                                    global_subframe_check.remove(uid);
                                    global_channel_status.add(i, 2);
                                    DLOG(INFO) << "send no spoofing to flowgraph " << in[i][0].PRN << " ch: " << i;
                                    std::cout << "send no spoofing to flowgraph " << in[i][0].PRN << " ch: " << i  << " " << uid << std::endl;
                                    if (d_queue != gr::msg_queue::sptr())
                                        {
                                            d_queue->handle(cmf->GetQueueMessage(i, 4));
                                        }
                                }
                        }
                }
            delete cmf;
        }


    // ############ 1. READ EPHEMERIS/UTC_MODE/IONO FROM GLOBAL MAPS ####

    d_ls_pvt->gps_ephemeris_map = global_gps_ephemeris_map.get_map_copy();

    if (global_gps_utc_model_map.size() > 0)
        {
            // UTC MODEL data is shared for all the GPS satellites. Read always at ID=0
            global_gps_utc_model_map.read(0, d_ls_pvt->gps_utc_model);
        }

    if (global_gps_iono_map.size() > 0)
        {
            // IONO data is shared for all the GPS satellites. Read always at ID=0
            global_gps_iono_map.read(0, d_ls_pvt->gps_iono);
        }

    // update SBAS data collections
    if (global_sbas_iono_map.size() > 0)
        {
            // SBAS ionospheric correction is shared for all the GPS satellites. Read always at ID=0
            global_sbas_iono_map.read(0, d_ls_pvt->sbas_iono);
        }
    d_ls_pvt->sbas_sat_corr_map = global_sbas_sat_corr_map.get_map_copy();
    d_ls_pvt->sbas_ephemeris_map = global_sbas_ephemeris_map.get_map_copy();

    // read SBAS raw messages directly from queue and write them into rinex file
    Sbas_Raw_Msg sbas_raw_msg;
    while (global_sbas_raw_msg_queue.try_pop(sbas_raw_msg))
        {
            // create the header of not yet done
            if(!b_rinex_sbs_header_writen)
                {
                    rp->rinex_sbs_header(rp->sbsFile);
                    b_rinex_sbs_header_writen = true;
                }

            // Define the RX time of the SBAS message by using the GPS time.
            // It has only an effect if there has not been yet a SBAS MT12 available
            // when the message was received.
            if(sbas_raw_msg.get_rx_time_obj().is_related() == false
                    && gnss_pseudoranges_map.size() > 0
                    && d_ls_pvt->gps_ephemeris_map.size() > 0)
                {
                    // doesn't matter which channel/satellite we choose
                    Gnss_Synchro gs = gnss_pseudoranges_map.begin()->second;
                    Gps_Ephemeris eph = d_ls_pvt->gps_ephemeris_map.begin()->second;

                    double relative_rx_time = gs.Tracking_timestamp_secs;
                    int gps_week = eph.i_GPS_week;
                    double gps_sec = gs.d_TOW_at_current_symbol;

                    Sbas_Time_Relation time_rel(relative_rx_time, gps_week, gps_sec);
                    sbas_raw_msg.relate(time_rel);
                }

            // send the message to the rinex logger if it has a valid GPS time stamp
            if(sbas_raw_msg.get_rx_time_obj().is_related())
                {
                    rp->log_rinex_sbs(rp->sbsFile, sbas_raw_msg);
                }
        }

     
    // ############ 2 COMPUTE THE PVT ################################
    if (gnss_pseudoranges_map.size() > 0 and d_ls_pvt->gps_ephemeris_map.size() >0 and 0)
        {
            
            // compute on the fly PVT solution
            //mod 8/4/2012 Set the PVT computation rate in this block
            if ((d_sample_counter % d_output_rate_ms) == 0)
                {
                    bool pvt_result;
                    pvt_result = d_ls_pvt->get_PVT(gnss_pseudoranges_map, d_rx_time, d_flag_averaging);
                    DLOG(INFO) << "pvt result";
                    if (pvt_result == true)
                        {
                            //d_kml_dump->print_position(d_ls_pvt, d_flag_averaging);
                            //d_nmea_printer->Print_Nmea_Line(d_ls_pvt, d_flag_averaging);

                            if (!b_rinex_header_writen) //  & we have utc data in nav message!
                                {
                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                        {
                                            rp->rinex_obs_header(rp->obsFile, gps_ephemeris_iter->second,d_rx_time);
                                            rp->rinex_nav_header(rp->navFile, d_ls_pvt->gps_iono, d_ls_pvt->gps_utc_model);
                                            b_rinex_header_writen = true; // do not write header anymore
                                        }
                                }
                            if(b_rinex_header_writen) // Put here another condition to separate annotations (e.g 30 s)
                                {
                                    // Limit the RINEX navigation output rate to 1/6 seg
                                    // Notice that d_sample_counter period is 1ms (for GPS correlators)
                                    if ((d_sample_counter - d_last_sample_nav_output) >= 6000)
                                        {
                                            rp->log_rinex_nav(rp->navFile, d_ls_pvt->gps_ephemeris_map);
                                            d_last_sample_nav_output = d_sample_counter;
                                        }
                                    std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter;
                                    gps_ephemeris_iter = d_ls_pvt->gps_ephemeris_map.begin();
                                    if (gps_ephemeris_iter != d_ls_pvt->gps_ephemeris_map.end())
                                        {
                                            rp->log_rinex_obs(rp->obsFile, gps_ephemeris_iter->second, d_rx_time, gnss_pseudoranges_map);
                                        }
                                }
                        }

                //Check if the value of the position is logical and if the satellites have movement is probable.
                if(d_alt_detection)
                    {
                        if(d_ls_pvt->b_valid_position == true)
                        {
                            d_spoofing_detector.check_position(d_ls_pvt->d_latitude_d, d_ls_pvt->d_longitude_d, d_ls_pvt->d_height_m); 
                        }
                    }

                if(d_satpos_detection)
                    {
                        std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter2;
                        std::map<int,Gnss_Synchro>::iterator gnss_pseudoranges_iter;
                        for(gnss_pseudoranges_iter = gnss_pseudoranges_map.begin();
                                gnss_pseudoranges_iter != gnss_pseudoranges_map.end();
                                gnss_pseudoranges_iter++)
                        {
                            std::map<int,Gps_Ephemeris>::iterator gps_ephemeris_iter2;
                            gps_ephemeris_iter2 = d_ls_pvt->gps_ephemeris_map.find(gnss_pseudoranges_iter->first);


                            if (gps_ephemeris_iter2 != d_ls_pvt->gps_ephemeris_map.end())
                            {
                                d_spoofing_detector.check_satpos(gps_ephemeris_iter2->second.i_satellite_PRN, gps_ephemeris_iter2->second.timestamp , 
                                        gps_ephemeris_iter2->second.d_satpos_X, gps_ephemeris_iter2->second.d_satpos_Y, 
                                        gps_ephemeris_iter2->second.d_satpos_Z);
                            }
                        }
                    }
                }



            // DEBUG MESSAGE: Display position in console output
            if (((d_sample_counter % d_display_rate_ms) == 0) and d_ls_pvt->b_valid_position == true)
                {
                    std::cout << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]" << std::endl;

                    LOG(INFO) << "Position at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " is Lat = " << d_ls_pvt->d_latitude_d << " [deg], Long = " << d_ls_pvt->d_longitude_d
                              << " [deg], Height= " << d_ls_pvt->d_height_m << " [m]";

                    LOG(INFO) << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->d_position_UTC_time)
                              << " is HDOP = " << d_ls_pvt->d_HDOP << " VDOP = "
                              << d_ls_pvt->d_VDOP <<" TDOP = " << d_ls_pvt->d_TDOP << " GDOP = " << d_ls_pvt->d_GDOP;
                }
            // MULTIPLEXED FILE RECORDING - Record results to file
            if(d_dump == true)
                {
                    try
                    {
                            double tmp_double;
                            for (unsigned int i = 0; i < d_nchannels ; i++)
                                {
                                    tmp_double = in[i][0].Pseudorange_m;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    tmp_double = 0;
                                    d_dump_file.write((char*)&tmp_double, sizeof(double));
                                    d_dump_file.write((char*)&d_rx_time, sizeof(double));
                                }
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                    }
                }
        }

//    }

    consume_each(1); //one by one
    return 0;
}


