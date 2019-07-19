/*!
 * \file gps_l1_ca_sd_telemetry_decoder_gs.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gps_l1_ca_sd_telemetry_decoder_gs.h"
#include "gps_ephemeris.h"  // for Gps_Ephemeris
#include "gps_iono.h"       // for Gps_Iono
#include "gps_utc_model.h"  // for Gps_Utc_Model
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cmath>      // for round
#include <cstring>    // for memcpy
#include <exception>  // for exception
#include <iostream>   // for cout
#include <memory>     // for shared_ptr
#include "display.h"


#ifndef _rotl
#define _rotl(X, N) (((X) << (N)) ^ ((X) >> (32 - (N))))  // Used in the parity check algorithm
#endif

Concurrent_Map<Gps_Ephemeris> eph_map;
extern Concurrent_Map<int> tracking_sats;
extern Concurrent_Map<Subframe> global_subframe_map;

gps_l1_ca_sd_telemetry_decoder_gs_sptr
gps_l1_ca_make_sd_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump, Spoofing_Detector spoofing_detector)
{
    return gps_l1_ca_sd_telemetry_decoder_gs_sptr(new gps_l1_ca_sd_telemetry_decoder_gs(satellite, dump, spoofing_detector));
}


gps_l1_ca_sd_telemetry_decoder_gs::gps_l1_ca_sd_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump, Spoofing_Detector spoofing_detector) : gr::block("gps_navigation_gs", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    this->message_port_register_out(pmt::mp("events"));
    d_last_valid_preamble = 0;
    d_sent_tlm_failed_msg = false;


    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Initializing GPS L1 TELEMETRY DECODER";

    d_bits_per_preamble = GPS_CA_PREAMBLE_LENGTH_BITS;
    d_samples_per_preamble = d_bits_per_preamble * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
    d_preamble_period_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
    // set the preamble
    d_required_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
    // preamble bits to sampled symbols
    d_preamble_samples = static_cast<int32_t *>(volk_gnsssdr_malloc(d_samples_per_preamble * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    d_frame_length_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
    d_max_symbols_without_valid_frame = d_required_symbols * 10;  // rise alarm 1 minute without valid tlm
    int32_t n = 0;
    for (int32_t i = 0; i < d_bits_per_preamble; i++)
        {
            if (GPS_CA_PREAMBLE.at(i) == '1')
                {
                    for (uint32_t j = 0; j < GPS_CA_TELEMETRY_SYMBOLS_PER_BIT; j++)
                        {
                            d_preamble_samples[n] = 1;
                            n++;
                        }
                }
            else
                {
                    for (uint32_t j = 0; j < GPS_CA_TELEMETRY_SYMBOLS_PER_BIT; j++)
                        {
                            d_preamble_samples[n] = -1;
                            n++;
                        }
                }
        }
    d_sample_counter = 0ULL;
    d_stat = 0;
    d_preamble_index = 0ULL;

    d_flag_frame_sync = false;

    d_flag_parity = false;
    d_TOW_at_current_symbol_ms = 0;
    d_TOW_at_Preamble_ms = 0;
    d_CRC_error_counter = 0;
    d_flag_preamble = false;
    d_channel = 0;
    flag_TOW_set = false;
    flag_PLL_180_deg_phase_locked = false;
    d_prev_GPS_frame_4bytes = 0;
    d_symbol_history.set_capacity(d_required_symbols);
    d_spoofing_detector = spoofing_detector;
}


gps_l1_ca_sd_telemetry_decoder_gs::~gps_l1_ca_sd_telemetry_decoder_gs()
{
    volk_gnsssdr_free(d_preamble_samples);
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
}


bool gps_l1_ca_sd_telemetry_decoder_gs::gps_word_parityCheck(uint32_t gpsword)
{
    uint32_t d1, d2, d3, d4, d5, d6, d7, t, parity;
    // XOR as many bits in parallel as possible.  The magic constants pick
    //   up bits which are to be XOR'ed together to implement the GPS parity
    //   check algorithm described in IS-GPS-200E.  This avoids lengthy shift-
    //   and-xor loops.
    d1 = gpsword & 0xFBFFBF00;
    d2 = _rotl(gpsword, 1) & 0x07FFBF01;
    d3 = _rotl(gpsword, 2) & 0xFC0F8100;
    d4 = _rotl(gpsword, 3) & 0xF81FFE02;
    d5 = _rotl(gpsword, 4) & 0xFC00000E;
    d6 = _rotl(gpsword, 5) & 0x07F00001;
    d7 = _rotl(gpsword, 6) & 0x00003000;
    t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;
    // Now XOR the 5 6-bit fields together to produce the 6-bit final result.
    parity = t ^ _rotl(t, 6) ^ _rotl(t, 12) ^ _rotl(t, 18) ^ _rotl(t, 24);
    parity = parity & 0x3F;
    if (parity == (gpsword & 0x3F))
        {
            return true;
        }

    return false;
}


void gps_l1_ca_sd_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_nav.reset();
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    d_nav.i_satellite_PRN = d_satellite.get_PRN();
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void gps_l1_ca_sd_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    d_nav.i_channel_ID = channel;
    LOG(WARNING) << "Navigation channel set to " << channel << " PRN: " << d_satellite;
       
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename = "telemetry";
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel
                                      << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
                }
        }
}

Gps_Ephemeris gps_l1_ca_sd_telemetry_decoder_gs::compare_eph(Gps_Ephemeris shrd_eph)
{
    int sv = shrd_eph.i_satellite_PRN;
    Gps_Ephemeris eph;
    bool is_same = true;
    if(eph_map.read(sv, eph))
    {
        if
        ((shrd_eph.d_TOW - eph.d_TOW) % 3 == 0 or eph.d_TOW == shrd_eph.d_TOW)
        // and eph.d_Crs == shrd_eph.d_Crs
        // and eph.d_Delta_n == shrd_eph.d_Delta_n
        // and eph.d_M_0 == shrd_eph.d_M_0
        // and eph.d_Cuc == shrd_eph.d_Cuc
        // and eph.d_e_eccentricity == shrd_eph.d_e_eccentricity
        // and eph.d_Cus == shrd_eph.d_Cus
        // and eph.d_sqrt_A == shrd_eph.d_sqrt_A
        // and eph.d_Toe == shrd_eph.d_Toe
        // and eph.d_Toc == shrd_eph.d_Toc
        // and eph.d_Cic == shrd_eph.d_Cic
        // and eph.d_OMEGA0 == shrd_eph.d_OMEGA0
        // and eph.d_Cis == shrd_eph.d_Cis
        // and eph.d_i_0 == shrd_eph.d_i_0
        // and eph.d_Crc == shrd_eph.d_Crc
        // and eph.d_OMEGA == shrd_eph.d_OMEGA
        // and eph.d_OMEGA_DOT == shrd_eph.d_OMEGA_DOT
        // and eph.d_IDOT == shrd_eph.d_IDOT
        // and eph.i_code_on_L2 == shrd_eph.i_code_on_L2
        // and eph.i_GPS_week == shrd_eph.i_GPS_week
        // and eph.b_L2_P_data_flag == shrd_eph.b_L2_P_data_flag
        // and eph.i_SV_accuracy == shrd_eph.i_SV_accuracy
        // and eph.i_SV_health == shrd_eph.i_SV_health
        // and eph.d_TGD == shrd_eph.d_TGD
        // and eph.d_IODC == shrd_eph.d_IODC
        // and eph.d_IODE_SF2 == shrd_eph.d_IODE_SF2
        // and eph.d_IODE_SF3 == shrd_eph.d_IODE_SF3
        // and eph.i_AODO == shrd_eph.i_AODO
        // and eph.b_fit_interval_flag == shrd_eph.b_fit_interval_flag
        // and eph.d_spare1 == shrd_eph.d_spare1
        // and eph.d_spare2 == shrd_eph.d_spare2
        // and eph.d_A_f0 == shrd_eph.d_A_f0
        // and eph.d_A_f1 == shrd_eph.d_A_f1
        // and eph.d_A_f2 == shrd_eph.d_A_f2
        // and eph.b_integrity_status_flag == shrd_eph.b_integrity_status_flag
        // and eph.b_alert_flag == shrd_eph.b_alert_flag
        // and eph.b_antispoofing_flag == shrd_eph.b_antispoofing_flag
        // and eph.d_satClkDrift == shrd_eph.d_satClkDrift
        // and eph.d_dtr == shrd_eph.d_dtr
        // and eph.d_satpos_X == shrd_eph.d_satpos_X
        // and eph.d_satpos_Y == shrd_eph.d_satpos_Y
        // and eph.d_satpos_Z == shrd_eph.d_satpos_Z
        // and eph.d_satvel_X == shrd_eph.d_satvel_X
        // and eph.d_satvel_Y == shrd_eph.d_satvel_Y
        // and eph.d_satvel_Z == shrd_eph.d_satvel_Z
        {
            is_same = true;
            //std::cout << std::endl << TEXT_BOLD_YELLOW << "Same subframe detected for SV: " << sv << TEXT_RESET << std::endl;
        }   
        else
        {   
            std::cout << std::endl << TEXT_BOLD_RED << "Different subframe detected for SV: " << sv << std::endl
                      << "TOW Offset: " << shrd_eph.d_TOW - eph.d_TOW << TEXT_RESET << std::endl;
            // std::cout << "Subframe 1 SV: " << eph.i_satellite_PRN
            //     << ": " << eph.d_TOW
            //     << ": " << eph.d_Crs
            //     << ": " << eph.d_Delta_n
            //     << ": " << eph.d_M_0
            //     << ": " << eph.d_Cuc
            //     << ": " << eph.d_e_eccentricity
            //     << ": " << eph.d_Cus
            //     << ": " << eph.d_sqrt_A
            //     << ": " << eph.d_Toe
            //     << ": " << eph.d_Toc
            //     << ": " << eph.d_Cic
            //     << ": " << eph.d_OMEGA0
            //     << ": " << eph.d_Cis
            //     << ": " << eph.d_i_0
            //     << ": " << eph.d_Crc
            //     << ": " << eph.d_OMEGA
            //     << ": " << eph.d_OMEGA_DOT
            //     << ": " << eph.d_IDOT
            //     << ": " << eph.i_code_on_L2
            //     << ": " << eph.i_GPS_week
            //     << ": " << eph.b_L2_P_data_flag
            //     << ": " << eph.i_SV_accuracy
            //     << ": " << eph.i_SV_health
            //     << ": " << eph.d_TGD
            //     << ": " << eph.d_IODC
            //     << ": " << eph.d_IODE_SF2
            //     << ": " << eph.d_IODE_SF3
            //     << ": " << eph.i_AODO
            //     << ": " << eph.b_fit_interval_flag
            //     << ": " << eph.d_spare1
            //     << ": " << eph.d_spare2
            //     << ": " << eph.d_A_f0
            //     << ": " << eph.d_A_f1
            //     << ": " << eph.d_A_f2
            //     << ": " << eph.b_integrity_status_flag
            //     << ": " << eph.b_alert_flag
            //     << ": " << eph.b_antispoofing_flag
            //     << ": " << eph.d_satClkDrift
            //     << ": " << eph.d_dtr
            //     << ": " << eph.d_satpos_X
            //     << ": " << eph.d_satpos_Y
            //     << ": " << eph.d_satpos_Z
            //     << ": " << eph.d_satvel_X
            //     << ": " << eph.d_satvel_Y
            //     << ": " << eph.d_satvel_Z;

            // std::cout << std::endl << "Subframe 2 SV: " << sv
            //     << ": " << shrd_eph.d_TOW
            //     << ": " << shrd_eph.d_Crs
            //     << ": " << shrd_eph.d_Delta_n
            //     << ": " << shrd_eph.d_M_0
            //     << ": " << shrd_eph.d_Cuc
            //     << ": " << shrd_eph.d_e_eccentricity
            //     << ": " << shrd_eph.d_Cus
            //     << ": " << shrd_eph.d_sqrt_A
            //     << ": " << shrd_eph.d_Toe
            //     << ": " << shrd_eph.d_Toc
            //     << ": " << shrd_eph.d_Cic
            //     << ": " << shrd_eph.d_OMEGA0
            //     << ": " << shrd_eph.d_Cis
            //     << ": " << shrd_eph.d_i_0
            //     << ": " << shrd_eph.d_Crc
            //     << ": " << shrd_eph.d_OMEGA
            //     << ": " << shrd_eph.d_OMEGA_DOT
            //     << ": " << shrd_eph.d_IDOT
            //     << ": " << shrd_eph.i_code_on_L2
            //     << ": " << shrd_eph.i_GPS_week
            //     << ": " << shrd_eph.b_L2_P_data_flag
            //     << ": " << shrd_eph.i_SV_accuracy
            //     << ": " << shrd_eph.i_SV_health
            //     << ": " << shrd_eph.d_TGD
            //     << ": " << shrd_eph.d_IODC
            //     << ": " << shrd_eph.d_IODE_SF2
            //     << ": " << shrd_eph.d_IODE_SF3
            //     << ": " << shrd_eph.i_AODO
            //     << ": " << shrd_eph.b_fit_interval_flag
            //     << ": " << shrd_eph.d_spare1
            //     << ": " << shrd_eph.d_spare2
            //     << ": " << shrd_eph.d_A_f0
            //     << ": " << shrd_eph.d_A_f1
            //     << ": " << shrd_eph.d_A_f2
            //     << ": " << shrd_eph.b_integrity_status_flag
            //     << ": " << shrd_eph.b_alert_flag
            //     << ": " << shrd_eph.b_antispoofing_flag
            //     << ": " << shrd_eph.d_satClkDrift
            //     << ": " << shrd_eph.d_dtr
            //     << ": " << shrd_eph.d_satpos_X
            //     << ": " << shrd_eph.d_satpos_Y
            //     << ": " << shrd_eph.d_satpos_Z
            //     << ": " << shrd_eph.d_satvel_X
            //     << ": " << shrd_eph.d_satvel_Y
            //     << ": " << shrd_eph.d_satvel_Z
            //     << TEXT_RESET << std::endl;
            
            is_same = false;

        }
        eph_map.remove(sv);
    }
    eph_map.add(sv, shrd_eph);

    return shrd_eph;
}

bool gps_l1_ca_sd_telemetry_decoder_gs::decode_subframe(double doppler, double code_phase)
{
    char subframe[GPS_SUBFRAME_LENGTH];

    int32_t symbol_accumulator_counter = 0;
    int32_t frame_bit_index = 0;
    int32_t word_index = 0;
    uint32_t GPS_frame_4bytes = 0;
    float symbol_accumulator = 0;
    bool subframe_synchro_confirmation = true;
    for (float subframe_symbol : d_symbol_history)
        {
            // ******* SYMBOL TO BIT *******
            // extended correlation to bit period is enabled in tracking!
            symbol_accumulator += subframe_symbol;  // accumulate the input value in d_symbol_accumulator
            symbol_accumulator_counter++;
            if (symbol_accumulator_counter == 20)
                {
                    // symbol to bit
                    if (symbol_accumulator > 0)
                        {
                            GPS_frame_4bytes += 1;  // insert the telemetry bit in LSB
                            //std::cout << "1";
                        }
                    else
                        {
                            //std::cout << "0";
                        }
                    symbol_accumulator = 0;
                    symbol_accumulator_counter = 0;

                    // ******* bits to words ******
                    frame_bit_index++;
                    if (frame_bit_index == 30)
                        {
                            frame_bit_index = 0;
                            // parity check
                            // Each word in wordbuff is composed of:
                            //      Bits 0 to 29 = the GPS data word
                            //      Bits 30 to 31 = 2 LSBs of the GPS word ahead.
                            // prepare the extended frame [-2 -1 0 ... 30]
                            if (d_prev_GPS_frame_4bytes & 0x00000001)
                            {
                                GPS_frame_4bytes = GPS_frame_4bytes | 0x40000000;
                            }
                            if (d_prev_GPS_frame_4bytes & 0x00000002)
                            {
                                GPS_frame_4bytes = GPS_frame_4bytes | 0x80000000;
                            }
                            // Check that the 2 most recently logged words pass parity. Have to first
                            // invert the data bits according to bit 30 of the previous word.
                            if (GPS_frame_4bytes & 0x40000000)
                            {
                                GPS_frame_4bytes ^= 0x3FFFFFC0;  // invert the data bits (using XOR)
                            }
                            // check parity. If ANY word inside the subframe fails the parity, set subframe_synchro_confirmation = false
                            if (not gps_l1_ca_sd_telemetry_decoder_gs::gps_word_parityCheck(GPS_frame_4bytes))
                            {
                                subframe_synchro_confirmation = false;
                            }
                            if( d_spoofing_detector.stop_tracking(d_satellite.get_PRN(), uid) )
                            {
                                
                                LOG(WARNING) << "No spoofing - stop tracking channel " << this->d_channel;
                                stop_tracking(uid);
                            }
                            // add word to subframe
                            // insert the word in the correct position of the subframe
                            std::memcpy(&subframe[word_index * GPS_WORD_LENGTH], &GPS_frame_4bytes, sizeof(uint32_t));
                            word_index++;
                            d_prev_GPS_frame_4bytes = GPS_frame_4bytes;  // save the actual frame
                            GPS_frame_4bytes = 0;
                        }
                    else
                        {
                            GPS_frame_4bytes <<= 1;  // shift 1 bit left the telemetry word
                        }
                }
        }

    // decode subframe
    // NEW GPS SUBFRAME HAS ARRIVED!
    if (subframe_synchro_confirmation)
        {
            std::map<int32_t, Gps_Navigation_Message::Sbf> subframe_data = d_nav.subframe_decoder(subframe);  // decode the subframe

            std::map<int32_t, Gps_Navigation_Message::Sbf>::iterator it = subframe_data.begin();

            int32_t subframe_ID = it->first;

            if (subframe_ID > 0 and subframe_ID < 6)
                {
                    d_preamble_time_ms = d_preamble_time_ms *1000;
                    d_spoofing_detector.New_subframe(subframe_ID, d_nav.i_satellite_PRN, d_nav, d_preamble_time_ms, uid); //FS_IN is hardcoded right now
                  
                    std::map<int, int> sats = tracking_sats.get_map_copy();

                    switch (subframe_ID)
                    {
                        case 1:
                            std::cout << TEXT_BOLD_RED << std::endl << "Currently tracking SVs:" << TEXT_RESET;
                            
                            for(std::map<int, int>::iterator it = sats.begin(); it != sats.end(); ++it)
                            {
                                std::cout << TEXT_BOLD_MAGENTA << " " << it->first << TEXT_RESET;
                            }
                            std::cout << std::endl;
                            break;

                        case 3:  // we have a new set of ephemeris data for the current SV
                            if (d_nav.satellite_validation() == true)
                                {
                                    // get ephemeris object for this SV (mandatory)
                                    //Gps_Ephemeris tmp_eph = d_nav.get_ephemeris();
                                    //d_nav.get_ephemeris());//
                                    std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(d_nav.get_ephemeris());//compare_eph(d_nav.get_ephemeris()));
                                    {
                                        // /std::cout << std::endl << "Different subframe detected for SV " <<  d_nav.i_satellite_PRN <<std::endl;
                                    }
                                    // if(d_nav.i_satellite_PRN  == 32)
                                    // {
                                    // LOG(ERROR) << std::endl
                                    //     << "SV: " << tmp_obj->i_satellite_PRN 
                                    //     << "; d_TOW : " << tmp_obj->d_TOW
                                    //     << "; d_Crs : " << tmp_obj->d_Crs
                                    //     << "; d_Delta_n : " << tmp_obj->d_Delta_n
                                    //     << "; d_M_0 : " << tmp_obj->d_M_0
                                    //     << "; d_Cuc : " << tmp_obj->d_Cuc
                                    //     << "; d_e_eccentricity : " << tmp_obj->d_e_eccentricity
                                    //     << "; d_Cus : " << tmp_obj->d_Cus
                                    //     << "; d_sqrt_A : " << tmp_obj->d_sqrt_A
                                    //     << "; d_Toe : " << tmp_obj->d_Toe
                                    //     << "; d_Toc : " << tmp_obj->d_Toc
                                    //     << "; d_Cic : " << tmp_obj->d_Cic
                                    //     << "; d_OMEGA0 : " << tmp_obj->d_OMEGA0
                                    //     << "; d_Cis : " << tmp_obj->d_Cis
                                    //     << "; d_i_0 : " << tmp_obj->d_i_0
                                    //     << "; d_Crc : " << tmp_obj->d_Crc
                                    //     << "; d_OMEGA : " << tmp_obj->d_OMEGA
                                    //     << "; d_OMEGA_DOT : " << tmp_obj->d_OMEGA_DOT
                                    //     << "; d_IDOT : " << tmp_obj->d_IDOT
                                    //     << "; i_code_on_L2 : " << tmp_obj->i_code_on_L2
                                    //     << "; i_GPS_week : " << tmp_obj->i_GPS_week
                                    //     << "; b_L2_P_data_flag : " << tmp_obj->b_L2_P_data_flag
                                    //     << "; i_SV_accuracy : " << tmp_obj->i_SV_accuracy
                                    //     << "; i_SV_health : " << tmp_obj->i_SV_health
                                    //     << "; d_TGD : " << tmp_obj->d_TGD
                                    //     << "; d_IODC : " << tmp_obj->d_IODC
                                    //     << "; d_IODE_SF2 : " << tmp_obj->d_IODE_SF2
                                    //     << "; d_IODE_SF3 : " << tmp_obj->d_IODE_SF3
                                    //     << "; i_AODO : " << tmp_obj->i_AODO
                                    //     << "; b_fit_interval_flag : " << tmp_obj->b_fit_interval_flag
                                    //     << "; d_spare1 : " << tmp_obj->d_spare1
                                    //     << "; d_spare2 : " << tmp_obj->d_spare2
                                    //     << "; d_A_f0 : " << tmp_obj->d_A_f0
                                    //     << "; d_A_f1 : " << tmp_obj->d_A_f1
                                    //     << "; d_A_f2 : " << tmp_obj->d_A_f2
                                    //     << "; b_integrity_status_flag : " << tmp_obj->b_integrity_status_flag
                                    //     << "; b_alert_flag : " << tmp_obj->b_alert_flag
                                    //     << "; b_antispoofing_flag : " << tmp_obj->b_antispoofing_flag
                                    //     << "; d_satClkDrift : " << tmp_obj->d_satClkDrift
                                    //     << "; d_dtr : " << tmp_obj->d_dtr
                                    //     << "; d_satpos_X : " << tmp_obj->d_satpos_X
                                    //     << "; d_satpos_Y : " << tmp_obj->d_satpos_Y
                                    //     << "; d_satpos_Z : " << tmp_obj->d_satpos_Z
                                    //     << "; d_satvel_X : " << tmp_obj->d_satvel_X
                                    //     << "; d_satvel_Y : " << tmp_obj->d_satvel_Y
                                    //     << "; d_satvel_Z : " << tmp_obj->d_satvel_Z
                                    //     << std::endl;
                                    // }
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 4:  // Possible IONOSPHERE and UTC model update (page 18)
                            if (d_nav.flag_iono_valid == true)
                                {
                                    std::shared_ptr<Gps_Iono> tmp_obj = std::make_shared<Gps_Iono>(d_nav.get_iono());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            if (d_nav.flag_utc_model_valid == true)
                                {
                                    std::shared_ptr<Gps_Utc_Model> tmp_obj = std::make_shared<Gps_Utc_Model>(d_nav.get_utc_model());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 5:
                            // get almanac (if available)
                            //TODO: implement almanac reader in navigation_message
                            break;
                        default:
                            break;
                    }

                    std::cout << TEXT_BOLD_BLUE << "New GPS NAV message received in channel " << this->d_channel << ": "
                              << "subframe "
                              << subframe_ID << " from satellite "
                              << Gnss_Satellite(std::string("GPS"), d_nav.i_satellite_PRN)
                              << " at " << d_preamble_time_ms
                              << " peak " << i_peak << " id: " << uid << "[Doppler: " << doppler << "; Code delay: " << code_phase << "]" << TEXT_RESET << std::endl;
                    //global_subframe_map.clear();
                    return true;
                }
            else
                {
                    return false;
                }
        }
    else
        {
            return false;
        }
}


void gps_l1_ca_sd_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    flag_TOW_set = false;
    d_symbol_history.clear();
    d_stat = 0;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


int gps_l1_ca_sd_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    // 1. Copy the current tracking output
    Gnss_Synchro current_symbol = in[0][0];

    d_preamble_time_ms = current_symbol.Tracking_timestamp_secs;
    // SD - Set peak and uid for sub-frames
    i_peak = in[0][0].peak;
    d_nav.i_peak = i_peak;

    uid = in[0][0].uid;
    d_nav.uid = uid;

    double doppler = current_symbol.Acq_doppler_hz; 
    double code_phase = current_symbol.Acq_delay_samples;

    //LOG(WARNING) << "Setting UID in nav_message: " << uid << "; PRN: " << in[0][0].PRN << "; Channel: " << in[0][0].Channel_ID << "; Peak: " << i_peak;
    // add new symbol to the symbol queue
    d_symbol_history.push_back(current_symbol.Prompt_I);
    d_sample_counter++;  // count for the processed symbols
    consume_each(1);
    d_flag_preamble = false;
    // check if there is a problem with the telemetry of the current satellite
    if (d_stat < 2 and d_sent_tlm_failed_msg == false)
        {
            if ((d_sample_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame)
                {
                    int message = 1;  //bad telemetry
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
                    d_sent_tlm_failed_msg = true;
                }
        }
    
    // Make sure to remove this once PoC is ready
    if(current_symbol.PRN == 27)
    {
        //int message = 1;  //bad telemetry
        //this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
        //d_sent_tlm_failed_msg = true;
    }
    // ******* frame sync ******************
    switch (d_stat)
        {
        case 0:  // no preamble information
            {
                // correlate with preamble
                int32_t corr_value = 0;
                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_SYMBOLS)
                    {
                        // ******* preamble correlation ********
                        for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_SYMBOLS; i++)
                            {
                                if (d_symbol_history[i] < 0.0)  // symbols clipping
                                    {
                                        corr_value -= d_preamble_samples[i];
                                    }
                                else
                                    {
                                        corr_value += d_preamble_samples[i];
                                    }
                            }
                    }
                if (abs(corr_value) >= d_samples_per_preamble)
                    {
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                        DLOG(INFO) << "Preamble detection for GPS L1 satellite " << this->d_satellite;
                        d_stat = 1;  // enter into frame pre-detection status
                    }
                flag_TOW_set = false;
                break;
            }
        case 1:  // possible preamble lock
            {
                // correlate with preamble
                int32_t corr_value = 0;
                int32_t preamble_diff = 0;
                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_SYMBOLS)
                {
                    // ******* preamble correlation ********
                    for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_SYMBOLS; i++)
                    {
                        if (d_symbol_history[i] < 0.0)  // symbols clipping
                        {
                            corr_value -= d_preamble_samples[i];
                        }
                        else
                        {
                            corr_value += d_preamble_samples[i];
                        }
                    }
                }
                if (abs(corr_value) >= d_samples_per_preamble)
                {
                    // check 6 seconds of preamble separation
                    preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                    if (abs(preamble_diff - d_preamble_period_symbols) == 0)
                    {
                        DLOG(INFO) << "Preamble confirmation for SAT " << this->d_satellite;
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                        if (corr_value < 0) flag_PLL_180_deg_phase_locked = true;
                        d_stat = 2;
                    }
                    else
                    {
                        if (preamble_diff > d_preamble_period_symbols)
                        {
                            d_stat = 0;  // start again
                            flag_TOW_set = false;
                        }
                        else if (preamble_diff > GPS_SUBFRAME_MS+1000)
                        {
                            DLOG(INFO) << "No frame sync SAT: " <<  this->d_satellite << " preamble_diff= " << preamble_diff << "\n"
                                    << "Send stop tracking to tracking module for sat " << this->d_satellite << " ch: " << d_channel;
                            stop_tracking(uid);
                        }
                    }
                }
                break;
            }
        case 2:  // preamble acquired
            {
                if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                {
                    DLOG(INFO) << "Preamble received for SAT " << this->d_satellite << "d_sample_counter=" << d_sample_counter << "\n";
                    // call the decoder
                    // 0. fetch the symbols into an array
                    d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                    if (decode_subframe(doppler, code_phase))
                    {
                        d_CRC_error_counter = 0;
                        d_flag_preamble = true;  // valid preamble indicator (initialized to false every work())
                        gr::thread::scoped_lock lock(d_setlock);
                        d_last_valid_preamble = d_sample_counter;
                        if (!d_flag_frame_sync)
                        {
                            d_flag_frame_sync = true;
                            DLOG(INFO) << " Frame sync SAT " << this->d_satellite;
                        }
                    }
                    else
                    {
                        d_CRC_error_counter++;
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                        if (d_CRC_error_counter > 2)
                        {
                            DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                            d_flag_frame_sync = false;
                            d_stat = 0;
                            d_TOW_at_current_symbol_ms = 0;
                            d_TOW_at_Preamble_ms = 0;
                            d_CRC_error_counter = 0;
                            flag_TOW_set = false;
                        }
                    }
                }
                break;
            }
        }

    // 2. Add the telemetry decoder information
    if (d_flag_preamble == true)
        {
            if (!(d_nav.d_TOW == 0))
            {
                d_TOW_at_current_symbol_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0);
                d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0);
                flag_TOW_set = true;
            }
            else
            {
                DLOG(INFO) << "Received GPS L1 TOW equal to zero at sat " << d_nav.i_satellite_PRN;
            }
        }
    else
        {
            if (flag_TOW_set == true)
                {
                    d_TOW_at_current_symbol_ms += GPS_L1_CA_CODE_PERIOD_MS;
                }
        }
    
    if(!(d_flag_frame_sync == true and d_flag_parity == true))
    {
        std::string tmp = std::to_string(d_satellite.get_PRN())+ "0" + std::to_string(in[0][0].peak)+"0"+std::to_string(d_channel);
        
        int unique_id = in[0][0].uid; //std::stoi(std::to_string(d_satellite.get_PRN())+ "0" + std::to_string(in[0][0].peak)+"0"+std::to_string(d_channel));
        // DLOG(INFO) << "flag valid word: remove " << (int)unique_id << " "
        //<< d_flag_frame_sync << " " << d_flag_parity << " " <<  flag_TOW_set;
        global_subframe_map.remove((int)unique_id);
        global_subframe_check.remove((int)unique_id);
        global_gps_time.remove((int)unique_id);
        //LOG(WARNING) << "Removed uid: " << uid << "; Channel: " << d_channel;

    }

    if (flag_TOW_set == true)
    {
        current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
        current_symbol.Flag_valid_word = flag_TOW_set;

        if (flag_PLL_180_deg_phase_locked == true)
            {
                // correct the accumulated phase for the Costas loop phase shift, if required
                current_symbol.Carrier_phase_rads += GPS_PI;
            }

        if (d_dump == true)
            {
                // MULTIPLEXED FILE RECORDING - Record results to file
                try
                {
                    double tmp_double;
                    uint64_t tmp_ulong_int;
                    tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    tmp_ulong_int = current_symbol.Tracking_sample_counter;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                    tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
                catch (const std::ifstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing observables dump file " << e.what();
                }
            }

        // 3. Make the output (copy the object contents to the GNU Radio reserved memory)
        *out[0] = current_symbol;

        return 1;
    }
    return 0;
}

void gps_l1_ca_sd_telemetry_decoder_gs::stop_tracking(unsigned int uid)
{
    if( channel_state != 2 )
        {
            global_subframe_map.remove(uid);
            global_gps_time.remove(uid);
            global_subframe_check.remove(uid);
            channel_state = 2; 
            DLOG(INFO) << "send stop tracking " << uid; 
            this->message_port_pub(pmt::mp("events"), pmt::from_long(4));//4 -> stop tracking
            LOG(WARNING) << "Removed uid: " << uid << "; Channel: " << d_channel;
        }
}

void gps_l1_ca_sd_telemetry_decoder_gs::set_state(unsigned int state)
{
    channel_state = state;
}
