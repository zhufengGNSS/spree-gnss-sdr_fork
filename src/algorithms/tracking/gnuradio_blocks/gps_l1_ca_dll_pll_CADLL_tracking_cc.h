/*!
 * \file gps_l1_ca_dll_pll_tracking_cc.h
 * \brief Interface of a code DLL + carrier PLL tracking block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007
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
 * (at your option) any later version.
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

#ifndef GNSS_SDR_GPS_L1_CA_DLL_PLL_CADLL_TRACKING_CC_H
#define	GNSS_SDR_GPS_L1_CA_DLL_PLL_CADLL_TRACKING_CC_H

#include <fstream>
#include <queue>
#include <map>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/block.h>
#include <gnuradio/msg_queue.h>
#include "concurrent_queue.h"
#include "gps_sdr_signal_processing.h"
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_ALL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include "integrator.h"
#include "correlator.h"

class Gps_L1_Ca_Dll_Pll_CADLL_Tracking_cc;

typedef boost::shared_ptr<Gps_L1_Ca_Dll_Pll_CADLL_Tracking_cc>
        gps_l1_ca_dll_pll_CADLL_tracking_cc_sptr;

gps_l1_ca_dll_pll_CADLL_tracking_cc_sptr
gps_l1_ca_dll_pll_CADLL_make_tracking_cc(long if_freq,
                                   long fs_in, unsigned
                                   int vector_length,
                                   boost::shared_ptr<gr::msg_queue> queue,
                                   bool dump,
                                   std::string dump_filename,
                                   float pll_bw_hz,
                                   float dll_bw_hz,
                                   float early_late_space_chips);



/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Gps_L1_Ca_Dll_Pll_CADLL_Tracking_cc: public gr::block
{
public:
    ~Gps_L1_Ca_Dll_Pll_CADLL_Tracking_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();
    void stop_tracking();
    void set_channel_queue(concurrent_queue<int> *channel_internal_queue);

    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast (int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend gps_l1_ca_dll_pll_CADLL_tracking_cc_sptr
    gps_l1_ca_dll_pll_CADLL_make_tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            boost::shared_ptr<gr::msg_queue> queue,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips);

    Gps_L1_Ca_Dll_Pll_CADLL_Tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            boost::shared_ptr<gr::msg_queue> queue,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips);
    void update_local_code();
    void update_local_code_m();
    void update_local_carrier();

    // tracking configuration vars
    boost::shared_ptr<gr::msg_queue> d_queue;
    concurrent_queue<int> *d_channel_internal_queue;
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;
    int d_last_seg;
    long d_if_freq;
    long d_fs_in;

    double d_early_late_spc_chips;

    gr_complex* d_ca_code;

    gr_complex* d_early_code;
    gr_complex* d_late_code;
    gr_complex* d_prompt_code;
    gr_complex* d_early_code_m;
    gr_complex* d_late_code_m;
    gr_complex* d_prompt_code_m;
    gr_complex* d_carr_sign;

    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;
    gr_complex *d_Early_m;
    gr_complex *d_Prompt_m;
    gr_complex *d_Late_m;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_samples_m;
    float d_rem_carr_phase_rad;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_2nd_DLL_filter d_code_loop_filter_m;
    Tracking_2nd_ALL_filter d_amplitude_loop_filter;
    Tracking_2nd_ALL_filter d_amplitude_loop_filter_m;
    Tracking_2nd_PLL_filter d_carrier_loop_filter;
    Integrator d_integrator;
    Integrator d_integrator_m;

    //ALL
    float a1 = 1e-5;
    float a2;
    bool first_run = true;

    // acquisition
    float d_acq_code_phase_samples;
    float d_acq_carrier_doppler_hz;
    // correlator
    Correlator d_correlator;

    // tracking vars
    double d_code_freq_chips;
    double d_code_freq_chips_m;
    float d_carrier_doppler_hz;
    float d_acc_carrier_phase_rad;
    float d_code_phase_samples;
    float d_acc_code_phase_secs;
    float d_acc_code_phase_secs_m;

    double delay;

    //PRN period in samples
    int d_current_prn_length_samples;
    int d_current_prn_length_samples_m;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_sample_counter_m;
    unsigned long int d_acq_sample_stamp;
    double d_sample_counter_f;
    double d_sample_counter_fm;
    

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    float d_carrier_lock_test;
    float d_CN0_SNV_dB_Hz;
    float d_carrier_lock_threshold;
    int d_carrier_lock_fail_counter;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    //init CADLL
    bool CADLL_init = true;
};

#endif //GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_CC_H
