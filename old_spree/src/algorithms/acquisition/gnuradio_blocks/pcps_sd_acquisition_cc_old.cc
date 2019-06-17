/*!
 * \file pcps_sd_acquisition_cc.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Hildur Olafsdottir, 2015. ohildur(at)gmail.com 
 *          </ul>
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

#include "pcps_sd_acquisition_cc.h"
#include <sys/time.h>
#include <sstream>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk/volk.h>
#include "gnss_signal_processing.h"
#include "control_message_factory.h"
#include "concurrent_map.h"
#include "persistence1d.hpp"
#include <iostream>
#include <fstream>
#include <chrono>


using namespace p1d;
extern concurrent_map< std::map< std::string, int>> global_code_phase;

struct Peak{
    int code_phase;
    int doppler;
    float mag;
};

using google::LogMessage;

pcps_sd_acquisition_cc_sptr pcps_make_sd_acquisition_cc(
                                 unsigned int sampled_ms, unsigned int max_dwells,
                                 unsigned int doppler_max, long freq, long fs_in,
                                 int samples_per_ms, int samples_per_code,
                                 bool bit_transition_flag,
                                 gr::msg_queue::sptr queue, bool dump,
                                 std::string dump_filename)
{

    return pcps_sd_acquisition_cc_sptr(
            new pcps_sd_acquisition_cc(sampled_ms, max_dwells, doppler_max, freq, fs_in, samples_per_ms,
                                     samples_per_code, bit_transition_flag, queue, dump, dump_filename));
}

pcps_sd_acquisition_cc::pcps_sd_acquisition_cc(
                         unsigned int sampled_ms, unsigned int max_dwells,
                         unsigned int doppler_max, long freq, long fs_in,
                         int samples_per_ms, int samples_per_code,
                         bool bit_transition_flag,
                         gr::msg_queue::sptr queue, bool dump,
                         std::string dump_filename) :
    gr::block("pcps_sd_acquisition_cc",
    gr::io_signature::make(1, 1, sizeof(gr_complex) * sampled_ms * samples_per_ms),
    gr::io_signature::make(0, 0, sizeof(gr_complex) * sampled_ms * samples_per_ms))
{
    d_sample_counter = 0;    // SAMPLE COUNTER
    d_active = false;
    d_state = 0;
    d_queue = queue;
    d_freq = freq;
    d_fs_in = fs_in;
    d_samples_per_ms = samples_per_ms;
    d_samples_per_code = samples_per_code;
    DLOG(INFO) << "samples per code " << samples_per_code;
    d_sampled_ms = sampled_ms;
    d_max_dwells = max_dwells;
    d_well_count = 0;
    d_doppler_max = doppler_max;
    d_fft_size = d_sampled_ms * d_samples_per_ms;
    d_mag = 0;
    d_mag_2nd_highest = 0;

    d_input_power = 0.0;
    d_num_doppler_bins = 0;
    d_bit_transition_flag = bit_transition_flag;

    //todo: do something if posix_memalign fails
    if (posix_memalign((void**)&d_fft_codes, 16, d_fft_size * sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_magnitude, 16, d_fft_size * sizeof(float)) == 0){};

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    // For dumping samples into a file
    d_dump = dump;
    d_dump_filename = dump_filename;
}

pcps_sd_acquisition_cc::~pcps_sd_acquisition_cc()
{
    if (d_num_doppler_bins > 0)
        {
            for (unsigned int i = 0; i < d_num_doppler_bins; i++)
                {
                    free(d_grid_doppler_wipeoffs[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
        }

    free(d_fft_codes);
    free(d_magnitude);

    delete d_ifft;
    delete d_fft_if;

    if (d_dump)
        {
            d_dump_file.close();
        }
}

void pcps_sd_acquisition_cc::set_local_code(std::complex<float> * code)
{
    memcpy(d_fft_if->get_inbuf(), code, sizeof(gr_complex)*d_fft_size);

    d_fft_if->execute(); // We need the FFT of local code

    //Conjugate the local code
    if (is_unaligned())
        {
            volk_32fc_conjugate_32fc_u(d_fft_codes,d_fft_if->get_outbuf(),d_fft_size);
        }
    else
        {
            volk_32fc_conjugate_32fc_a(d_fft_codes,d_fft_if->get_outbuf(),d_fft_size);
        }
}

void pcps_sd_acquisition_cc::init()
{
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0;
    d_mag = 0.0;
    d_mag_2nd_highest = 0.0;
    d_input_power = 0.0;

    // Count the number of bins
    d_num_doppler_bins = 0;
    for (int doppler = (int)(-d_doppler_max);
         doppler <= (int)d_doppler_max;
         doppler += d_doppler_step)
    {
        d_num_doppler_bins++;
    }

    // Create the carrier Doppler wipeoff signals
    d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_bins];
    for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            if (posix_memalign((void**)&(d_grid_doppler_wipeoffs[doppler_index]), 16,
                               d_fft_size * sizeof(gr_complex)) == 0){};

            int doppler = -(int)d_doppler_max + d_doppler_step*doppler_index;
            complex_exp_gen_conj(d_grid_doppler_wipeoffs[doppler_index],
                                 d_freq + doppler, d_fs_in, d_fft_size);
        }

}

int pcps_sd_acquisition_cc::general_work(int noutput_items,
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    /*
     * By J.Arribas, L.Esteve and M.Molina
     * Acquisition strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. Record the maximum peak and the associated synchronization parameters
     * 5. Compute the test statistics and compare to the threshold
     * 6. Declare positive or negative acquisition using a message queue
     */

    int acquisition_message = -1; //0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL

    switch (d_state)
    {
    case 0:
        {
            if (d_active)
                {
                    //restart acquisition variables
                    d_gnss_synchro->Acq_delay_samples = 0.0;
                    d_gnss_synchro->Acq_doppler_hz = 0.0;
                    d_gnss_synchro->Acq_samplestamp_samples = 0;
                    d_well_count = 0;
                    d_mag = 0.0;
                    d_mag_2nd_highest = 0.0;
                    d_input_power = 0.0;
                    d_test_statistics = 0.0;

                    d_state = 1;
                }

            d_sample_counter += d_fft_size * ninput_items[0]; // sample counter
            consume_each(ninput_items[0]);

            break;
        }

    case 1:
        {

            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            // initialize acquisition algorithm
            int doppler;
            unsigned int indext = 0;
            float magt = 0.0;
            const gr_complex *in = (const gr_complex *)input_items[0]; //Get the input samples pointer
            float fft_normalization_factor = (float)d_fft_size * (float)d_fft_size;
            d_input_power = 0.0;
            d_mag = 0.0;
            d_mag_2nd_highest = 0.0;

            d_sample_counter += d_fft_size; // sample counter

            d_well_count++;


//            minimum_sample_difference = (d_samples_per_code/GPS_L1_CA_CODE_LENGTH_CHIPS)

            DLOG(INFO) << "Channel: " << d_channel
                    << ", doing acquisition of satellite: " << d_gnss_synchro->System << " "<< d_gnss_synchro->PRN
                    << ", sample stamp: " << d_sample_counter << ", threshold: "
                    << d_threshold << ", doppler_max: " << d_doppler_max
                    << ", doppler_step: " << d_doppler_step;

            bool acquire_auxiliary_peaks = false;
            if(d_peak != 0)  
                {
                    DLOG(INFO) << "acquire aux";
                    acquire_auxiliary_peaks = true;
                }
            
            // 1- Compute the input signal power estimation
            volk_32fc_magnitude_squared_32f_a(d_magnitude, in, d_fft_size);
            volk_32f_accumulator_s32f_a(&d_input_power, d_magnitude, d_fft_size);
            d_input_power /= (float)d_fft_size;
            
            //spoofing
            std::vector<float> peaks;
            float threshold_spoofing = d_threshold * d_input_power * (fft_normalization_factor * fft_normalization_factor); 
            std::map<float, Peak> d_highest_peaks;

            // 2- Doppler frequency search loop
            for (unsigned int doppler_index=0;doppler_index<d_num_doppler_bins;doppler_index++)
                {
                    // doppler search steps

                    doppler=-(int)d_doppler_max+d_doppler_step*doppler_index;

                    volk_32fc_x2_multiply_32fc_a(d_fft_if->get_inbuf(), in,
                                d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

                    // 3- Perform the FFT-based convolution  (parallel time search)
                    // Compute the FFT of the carrier wiped--off incoming signal
                    d_fft_if->execute();

                    // Multiply carrier wiped--off, Fourier transformed incoming signal
                    // with the local FFT'd code reference using SIMD operations with VOLK library
                    volk_32fc_x2_multiply_32fc_a(d_ifft->get_inbuf(),
                                d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                    // compute the inverse FFT
                    d_ifft->execute();

                    // Search maximum
                    volk_32fc_magnitude_squared_32f_a(d_magnitude, d_ifft->get_outbuf(), d_fft_size);

                    // TODO: write d_magnitude  
                    volk_32f_index_max_16u_a(&indext, d_magnitude, d_fft_size);

                    // Normalize the maximum value to correct the scale factor introduced by FFTW
                    magt = d_magnitude[indext] / (fft_normalization_factor * fft_normalization_factor);
                    if(acquire_auxiliary_peaks)
                    {    
                        for(unsigned int i = 0; i < d_fft_size; i++)
                        {
                            peaks.push_back(d_magnitude[i]);
                        }
                        //Find the local maxima for the peaks for each doppler bin
                        Persistence1D p;
                        std::vector<float> dp; 
                        if(*std::max_element(peaks.begin(), peaks.end()) >= threshold_spoofing) 
                            {
                                p.RunPersistence(peaks);
                                std::vector <TPairedExtrema> Extrema;
                                p.GetPairedExtrema(Extrema, 0);

                                for( std::vector< TPairedExtrema >::iterator it = Extrema.begin(); it != Extrema.end(); it++)
                                {
                                    if( peaks.at((*it).MaxIndex) >= threshold_spoofing)
                                        {
                                            Peak peak;
                                            peak.mag = peaks.at((*it).MaxIndex) / (fft_normalization_factor * fft_normalization_factor);
                                            peak.doppler = (double)doppler; 
                                            peak.code_phase = (*it).MaxIndex%d_samples_per_code;
                                            d_highest_peaks[peak.mag] = peak;
                                        }
                                }   
                        }
                        peaks.clear();
                    }

                    // 4- record the maximum peak and the associated synchronization parameters
                    if (d_mag < magt)
                        {
                            d_mag = magt;

                            // In case that d_bit_transition_flag = true, we compare the potentially
                            // new maximum test statistics (d_mag/d_input_power) with the value in
                            // d_test_statistics. When the second dwell is being processed, the value
                            // of d_mag/d_input_power could be lower than d_test_statistics (i.e,
                            // the maximum test statistics in the previous dwell is greater than
                            // current d_mag/d_input_power). Note that d_test_statistics is not
                            // restarted between consecutive dwells in multidwell operation.
                            if (d_test_statistics < (d_mag / d_input_power) || !d_bit_transition_flag)
                            {
                                d_gnss_synchro->Acq_delay_samples = (double)(indext % d_samples_per_code);
                                d_gnss_synchro->Acq_doppler_hz = (double)doppler;
                                d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;
                                // 5- Compute the test statistics and compare to the threshold
                                d_test_statistics = d_mag / d_input_power;

                            }
                        }

                    // Record results to file if required
                    if (d_dump)
                        {
                            
                            std::stringstream filename;
                            std::streamsize n = 2 * sizeof(float) * (d_fft_size); // complex file write
                            filename.str("");
                            filename << "../data/test_statistics_" << d_gnss_synchro->System
                                     <<"_" << d_gnss_synchro->Signal << "_sat_"
                                     << d_gnss_synchro->PRN << "_doppler_" <<  doppler << ".dat";
                            d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                            //d_dump_file.open(d_dump_filename, std::ios::out | std::ios::binary);
                            d_dump_file.write((char*)d_ifft->get_outbuf(), n); //write directly |abs(x)|^2 in this Doppler bin?
                            d_dump_file.close();
                        }
                }
            

            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
            DLOG(INFO) << "d_peak " << d_peak; 
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << d_test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            DLOG(INFO) << "magnitude " << d_mag;
            DLOG(INFO) << "input signal power " << d_input_power;


            bool found_peak = false;
            if(acquire_auxiliary_peaks)
                {    
                    std::map<float, Peak>::reverse_iterator rit;
                    std::map<float, Peak>::reverse_iterator rit2;
                    std::map<float, Peak> d_highest_peaks_reduced;
                    bool use_peak;
                    DLOG(INFO) << "### all peaks: ###" << d_highest_peaks.size();
                    for (rit=d_highest_peaks.rbegin(); rit!=d_highest_peaks.rend(); ++rit)
                    {
                        use_peak = true;
                        DLOG(INFO) << rit->second.code_phase << " " << rit->second.doppler <<" "<< rit->second.mag;
                        for (rit2=d_highest_peaks_reduced.rbegin(); rit2!=d_highest_peaks_reduced.rend(); ++rit2)
                        {
                            if(abs(rit->second.code_phase - rit2->second.code_phase) <= 1 && 
                               abs(rit->second.doppler - rit2->second.doppler) <= d_doppler_step)
                                {
                                    use_peak = false;
                                }

                        } 

                        if(use_peak)
                            {
                                d_highest_peaks_reduced[rit->first] = rit->second;
                            }
                    }
                    
                    /*DLOG(INFO) << "### ###";
                    for (rit=d_highest_peaks_reduced.rbegin(); rit!=d_highest_peaks_reduced.rend(); ++rit)
                    {
                        DLOG(INFO) << rit->second.code_phase << " " << rit->second.doppler <<" "<< rit->second.mag;
                    }
                    DLOG(INFO) << "### ###";
*/
                    //If there is more than one peak present, acquire the highest
                    if(d_peak == 1 && d_highest_peaks_reduced.size() > 0)
                    {
                        found_peak = true;
                    }
                    else if(d_highest_peaks_reduced.size() >= d_peak)
                    {
                        std::map<float, Peak>::reverse_iterator rit;

                        unsigned int i = 1;
                        DLOG(INFO) << "### peaks: ###";
                        for (rit=d_highest_peaks_reduced.rbegin(); rit!=d_highest_peaks_reduced.rend(); ++rit)
                        {
                            if(i == d_peak)
                                {
                                    found_peak = true; 
                                    DLOG(INFO) << "!!! peak found !!!";
                                    DLOG(INFO) << "peak " << rit->first; 
                                    DLOG(INFO) << "d_peak " << d_peak; 
                                    DLOG(INFO) << "code phase " << rit->second.code_phase; 
                                    d_test_statistics = rit->first/ d_input_power; 
                                    d_gnss_synchro->Acq_delay_samples = rit->second.code_phase; 
                                    d_gnss_synchro->Acq_doppler_hz = rit->second.doppler; 
                                }
                            
                            ++i;
                        }
                    }
            
                }

           std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
           auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>( t2 - t1 ).count();
           DLOG(INFO) << "duration " << acquire_auxiliary_peaks << " " << duration;

           if(acquire_auxiliary_peaks)
           {
               //found_peak = false; 
               //d_test_statistics = 0;
           }

            DLOG(INFO) << "found peak: " << found_peak << " aux " << acquire_auxiliary_peaks ;
            if (!d_bit_transition_flag)
                {
                    if(acquire_auxiliary_peaks && !found_peak)
                        {
                            DLOG(INFO) << "acq + no peak found";
                            d_state = 3; // Negative acquisition
                        }
                    else if (d_test_statistics > d_threshold)
                        {
                            d_state = 2; // Positive acquisition
                       }
                    else if (d_well_count == d_max_dwells)
                        {
                            d_state = 3; // Negative acquisition
                        }
                }
            else
                {
                    if (d_well_count == d_max_dwells) // d_max_dwells = 2
                        {
                            if(acquire_auxiliary_peaks && !found_peak)
                                {
                                    d_state = 3; // Negative acquisition
                                }
                            else if (d_test_statistics > d_threshold)
                                {
                                    d_state = 2; // Positive acquisition
                                }
                            else
                                {
                                    d_state = 3; // Negative acquisition
                                }
                        }
                }
            consume_each(1);

            break;
        }

    case 2:
        {
            // 6.1- Declare positive acquisition using a message queue
            DLOG(INFO) << "positive acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
            DLOG(INFO) << "peak: " << d_peak;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << d_test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            DLOG(INFO) << "magnitude " << d_mag;
            DLOG(INFO) << "input signal power " << d_input_power;

            d_active = false;
            d_state = 0;

            d_sample_counter += d_fft_size * ninput_items[0]; // sample counter
            consume_each(ninput_items[0]);

            acquisition_message = 1;
            d_channel_internal_queue->push(acquisition_message);

            break;
        }

    case 3:
        {
            // 6.2- Declare negative acquisition using a message queue
            DLOG(INFO) << "negative acquisition";
            DLOG(INFO) << "satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
            DLOG(INFO) << "sample_stamp " << d_sample_counter;
            DLOG(INFO) << "test statistics value " << d_test_statistics;
            DLOG(INFO) << "test statistics threshold " << d_threshold;
            DLOG(INFO) << "code phase " << d_gnss_synchro->Acq_delay_samples;
            DLOG(INFO) << "doppler " << d_gnss_synchro->Acq_doppler_hz;
            DLOG(INFO) << "magnitude " << d_mag;
            DLOG(INFO) << "input signal power " << d_input_power;

            d_active = false;
            d_state = 0;

            d_sample_counter += d_fft_size * ninput_items[0]; // sample counter
            consume_each(ninput_items[0]);

            acquisition_message = 2;
            d_channel_internal_queue->push(acquisition_message);

            break;
        }
    }

    return 0;
}
