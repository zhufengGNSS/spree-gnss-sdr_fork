/*!
 * \file pcps_sd_acquisition.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Cillian O'Driscoll, 2017. cillian(at)ieee.org
 *          </ul>
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

#include "pcps_sd_acquisition.h"
#include "GLONASS_L1_L2_CA.h"  // for GLONASS_TWO_PI
#include "GPS_L1_CA.h"         // for GPS_TWO_PI
#include "gnss_frequencies.h"
#include "gnss_sdr_create_directory.h"
#include "gnss_synchro.h"
#if HAS_STD_FILESYSTEM
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
#include <experimental/filesystem>
#else
#include <filesystem>
#endif
#else
#include <boost/filesystem/path.hpp>
#endif
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <pmt/pmt.h>        // for from_long
#include <pmt/pmt_sugar.h>  // for mp
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // for fill_n, min
#include <cmath>      // for floor, fmod, rint, ceil
#include <cstring>    // for memcpy
#include <iostream>
#include <map>
#include "persistence1d.hpp"


#if HAS_STD_FILESYSTEM
#if HAS_STD_FILESYSTEM_EXPERIMENTAL
namespace fs = std::experimental::filesystem;
#else
namespace fs = std::filesystem;
#endif
#else
namespace fs = boost::filesystem;
#endif

pcps_sd_acquisition_sptr pcps_make_sd_acquisition(const Acq_Conf& conf_)
{
    return pcps_sd_acquisition_sptr(new pcps_sd_acquisition(conf_));
}


pcps_sd_acquisition::pcps_sd_acquisition(const Acq_Conf& conf_) : gr::block("pcps_sd_acquisition",
                                                                gr::io_signature::make(1, 1, conf_.it_size),
                                                                gr::io_signature::make(0, 0, conf_.it_size))
{
    this->message_port_register_out(pmt::mp("events"));

    acq_parameters = conf_;
    d_sample_counter = 0ULL;  // SAMPLE COUNTER
    d_active = false;
    d_positive_acq = 0;
    d_state = 0;
    d_old_freq = 0LL;
    d_num_noncoherent_integrations_counter = 0U;
    d_consumed_samples = acq_parameters.sampled_ms * acq_parameters.samples_per_ms * (acq_parameters.bit_transition_flag ? 2 : 1);
    if (acq_parameters.sampled_ms == acq_parameters.ms_per_code)
        {
            d_fft_size = d_consumed_samples;
        }
    else
        {
            d_fft_size = d_consumed_samples * 2;
        }
    // d_fft_size = next power of two?  ////
    d_mag = 0;
    d_input_power = 0.0;
    d_num_doppler_bins = 0U;
    d_threshold = 0.0;
    d_doppler_step = 0U;
    d_doppler_center_step_two = 0.0;
    d_test_statistics = 0.0;
    d_channel = 0U;
    if (conf_.it_size == sizeof(gr_complex))
        {
            d_cshort = false;
        }
    else
        {
            d_cshort = true;
        }

    // COD:
    // Experimenting with the overlap/save technique for handling bit trannsitions
    // The problem: Circular correlation is asynchronous with the received code.
    // In effect the first code phase used in the correlation is the current
    // estimate of the code phase at the start of the input buffer. If this is 1/2
    // of the code period a bit transition would move all the signal energy into
    // adjacent frequency bands at +/- 1/T where T is the integration time.
    //
    // We can avoid this by doing linear correlation, effectively doubling the
    // size of the input buffer and padding the code with zeros.
    if (acq_parameters.bit_transition_flag)
        {
            d_fft_size = d_consumed_samples * 2;
            acq_parameters.max_dwells = 1;  // Activation of acq_parameters.bit_transition_flag invalidates the value of acq_parameters.max_dwells
        }

    d_tmp_buffer = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
    d_fft_codes = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_magnitude = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
    d_input_signal = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // Direct FFT
    d_fft_if = new gr::fft::fft_complex(d_fft_size, true);

    // Inverse FFT
    d_ifft = new gr::fft::fft_complex(d_fft_size, false);

    d_gnss_synchro = nullptr;
    d_grid_doppler_wipeoffs = nullptr;
    d_grid_doppler_wipeoffs_step_two = nullptr;
    d_magnitude_grid = nullptr;
    d_worker_active = false;
    d_data_buffer = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_consumed_samples * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    if (d_cshort)
        {
            d_data_buffer_sc = static_cast<lv_16sc_t*>(volk_gnsssdr_malloc(d_consumed_samples * sizeof(lv_16sc_t), volk_gnsssdr_get_alignment()));
        }
    else
        {
            d_data_buffer_sc = nullptr;
        }
    grid_ = arma::fmat();
    narrow_grid_ = arma::fmat();
    d_step_two = false;
    d_num_doppler_bins_step2 = acq_parameters.num_doppler_bins_step2;

    d_samplesPerChip = acq_parameters.samples_per_chip;
    d_buffer_count = 0U;
    // todo: CFAR statistic not available for non-coherent integration
    if (acq_parameters.max_dwells == 1)
        {
            d_use_CFAR_algorithm_flag = acq_parameters.use_CFAR_algorithm_flag;
        }
    else
        {
            d_use_CFAR_algorithm_flag = false;
        }
    d_dump_number = 0LL;
    d_dump_channel = acq_parameters.dump_channel;
    d_dump = acq_parameters.dump;
    d_dump_filename = acq_parameters.dump_filename;
    if (d_dump)
        {
            std::string dump_path;
            // Get path
            if (d_dump_filename.find_last_of('/') != std::string::npos)
                {
                    std::string dump_filename_ = d_dump_filename.substr(d_dump_filename.find_last_of('/') + 1);
                    dump_path = d_dump_filename.substr(0, d_dump_filename.find_last_of('/'));
                    d_dump_filename = dump_filename_;
                }
            else
                {
                    dump_path = std::string(".");
                }
            if (d_dump_filename.empty())
                {
                    d_dump_filename = "acquisition";
                }
            // remove extension if any
            if (d_dump_filename.substr(1).find_last_of('.') != std::string::npos)
                {
                    d_dump_filename = d_dump_filename.substr(0, d_dump_filename.find_last_of('.'));
                }
            d_dump_filename = dump_path + fs::path::preferred_separator + d_dump_filename;
            // create directory
            if (!gnss_sdr_create_directory(dump_path))
                {
                    std::cerr << "GNSS-SDR cannot create dump file for the Acquisition block. Wrong permissions?" << std::endl;
                    d_dump = false;
                }
        }
}


pcps_sd_acquisition::~pcps_sd_acquisition()
{
    if (d_num_doppler_bins > 0)
        {
            for (uint32_t i = 0; i < d_num_doppler_bins; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs[i]);
                    volk_gnsssdr_free(d_magnitude_grid[i]);
                }
            delete[] d_grid_doppler_wipeoffs;
            delete[] d_magnitude_grid;
        }
    if (acq_parameters.make_2_steps)
        {
            for (uint32_t i = 0; i < d_num_doppler_bins_step2; i++)
                {
                    volk_gnsssdr_free(d_grid_doppler_wipeoffs_step_two[i]);
                }
            delete[] d_grid_doppler_wipeoffs_step_two;
        }
    volk_gnsssdr_free(d_fft_codes);
    volk_gnsssdr_free(d_magnitude);
    volk_gnsssdr_free(d_tmp_buffer);
    volk_gnsssdr_free(d_input_signal);
    delete d_ifft;
    delete d_fft_if;
    volk_gnsssdr_free(d_data_buffer);
    if (d_cshort)
        {
            volk_gnsssdr_free(d_data_buffer_sc);
        }
}


void pcps_sd_acquisition::set_resampler_latency(uint32_t latency_samples)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    acq_parameters.resampler_latency_samples = latency_samples;
}


void pcps_sd_acquisition::set_local_code(std::complex<float>* code)
{
    // reset the intermediate frequency
    d_old_freq = 0LL;
    // This will check if it's fdma, if yes will update the intermediate frequency and the doppler grid
    if (is_fdma())
        {
            update_grid_doppler_wipeoffs();
        }
    // COD
    // Here we want to create a buffer that looks like this:
    // [ 0 0 0 ... 0 c_0 c_1 ... c_L]
    // where c_i is the local code and there are L zeros and L chips
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    if (acq_parameters.bit_transition_flag)
        {
            int32_t offset = d_fft_size / 2;
            std::fill_n(d_fft_if->get_inbuf(), offset, gr_complex(0.0, 0.0));
            memcpy(d_fft_if->get_inbuf() + offset, code, sizeof(gr_complex) * offset);
        }
    else
        {
            if (acq_parameters.sampled_ms == acq_parameters.ms_per_code)
                {
                    memcpy(d_fft_if->get_inbuf(), code, sizeof(gr_complex) * d_consumed_samples);
                }
            else
                {
                    std::fill_n(d_fft_if->get_inbuf(), d_fft_size - d_consumed_samples, gr_complex(0.0, 0.0));
                    memcpy(d_fft_if->get_inbuf() + d_consumed_samples, code, sizeof(gr_complex) * d_consumed_samples);
                }
        }

    d_fft_if->execute();  // We need the FFT of local code
    volk_32fc_conjugate_32fc(d_fft_codes, d_fft_if->get_outbuf(), d_fft_size);
}


bool pcps_sd_acquisition::is_fdma()
{
    // Dealing with FDMA system
    if (strcmp(d_gnss_synchro->Signal, "1G") == 0)
        {
            d_old_freq += DFRQ1_GLO * GLONASS_PRN.at(d_gnss_synchro->PRN);
            LOG(INFO) << "Trying to acquire SV PRN " << d_gnss_synchro->PRN << " with freq " << d_old_freq << " in Glonass Channel " << GLONASS_PRN.at(d_gnss_synchro->PRN) << std::endl;
            return true;
        }
    if (strcmp(d_gnss_synchro->Signal, "2G") == 0)
        {
            d_old_freq += DFRQ2_GLO * GLONASS_PRN.at(d_gnss_synchro->PRN);
            LOG(INFO) << "Trying to acquire SV PRN " << d_gnss_synchro->PRN << " with freq " << d_old_freq << " in Glonass Channel " << GLONASS_PRN.at(d_gnss_synchro->PRN) << std::endl;
            return true;
        }
    return false;
}


void pcps_sd_acquisition::update_local_carrier(gr_complex* carrier_vector, int32_t correlator_length_samples, float freq)
{
    float phase_step_rad;
    if (acq_parameters.use_automatic_resampler)
        {
            phase_step_rad = GPS_TWO_PI * freq / static_cast<float>(acq_parameters.resampled_fs);
        }
    else
        {
            phase_step_rad = GPS_TWO_PI * freq / static_cast<float>(acq_parameters.fs_in);
        }
    float _phase[1];
    _phase[0] = 0.0;
    volk_gnsssdr_s32f_sincos_32fc(carrier_vector, -phase_step_rad, _phase, correlator_length_samples);
}


void pcps_sd_acquisition::init()
{
    d_gnss_synchro->Flag_valid_acquisition = false;
    d_gnss_synchro->Flag_valid_symbol_output = false;
    d_gnss_synchro->Flag_valid_pseudorange = false;
    d_gnss_synchro->Flag_valid_word = false;
    d_gnss_synchro->Acq_doppler_step = 0U;
    d_gnss_synchro->Acq_delay_samples = 0.0;
    d_gnss_synchro->Acq_doppler_hz = 0.0;
    d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
    d_mag = 0.0;
    d_input_power = 0.0;

    d_num_doppler_bins = static_cast<uint32_t>(std::ceil(static_cast<double>(static_cast<int32_t>(acq_parameters.doppler_max) - static_cast<int32_t>(-acq_parameters.doppler_max)) / static_cast<double>(d_doppler_step)));

    // Create the carrier Doppler wipeoff signals
    if (d_grid_doppler_wipeoffs == nullptr)
        {
            d_grid_doppler_wipeoffs = new gr_complex*[d_num_doppler_bins];
        }
    if (acq_parameters.make_2_steps && (d_grid_doppler_wipeoffs_step_two == nullptr))
        {
            d_grid_doppler_wipeoffs_step_two = new gr_complex*[d_num_doppler_bins_step2];
            for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
                {
                    d_grid_doppler_wipeoffs_step_two[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
                }
        }

    if (d_magnitude_grid == nullptr)
        {
            d_magnitude_grid = new float*[d_num_doppler_bins];
            for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                {
                    d_grid_doppler_wipeoffs[doppler_index] = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_fft_size * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
                    d_magnitude_grid[doppler_index] = static_cast<float*>(volk_gnsssdr_malloc(d_fft_size * sizeof(float), volk_gnsssdr_get_alignment()));
                }
        }

    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            for (uint32_t k = 0; k < d_fft_size; k++)
                {
                    d_magnitude_grid[doppler_index][k] = 0.0;
                }
            int32_t doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, d_old_freq + doppler);
        }

    d_worker_active = false;

    if (d_dump)
        {
            uint32_t effective_fft_size = (acq_parameters.bit_transition_flag ? (d_fft_size / 2) : d_fft_size);
            grid_ = arma::fmat(effective_fft_size, d_num_doppler_bins, arma::fill::zeros);
            narrow_grid_ = arma::fmat(effective_fft_size, d_num_doppler_bins_step2, arma::fill::zeros);
        }
}


void pcps_sd_acquisition::update_grid_doppler_wipeoffs()
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
        {
            int32_t doppler = -static_cast<int32_t>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;
            update_local_carrier(d_grid_doppler_wipeoffs[doppler_index], d_fft_size, d_old_freq + doppler);
        }
}


void pcps_sd_acquisition::update_grid_doppler_wipeoffs_step2()
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
        {
            float doppler = (static_cast<float>(doppler_index) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * acq_parameters.doppler_step2;
            update_local_carrier(d_grid_doppler_wipeoffs_step_two[doppler_index], d_fft_size, d_doppler_center_step_two + doppler);
        }
}


void pcps_sd_acquisition::set_state(int32_t state)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_state = state;
    if (d_state == 1)
        {
            d_gnss_synchro->Acq_delay_samples = 0.0;
            d_gnss_synchro->Acq_doppler_hz = 0.0;
            d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
            d_gnss_synchro->Acq_doppler_step = 0U;
            d_mag = 0.0;
            d_input_power = 0.0;
            d_test_statistics = 0.0;
            d_active = true;
        }
    else if (d_state == 0)
        {
        }
    else
        {
            LOG(ERROR) << "State can only be set to 0 or 1";
        }
}


void pcps_sd_acquisition::send_positive_acquisition()
{
    // Declare positive acquisition using a message port
    // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    LOG(INFO) << "================= POSITIVE ACQ =================";
    LOG(INFO) <<"Positive acquisition";
    LOG(INFO) <<"Channel " << d_gnss_synchro->Channel_ID;
    LOG(INFO) <<"Satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
    LOG(INFO) <<"Sample_stamp " << d_sample_counter;
    LOG(INFO) <<"Test statistics value " << d_test_statistics;
    LOG(INFO) <<"Test statistics threshold " << d_threshold;
    LOG(INFO) <<"Code phase " << d_gnss_synchro->Acq_delay_samples;
    LOG(INFO) <<"Doppler " << d_gnss_synchro->Acq_doppler_hz;
    LOG(INFO) <<"Magnitude " << d_mag;
    LOG(INFO) <<"Input signal power " << d_input_power;
    d_positive_acq = 1;
    LOG(INFO) << "=================================================";

    if (!d_channel_fsm.expired())
        {
            //the channel FSM is set, so, notify it directly the positive acquisition to minimize delays
            d_channel_fsm.lock()->Event_valid_acquisition();
        }
    else
        {
            this->message_port_pub(pmt::mp("events"), pmt::from_long(1));
        }
}


void pcps_sd_acquisition::send_negative_acquisition()
{
    // Declare negative acquisition using a message port
    // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    LOG(INFO) << "================= NEGATIVE ACQ =================";
    LOG(INFO) <<"Negative acquisition";
    LOG(INFO) <<"Channel " << d_gnss_synchro->Channel_ID;
    LOG(INFO) <<"Satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;
    LOG(INFO) <<"Sample_stamp " << d_sample_counter;
    LOG(INFO) <<"Test statistics value " << d_test_statistics;
    LOG(INFO) <<"Test statistics threshold " << d_threshold;
    LOG(INFO) <<"Code phase " << d_gnss_synchro->Acq_delay_samples;
    LOG(INFO) <<"Doppler " << d_gnss_synchro->Acq_doppler_hz;
    LOG(INFO) <<"Magnitude " << d_mag;
    LOG(INFO) <<"Input signal power " << d_input_power;
    d_positive_acq = 1;
    LOG(INFO) << "=================================================";
    this->message_port_pub(pmt::mp("events"), pmt::from_long(2));
}


void pcps_sd_acquisition::dump_results(int32_t effective_fft_size)
{
    d_dump_number++;
    std::string filename = d_dump_filename;
    filename.append("_");
    filename.append(1, d_gnss_synchro->System);
    filename.append("_");
    filename.append(1, d_gnss_synchro->Signal[0]);
    filename.append(1, d_gnss_synchro->Signal[1]);
    filename.append("_ch_");
    filename.append(std::to_string(d_channel));
    filename.append("_");
    filename.append(std::to_string(d_dump_number));
    filename.append("_sat_");
    filename.append(std::to_string(d_gnss_synchro->PRN));
    filename.append(".mat");

    mat_t* matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    if (matfp == nullptr)
        {
            std::cout << "Unable to create or open Acquisition dump file" << std::endl;
            //acq_parameters.dump = false;
        }
    else
        {
            size_t dims[2] = {static_cast<size_t>(effective_fft_size), static_cast<size_t>(d_num_doppler_bins)};
            matvar_t* matvar = Mat_VarCreate("acq_grid", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, grid_.memptr(), 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            dims[0] = static_cast<size_t>(1);
            dims[1] = static_cast<size_t>(1);
            matvar = Mat_VarCreate("doppler_max", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &acq_parameters.doppler_max, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("doppler_step", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_doppler_step, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("d_positive_acq", MAT_C_INT32, MAT_T_INT32, 1, dims, &d_positive_acq, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            auto aux = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
            matvar = Mat_VarCreate("acq_doppler_hz", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            aux = static_cast<float>(d_gnss_synchro->Acq_delay_samples);
            matvar = Mat_VarCreate("acq_delay_samples", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("test_statistic", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_test_statistics, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("threshold", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_threshold, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("input_power", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &d_input_power, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("sample_counter", MAT_C_UINT64, MAT_T_UINT64, 1, dims, &d_sample_counter, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_gnss_synchro->PRN, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("num_dwells", MAT_C_UINT32, MAT_T_UINT32, 1, dims, &d_num_noncoherent_integrations_counter, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            if (acq_parameters.make_2_steps)
                {
                    dims[0] = static_cast<size_t>(effective_fft_size);
                    dims[1] = static_cast<size_t>(d_num_doppler_bins_step2);
                    matvar = Mat_VarCreate("acq_grid_narrow", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, narrow_grid_.memptr(), 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    dims[0] = static_cast<size_t>(1);
                    dims[1] = static_cast<size_t>(1);
                    matvar = Mat_VarCreate("doppler_step_narrow", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &acq_parameters.doppler_step2, 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);

                    aux = d_doppler_center_step_two - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0)) * acq_parameters.doppler_step2;
                    matvar = Mat_VarCreate("doppler_grid_narrow_min", MAT_C_SINGLE, MAT_T_SINGLE, 1, dims, &aux, 0);
                    Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
                    Mat_VarFree(matvar);
                }

            Mat_Close(matfp);
        }
}


std::map<float, pcps_sd_acquisition::Peak> pcps_sd_acquisition::max_to_input_power_statistic(uint32_t& indext, int32_t& doppler, float input_power, float spoofing_threshold, bool acquire_auxiliary_peaks,uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step)
{
    float grid_maximum = 0.0;
    uint32_t index_doppler = 0U;
    uint32_t tmp_intex_t = 0U;
    uint32_t index_time = 0U;
    float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);
    float magt;

    std::map<float, pcps_sd_acquisition::Peak>peaks_map;
    std::vector<float>peaks;
    pcps_sd_acquisition::Peak peak;

    // Find the correlation peak and the carrier frequency
    if(acquire_auxiliary_peaks && acq_parameters.spoofing_detection)
    {
        LOG(INFO) << "================= ALL QUALIFYING PEAKS ================= " << spoofing_threshold;
    }

    for (uint32_t i = 0; i < num_doppler_bins; i++)
        {
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_magnitude_grid[i], d_fft_size);
            

            if(acquire_auxiliary_peaks && acq_parameters.spoofing_detection)
            {
                for(int j = 0; j < d_fft_size; j++)
                {
                    peaks.push_back(d_magnitude_grid[i][j]);
                }
                p1d::Persistence1D p;
                std::vector<float> dp; 
                if(*std::max_element(peaks.begin(), peaks.end()) >= spoofing_threshold) 
                {
                    p.RunPersistence(peaks);
                    std::vector <p1d::TPairedExtrema> Extrema;
                    p.GetPairedExtrema(Extrema, 0);

                    for( std::vector< p1d::TPairedExtrema >::iterator it = Extrema.begin(); it != Extrema.end(); it++)
                    {
                        if( peaks.at((*it).MaxIndex) >= spoofing_threshold)
                        {
                            Peak peak;
                            peak.mag = peaks.at((*it).MaxIndex) / (fft_normalization_factor * fft_normalization_factor);
                            if (!d_step_two)
                            {
                                doppler = -static_cast<int32_t>(doppler_max) + doppler_step * static_cast<int32_t>(i);
                            }
                            else
                            {
                                doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(i) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * acq_parameters.doppler_step2);
                            }
                            //peak.mag = d_magnitude_grid[i][tmp_intex_t] / (fft_normalization_factor * fft_normalization_factor);
                            peak.doppler = (double)doppler;
                            if(acq_parameters.use_automatic_resampler)
                            {
                                peak.code_phase = static_cast<double>(std::fmod(static_cast<float>((*it).MaxIndex), acq_parameters.samples_per_code)) * acq_parameters.resampler_ratio;
                                peak.code_phase -= static_cast<double>(acq_parameters.resampler_latency_samples);  //account the resampler filter latency
                            }
                            else
                            {
                                peak.code_phase = static_cast<double>(std::fmod(static_cast<float>((*it).MaxIndex), acq_parameters.samples_per_code));
                            }
                            peak.test_stats = peak.mag / input_power;
                            peaks_map[peak.mag] = peak;
                        }
                    }   
                }
                peaks.clear();
                //LOG(INFO) << "Peak: " << peak.mag << "; Code phase " << peak.code_phase << "; Doppler " << peak.doppler << "; Test statistics " << peak.test_stats;
            }
            else if (d_magnitude_grid[i][tmp_intex_t] > grid_maximum && !acq_parameters.spoofing_detection)
            {
                grid_maximum = d_magnitude_grid[i][tmp_intex_t];
                index_doppler = i;
                index_time = tmp_intex_t;
            }
        }
    indext = index_time;
    if(!acq_parameters.spoofing_detection)
    {
        //Acquire the highest peak if auxiliary peak detection is disabled
        if (!d_step_two)
        {
            doppler = -static_cast<int32_t>(doppler_max) + doppler_step * static_cast<int32_t>(index_doppler);
        }
        else
        {
            doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * acq_parameters.doppler_step2);
        }
        peak.mag = grid_maximum / (fft_normalization_factor * fft_normalization_factor);
        peak.doppler = doppler; 
        peak.code_phase = static_cast<double>(std::fmod(static_cast<float>(indext), acq_parameters.samples_per_code));
        peak.test_stats = peak.mag / input_power;
        peaks_map[peak.mag] = peak;
        LOG(INFO) << "Highest peak: " << peak.mag << "; Code phase " << peak.code_phase << "; Doppler " << peak.doppler << "; Test statistics " << peak.test_stats;
    }
    else
    {
        //Find the local maxima for the peaks for each doppler bin
        
    }
    
    return peaks_map;
}


float pcps_sd_acquisition::first_vs_second_peak_statistic(uint32_t& indext, int32_t& doppler, uint32_t num_doppler_bins, int32_t doppler_max, int32_t doppler_step)
{
    // Look for correlation peaks in the results
    // Find the highest peak and compare it to the second highest peak
    // The second peak is chosen not closer than 1 chip to the highest peak

    float firstPeak = 0.0;
    uint32_t index_doppler = 0U;
    uint32_t tmp_intex_t = 0U;
    uint32_t index_time = 0U;

    // Find the correlation peak and the carrier frequency
    for (uint32_t i = 0; i < num_doppler_bins; i++)
        {
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_magnitude_grid[i], d_fft_size);
            if (d_magnitude_grid[i][tmp_intex_t] > firstPeak)
                {
                    firstPeak = d_magnitude_grid[i][tmp_intex_t];
                    index_doppler = i;
                    index_time = tmp_intex_t;
                }
        }
    indext = index_time;

    if (!d_step_two)
        {
            doppler = -static_cast<int32_t>(doppler_max) + doppler_step * static_cast<int32_t>(index_doppler);
        }
    else
        {
            doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * acq_parameters.doppler_step2);
        }

    // Find 1 chip wide code phase exclude range around the peak
    int32_t excludeRangeIndex1 = index_time - d_samplesPerChip;
    int32_t excludeRangeIndex2 = index_time + d_samplesPerChip;

    // Correct code phase exclude range if the range includes array boundaries
    if (excludeRangeIndex1 < 0)
        {
            excludeRangeIndex1 = d_fft_size + excludeRangeIndex1;
        }
    else if (excludeRangeIndex2 >= static_cast<int32_t>(d_fft_size))
        {
            excludeRangeIndex2 = excludeRangeIndex2 - d_fft_size;
        }

    int32_t idx = excludeRangeIndex1;
    memcpy(d_tmp_buffer, d_magnitude_grid[index_doppler], d_fft_size);
    do
        {
            d_tmp_buffer[idx] = 0.0;
            idx++;
            if (idx == static_cast<int32_t>(d_fft_size))
                {
                    idx = 0;
                }
        }
    while (idx != excludeRangeIndex2);

    // Find the second highest correlation peak in the same freq. bin ---
    volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_tmp_buffer, d_fft_size);
    float secondPeak = d_tmp_buffer[tmp_intex_t];

    // Compute the test statistics and compare to the threshold
    return firstPeak / secondPeak;
}


void pcps_sd_acquisition::acquisition_core(uint64_t samp_count)
{
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    gr::thread::scoped_lock lk(d_setlock);

    // Initialize acquisition algorithm
    int32_t doppler = 0;
    uint32_t indext = 0U;
    int effective_fft_size = (uint16_t)(acq_parameters.bit_transition_flag ? d_fft_size / 2 : d_fft_size);
    if (d_cshort)
    {
        volk_gnsssdr_16ic_convert_32fc(d_data_buffer, d_data_buffer_sc, d_consumed_samples);
    }
    //LOG(ERROR) << "HERE";
    pcps_sd_acquisition::Peak peak;

    if(d_gnss_synchro->PRN == 98)
    {
        d_state = 0;  // Negative acquisition
        d_step_two = false;
        send_negative_acquisition();
        d_worker_active = false;
    }
    else
    {
        
    }
    memcpy(d_input_signal, d_data_buffer, d_consumed_samples * sizeof(gr_complex));
    //LOG(ERROR) << "HERE";
    if (d_fft_size > d_consumed_samples)
    {
        for (uint32_t i = d_consumed_samples; i < d_fft_size; i++)
            {
                for (uint32_t i = d_consumed_samples; i < d_fft_size; i++)
                    {
                        d_input_signal[i] = gr_complex(0.0, 0.0);
                    }
            }
        const gr_complex* in = d_input_signal;  // Get the input samples pointer
        float magt = 0.0;
        float fft_normalization_factor = static_cast<float>(d_fft_size) * static_cast<float>(d_fft_size);

        d_input_power = 0.0;
        d_mag = 0.0;

        d_num_noncoherent_integrations_counter++;
        // d_sample_counter = samp_count; // sample counter

        //spoofing
        bool acquire_auxiliary_peaks = false;
        //acq_parameters.spoofing_detection = false;
        if(d_peak != 0)  
            {
                DLOG(INFO) << "acquire aux";
                acquire_auxiliary_peaks = true;
                //d_peak = d_peak + 1;
            }
        std::vector<float> peaks;

        std::map<float, pcps_sd_acquisition::Peak> d_highest_peaks;

        DLOG(INFO) << "Channel: " << d_channel
                << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
                << " ,sample stamp: " << samp_count << ", threshold: "
                << d_threshold << ", doppler_max: " << acq_parameters.doppler_max
                << ", doppler_step: " << d_doppler_step
                << ", use_CFAR_algorithm_flag: " << (d_use_CFAR_algorithm_flag ? "true" : "false");

        lk.unlock();

    if (d_use_CFAR_algorithm_flag or acq_parameters.bit_transition_flag)
    {
        // Compute the input signal power estimation
        volk_32fc_magnitude_squared_32f(d_tmp_buffer, in, d_fft_size);
        volk_32f_accumulator_s32f(&d_input_power, d_tmp_buffer, d_fft_size);
        d_input_power /= static_cast<float>(d_fft_size);
    }
    //LOG(ERROR) << "HERE";
    // Doppler frequency grid loop
    if (!d_step_two)
        {
            //spoofing
            bool acquire_auxiliary_peaks = false;
            if(d_peak != 0)  
                {
                    LOG(INFO) << "================= ALL QUALIFYING PEAKS ================= " << threshold_spoofing;
                }

                // 2- Doppler frequency search loop
                for (unsigned int doppler_index = 0; doppler_index < d_num_doppler_bins; doppler_index++)
                    {
                        // doppler search steps
                        doppler = -static_cast<int>(acq_parameters.doppler_max) + d_doppler_step * doppler_index;

                        volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in,
                                d_grid_doppler_wipeoffs[doppler_index], d_fft_size);

                        // 3- Perform the FFT-based convolution  (parallel time search)
                        // Compute the FFT of the carrier wiped--off incoming signal
                        d_fft_if->execute();

                        // Multiply carrier wiped--off, Fourier transformed incoming signal
                        // with the local FFT'd code reference using SIMD operations with VOLK library
                        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(),
                                d_fft_if->get_outbuf(), d_fft_codes, d_fft_size);

                        // compute the inverse FFT
                        d_ifft->execute();

                        // Search maximum
                        size_t offset = ( acq_parameters.bit_transition_flag ? effective_fft_size : 0 );
                        volk_32fc_magnitude_squared_32f(d_magnitude, d_ifft->get_outbuf() + offset, effective_fft_size);
                        volk_32f_index_max_32u(&indext, d_magnitude, effective_fft_size);
                        magt = d_magnitude[indext];

                        if (d_use_CFAR_algorithm_flag == true)
                        {
                            // Normalize the maximum value to correct the scale factor introduced by FFTW
                            magt = d_magnitude[indext] / (fft_normalization_factor * fft_normalization_factor);
                        }

                        if(acquire_auxiliary_peaks)
                        {    
                            for(unsigned int i = 0; i < d_fft_size; i++)
                            {
                                peaks.push_back(d_magnitude[i]);
                            }
                            //Find the local maxima for the peaks for each doppler bin
                            p1d::Persistence1D p;
                            std::vector<float> dp; 
                            if(*std::max_element(peaks.begin(), peaks.end()) >= threshold_spoofing) 
                                {
                                    p.RunPersistence(peaks);
                                    std::vector <p1d::TPairedExtrema> Extrema;
                                    p.GetPairedExtrema(Extrema, 0);

                //if (d_dump and d_channel == d_dump_channel)
                //{
                //LOG(ERROR) << "HERE";
                //memcpy(grid_.colptr(doppler_index), &d_magnitude[doppler_index], sizeof(float) * effective_fft_size);
                //LOG(ERROR) << "HERE";
                //}sss

                        // 4- record the maximum peak and the associated synchronization parameters
                        if (d_mag < magt)
                            {
                                d_mag = magt;

                                if (d_use_CFAR_algorithm_flag == false)
                                    {
                                        // Search grid noise floor approximation for this doppler line
                                        volk_32f_accumulator_s32f(&d_input_power, d_magnitude, effective_fft_size);
                                        d_input_power = (d_input_power - d_mag) / (effective_fft_size - 1);
                                    }

                                // In case that d_bit_transition_flag = true, we compare the potentially
                                // new maximum test statistics (d_mag/d_input_power) with the value in
                                // d_test_statistics. When the second dwell is being processed, the value
                                // of d_mag/d_input_power could be lower than d_test_statistics (i.e,
                                // the maximum test statistics in the previous dwell is greater than
                                // current d_mag/d_input_power). Note that d_test_statistics is not
                                // restarted between consecutive dwells in multidwell operation.

                                if (d_test_statistics < (d_mag / d_input_power) || !acq_parameters.bit_transition_flag)
                                    {
                                        d_gnss_synchro->Acq_delay_samples = static_cast<double>(indext % (int)acq_parameters.samples_per_code);
                                        d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                                        d_gnss_synchro->Acq_samplestamp_samples = d_sample_counter;

                                        // 5- Compute the test statistics and compare to the threshold
                                        //d_test_statistics = 2 * d_fft_size * d_mag / d_input_power;
                                        d_test_statistics = d_mag / d_input_power;
                                    }
                            }

                        // Record results to file if required
                        /*
                        if (d_dump)
                            {
                                std::stringstream filename;
                                std::streamsize n = 2 * sizeof(float) * (d_fft_size); // complex file write
                                filename.str("");

                                boost::filesystem::path p = d_dump_filename;
                                filename << p.parent_path().string()
                                        << boost::filesystem::path::preferred_separator
                                        << p.stem().string()
                                        << "_" << d_gnss_synchro->System
                                        <<"_" << d_gnss_synchro->Signal << "_sat_"
                                        << d_gnss_synchro->PRN << "_doppler_"
                                        <<  doppler
                                        << p.extension().string();

                                DLOG(INFO) << "Writing ACQ out to " << filename.str();

                                d_dump_file.open(filename.str().c_str(), std::ios::out | std::ios::binary);
                                d_dump_file.write((char*)d_ifft->get_outbuf(), n); //write directly |abs(x)|^2 in this Doppler bin?
                                d_dump_file.close();
                            }
                        */
                    }

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
                            DLOG(INFO) << " " << rit->second.code_phase << " " << rit->second.doppler <<" "<< rit->second.mag;
                            for (rit2=d_highest_peaks_reduced.rbegin(); rit2!=d_highest_peaks_reduced.rend(); ++rit2)
                            {
                                if(abs(rit->second.code_phase - rit2->second.code_phase) <= 1)
                                    {
                                        use_peak = false;
                                    }

                            } 
                            if(use_peak)
                                {
                                    d_highest_peaks_reduced[rit->first] = rit->second;
                                }
                        }
                        
                        LOG(INFO) << "### ### Size " << d_highest_peaks_reduced.size() << "; d_peak: " << d_peak;
                        for (rit=d_highest_peaks_reduced.rbegin(); rit!=d_highest_peaks_reduced.rend(); ++rit)
                        {
                            LOG(INFO) << "Channel: " << d_gnss_synchro->Channel_ID << " PRN: " << d_gnss_synchro->PRN << " Peak: " << rit->second.mag << "; Code phase " << rit->second.code_phase << "; Doppler " << rit->second.doppler << "; Test statistics " << rit->second.test_stats;
                        }
                        LOG(INFO) << "### ###";

                        //If there is more than one peak present, acquire the highest
                        if(d_peak == 1 && d_highest_peaks_reduced.size() > 0)
                        {
                            found_peak = true;
                        }
                        else
                        {
                            std::map<float, Peak>::reverse_iterator rit;

                            unsigned int i = 1;
                            DLOG(INFO) << "### peaks: ###";
                            for (rit=d_highest_peaks_reduced.rbegin(); rit!=d_highest_peaks_reduced.rend(); ++rit)
                            {
                                if(i == d_peak)
                                {
                                    found_peak = true; 
                                    LOG(INFO) << "!!! peak found !!!";
                                    LOG(INFO) << "peak " << rit->first; 
                                    LOG(INFO) << "d_peak " << d_peak; 
                                    LOG(INFO) << "code phase " << rit->second.code_phase; 
                                    d_test_statistics = rit->first/ d_input_power; 
                                    d_gnss_synchro->Acq_delay_samples = rit->second.code_phase; 
                                    d_gnss_synchro->Acq_doppler_hz = rit->second.doppler; 
                                }
                                ++i;
                            }
                        }
                
                    }
                    peaks.clear();
                }
                //LOG(ERROR) << "HERE";
                // 4- record the maximum peak and the associated synchronization parameters
                if (d_mag < magt)
                {
                    //found_peak = false; 
                    //d_test_statistics = 0;
                }

                DLOG(INFO) << "found peak: " << found_peak << " aux " << acquire_auxiliary_peaks ;
            }
        else
            {
                // Compute the test statistic
                if (d_use_CFAR_algorithm_flag)
                    {
                        //d_test_statistics = max_to_input_power_statistic(indext, doppler, d_input_power, d_num_doppler_bins_step2, static_cast<int32_t>(d_doppler_center_step_two - (static_cast<float>(d_num_doppler_bins_step2) / 2.0) * acq_parameters.doppler_step2), acq_parameters.doppler_step2);
                    }
                else
                    {
                        d_test_statistics = first_vs_second_peak_statistic(indext, doppler, d_num_doppler_bins_step2, static_cast<int32_t>(d_doppler_center_step_two - (static_cast<float>(d_num_doppler_bins_step2) / 2.0) * acq_parameters.doppler_step2), acq_parameters.doppler_step2);
                    }

                if (acq_parameters.use_automatic_resampler)
                    {
                        //take into account the acquisition resampler ratio
                        d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), acq_parameters.samples_per_code)) * acq_parameters.resampler_ratio;
                        d_gnss_synchro->Acq_delay_samples -= static_cast<double>(acq_parameters.resampler_latency_samples);  //account the resampler filter latency
                        d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                        d_gnss_synchro->Acq_samplestamp_samples = rint(static_cast<double>(samp_count) * acq_parameters.resampler_ratio);
                        d_gnss_synchro->Acq_doppler_step = acq_parameters.doppler_step2;
                    }
                else
                    {
                        d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(indext), acq_parameters.samples_per_code));
                        d_gnss_synchro->Acq_doppler_hz = static_cast<double>(doppler);
                        d_gnss_synchro->Acq_samplestamp_samples = samp_count;
                        d_gnss_synchro->Acq_doppler_step = acq_parameters.doppler_step2;
                    }
            }
            //LOG(ERROR) << "HERE";
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
                        d_active = false;
                        if (acq_parameters.make_2_steps)
                            {
                                if (d_step_two)
                                    {
                                        send_positive_acquisition();
                                        d_step_two = false;
                                        d_state = 0;  // Positive acquisition
                                    }
                                else
                                    {
                                        d_step_two = true;  // Clear input buffer and make small grid acquisition
                                        d_num_noncoherent_integrations_counter = 0;
                                        d_positive_acq = 0;
                                        d_state = 0;
                                    }
                            }
                        else
                            {
                                send_positive_acquisition();
                                d_state = 0;  // Positive acquisition
                            }
                    }
                else
                    {
                        d_buffer_count = 0;
                        d_state = 1;
                    }

                if (d_num_noncoherent_integrations_counter == acq_parameters.max_dwells)
                    {
                        if (d_state != 0)
                            {
                                send_negative_acquisition();
                            }
                        d_state = 0;
                        d_active = false;
                        d_step_two = false;
                    }
            }
        else
            {
                d_active = false;
                //LOG(ERROR) << "Test: " << d_highest_peaks.end()->second.test_stats << "; Threshold " << d_threshold;
                if (d_test_statistics > d_threshold)
                    {
                        if (acq_parameters.make_2_steps)
                            {
                                if (d_step_two)
                                    {
                                        send_positive_acquisition();
                                        d_step_two = false;
                                        d_state = 0;  // Positive acquisition
                                    }
                                else
                                    {
                                        d_step_two = true;  // Clear input buffer and make small grid acquisition
                                        d_num_noncoherent_integrations_counter = 0U;
                                        d_state = 0;
                                    }
                            }
                        else
                            {
                                send_positive_acquisition();
                                d_state = 0;  // Positive acquisition
                            }
                    }
                else
                    {
                        d_state = 0;  // Negative acquisition
                        d_step_two = false;
                        send_negative_acquisition();
                    }
            }
        d_worker_active = false;

        if ((d_num_noncoherent_integrations_counter == acq_parameters.max_dwells) or (d_positive_acq == 1))
        {
            // Record results to file if required
            if (d_dump and d_channel == d_dump_channel)
                {
                    pcps_sd_acquisition::dump_results(effective_fft_size);
                }
            d_num_noncoherent_integrations_counter = 0U;
            d_positive_acq = 0;
            // Reset grid
            for (uint32_t i = 0; i < d_num_doppler_bins; i++)
            {
                for (uint32_t k = 0; k < d_fft_size; k++)
                {
                    d_magnitude_grid[i][k] = 0.0;
                }
            }
        }
}


// Called by gnuradio to enable drivers, etc for i/o devices.
bool pcps_sd_acquisition::start()
{
    d_sample_counter = 0ULL;
    return true;
}


int pcps_sd_acquisition::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{
    /*
     * By J.Arribas, L.Esteve and M.Molina
     * Acquisition strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. Record the maximum peak and the associated synchronization parameters
     * 5. Compute the test statistics and compare to the threshold
     * 6. Declare positive or negative acquisition using a message port
     */
    d_peak = d_gnss_synchro->peak;
    gr::thread::scoped_lock lk(d_setlock);
    if (!d_active or d_worker_active)
        {
            if (!acq_parameters.blocking_on_standby)
                {
                    d_sample_counter += static_cast<uint64_t>(ninput_items[0]);
                    consume_each(ninput_items[0]);
                }
            if (d_step_two)
                {
                    d_doppler_center_step_two = static_cast<float>(d_gnss_synchro->Acq_doppler_hz);
                    update_grid_doppler_wipeoffs_step2();
                    d_state = 0;
                    d_active = true;
                }
            return 0;
        }
    //LOG(ERROR) << "HERE";
    switch (d_state)
        {
        case 0:
            {
                // Restart acquisition variables
                d_gnss_synchro->Acq_delay_samples = 0.0;
                d_gnss_synchro->Acq_doppler_hz = 0.0;
                d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
                d_gnss_synchro->Acq_doppler_step = 0U;
                d_mag = 0.0;
                d_input_power = 0.0;
                d_test_statistics = 0.0;
                d_state = 1;
                d_buffer_count = 0U;
                if (!acq_parameters.blocking_on_standby)
                    {
                        d_sample_counter += static_cast<uint64_t>(ninput_items[0]);  // sample counter
                        consume_each(ninput_items[0]);
                    }
                break;
            }
        case 1:
            {
                uint32_t buff_increment;
                if (d_cshort)
                    {
                        const auto* in = reinterpret_cast<const lv_16sc_t*>(input_items[0]);  // Get the input samples pointer
                        if ((ninput_items[0] + d_buffer_count) <= d_consumed_samples)
                            {
                                buff_increment = ninput_items[0];
                            }
                        else
                            {
                                buff_increment = d_consumed_samples - d_buffer_count;
                            }
                        memcpy(&d_data_buffer_sc[d_buffer_count], in, sizeof(lv_16sc_t) * buff_increment);
                    }
                else
                    {
                        const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);  // Get the input samples pointer
                        if ((ninput_items[0] + d_buffer_count) <= d_consumed_samples)
                            {
                                buff_increment = ninput_items[0];
                            }
                        else
                            {
                                buff_increment = d_consumed_samples - d_buffer_count;
                            }
                        memcpy(&d_data_buffer[d_buffer_count], in, sizeof(gr_complex) * buff_increment);
                    }

                // If buffer will be full in next iteration
                if (d_buffer_count >= d_consumed_samples)
                    {
                        d_state = 2;
                    }
                d_buffer_count += buff_increment;
                d_sample_counter += static_cast<uint64_t>(buff_increment);
                consume_each(buff_increment);
                break;
            }
        case 2:
            {
                // Copy the data to the core and let it know that new data is available
                if (acq_parameters.blocking)
                    {
                        lk.unlock();
                        //LOG(ERROR) << "HERE";
                        acquisition_core(d_sample_counter);
                    }
                else
                    {
                        gr::thread::thread d_worker(&pcps_sd_acquisition::acquisition_core, this, d_sample_counter);
                        d_worker_active = true;
                    }
                consume_each(0);
                d_buffer_count = 0U;
                //LOG(ERROR) << "HERE";
                break;
            }
        }
    //LOG(ERROR) << "HERE";
    return 0;
}
