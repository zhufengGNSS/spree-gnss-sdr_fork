
    //LOG(ERROR) << "HERE";
    if (d_fft_size > d_consumed_samples)
    {
        for (uint32_t i = d_consumed_samples; i < d_fft_size; i++)
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
                //if (d_dump and d_channel == d_dump_channel)
                //{
                //LOG(ERROR) << "HERE";
                //memcpy(grid_.colptr(doppler_index), &d_magnitude[doppler_index], sizeof(float) * effective_fft_size);
                //LOG(ERROR) << "HERE";
                //}sss
                    peaks.clear();
                }
                //LOG(ERROR) << "HERE";
                // 4- record the maximum peak and the associated synchronization parameters
                if (d_mag < magt)
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
                consume_each(0);
                d_buffer_count = 0U;
                //LOG(ERROR) << "HERE";
                break;
            }
        }
    //LOG(ERROR) << "HERE";
    return 0;
}
