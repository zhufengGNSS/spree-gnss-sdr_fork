/*!
 * \file gps_l1_ca_pcps_acquisition_fine_doppler.h
 * \brief Adapts a PCPS acquisition block with fine Doppler estimation to an AcquisitionInterface for
 *  GPS L1 C/A signals
 * \authors <ul>
 *          <li> Javier Arribas, 2013. jarribas(at)cttc.es
 *          </ul> *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FINE_DOPPLER_H_
#define GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FINE_DOPPLER_H_

#include <string>
#include "gnss_synchro.h"
#include "acquisition_interface.h"
#include "pcps_acquisition_fine_doppler_cc.h"



class ConfigurationInterface;

/*!
 * \brief This class Adapts a PCPS acquisition block with fine Doppler estimation to an AcquisitionInterface for
 *  GPS L1 C/A signals
 */
class GpsL1CaPcpsAcquisitionFineDoppler: public AcquisitionInterface
{
public:
    GpsL1CaPcpsAcquisitionFineDoppler(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GpsL1CaPcpsAcquisitionFineDoppler();

    std::string role()
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler"
     */
    std::string implementation()
    {
        return "GPS_L1_CA_PCPS_Acquisition_Fine_Doppler";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(boost::shared_ptr<gr::top_block> top_block);
    void disconnect(boost::shared_ptr<gr::top_block> top_block);
    boost::shared_ptr<gr::basic_block> get_left_block();
    boost::shared_ptr<gr::basic_block> get_right_block();

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     *  tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);

    /*!
     * \brief Set acquisition channel unique ID
     */
    void set_channel(unsigned int channel);

    /*!
     * \brief Set statistics threshold of PCPS algorithm
     */
    void set_threshold(float threshold);

    /*!
     * \brief Set maximum Doppler off grid search
     */
    void set_doppler_max(unsigned int doppler_max);

    /*!
     * \brief Set Doppler steps for the grid search
     */
    void set_doppler_step(unsigned int doppler_step);

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init();

    void set_local_code();

    /*!
     * \brief Returns the maximum peak of grid search
     */
    signed int mag();

    /*!
     * \brief Restart acquisition algorithm
     */
    void reset();

private:
    pcps_acquisition_fine_doppler_cc_sptr acquisition_cc_;
    size_t item_size_;
    std::string item_type_;
    unsigned int vector_length_;
    unsigned int channel_;
    float threshold_;
    int doppler_max_;
    unsigned int doppler_step_;
    int doppler_min_;
    unsigned int sampled_ms_;
    int max_dwells_;
    long fs_in_;
    long if_;
    bool dump_;
    std::string dump_filename_;
    std::complex<float> * code_;
    Gnss_Synchro * gnss_synchro_;
    unsigned int peak_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif /* GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FINE_DOPPLER_H_ */
