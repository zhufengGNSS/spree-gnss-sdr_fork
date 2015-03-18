/*!
 * \file tracking_2nd_ALL_filter.cc
 * \brief Implementation of a 2nd order ALL filter for amplitude tracking loop.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Class that implements 2 order PLL filter for amplitude tracking loop.
 * The algorithm is described in :
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.~H.~Jensen, A Software-Defined
 * GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007, Applied and Numerical Harmonic Analysis.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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


#include "tracking_2nd_ALL_filter.h"


void Tracking_2nd_ALL_filter::calculate_lopp_coef(float* tau1,float* tau2, float lbw, float zeta, float k)
{
    // Solve natural frequency
    float Wn;
    Wn = lbw*8*zeta / (4*zeta*zeta + 1);
    // solve for t1 & t2
    *tau1 = k / (Wn * Wn);
    *tau2 = (2.0 * zeta) / Wn;
}



void Tracking_2nd_ALL_filter::set_ALL_BW(float all_bw_hz)
{
    //Calculate filter coefficient values
    d_allnoisebandwidth  = all_bw_hz;
    calculate_lopp_coef(&d_tau1_amplitude, &d_tau2_amplitude, d_allnoisebandwidth, d_alldampingratio, 1.0);// Calculate filter coefficient values
}



void Tracking_2nd_ALL_filter::initialize()
{
    // amplitude tracking loop parameters
    d_old_amplitude_nco   = 0.0;
    d_old_amplitude_error = 0.0;
}



float Tracking_2nd_ALL_filter::get_amplitude_nco(float ALL_discriminator)
{
    float amplitude_nco;
    amplitude_nco = d_old_amplitude_nco+ (d_tau2_amplitude/d_tau1_amplitude)*(ALL_discriminator - d_old_amplitude_error) + (ALL_discriminator+d_old_amplitude_error) * (d_pdi_amplitude/(2*d_tau1_amplitude));
    d_old_amplitude_nco   = amplitude_nco;
    d_old_amplitude_error = ALL_discriminator; //[chips]
    return amplitude_nco;
}

Tracking_2nd_ALL_filter::Tracking_2nd_ALL_filter (float pdi_amplitude)
{
    d_pdi_amplitude = pdi_amplitude;// Summation interval for amplitude 
    d_alldampingratio = 0.7;
}

Tracking_2nd_ALL_filter::Tracking_2nd_ALL_filter ()
{
    d_pdi_amplitude = 0.001;// Summation interval for amplitude 
    d_alldampingratio = 0.7;
}

Tracking_2nd_ALL_filter::~Tracking_2nd_ALL_filter ()
{}

void Tracking_2nd_ALL_filter::set_pdi(float pdi_amplitude)
{
    d_pdi_amplitude = pdi_amplitude; // Summation interval for amplitude 
}
