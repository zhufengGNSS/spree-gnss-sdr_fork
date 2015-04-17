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


#include "integrator.h"

void Integrator::initialize()
{
    // amplitude tracking loop parameters
    old_result = 0.0;
    old_discriminator = 0.0;
}



float Integrator::integrate(float discriminator)
{
    float result;
    float t = 2/T;
    result = old_result*(1-t) + t*discriminator+t*old_discriminator;
    result = result/(t+1);
    old_result = result;
    old_discriminator = discriminator;
    return result;
}


Integrator::Integrator()
{
    T = 0.01;
}

Integrator::~Integrator()
{}

