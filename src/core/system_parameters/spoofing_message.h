/*!
 * \file gnss_synchro.h
 * \brief  Interface of the Gnss_Synchro class
 * \author
 *  Hildur Ólafsdóttir, 2014. ohildur(at)gmail.com 
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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
#ifndef GNSS_SDR_SPOOFING_MESSAGE_H_
#define GNSS_SDR_SPOOFING_MESSAGE_H_

#include "spoofing_message.h"

/*!
 * \brief This is the class that contains the information that is shared
 * by the processing blocks.
 */
class  Spoofing_Message
{
public:
    int spoofing_case;
    std::set<unsigned int> satellites;
    std::string description;
    std::string spoofing_report;
};

#endif
