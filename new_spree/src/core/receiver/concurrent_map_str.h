/*!
 * \file concurrent_map.h
 * \brief Interface of a thread-safe std::map
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_CONCURRENT_MAP_STR_H
#define GNSS_SDR_CONCURRENT_MAP_STR_H

#include <map>
#include <string>
#include <utility>
#include <boost/thread/mutex.hpp>

template<typename Data>


/*!
 * \brief This class implements a thread-safe std::map
 *
 */
class concurrent_map_str
{
    typedef typename std::map<std::string,Data>::iterator Data_iterator; // iterator is scope dependent
private:
    std::map<std::string,Data> the_map;
    boost::mutex the_mutex;
public:
    void write(std::string key, Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        Data_iterator data_iter;
        data_iter = the_map.find(key);
        if (data_iter != the_map.end())
            {
                data_iter->second = data; // update
            }
        else
            {
                the_map.insert(std::pair<std::string, Data>(key, data)); // insert SILENTLY fails if the item already exists in the map!
            }
        lock.unlock();
    }

    void add(std::string key, Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_map[key] = data;
        lock.unlock();
    }

    void remove(std::string key)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_map.erase(key);
        lock.unlock();
    }

    std::map<std::string ,Data> get_map_copy()
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_map;
        lock.unlock();
    }

    int size()
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_map.size();
        lock.unlock();
    }

    bool read(std::string key, Data& p_data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        Data_iterator data_iter;
        data_iter = the_map.find(key);
        if (data_iter != the_map.end())
            {
                p_data = data_iter->second;
                lock.unlock();
                return true;
            }
        else
            {
                lock.unlock();
                return false;
            }
    }
};

#endif
