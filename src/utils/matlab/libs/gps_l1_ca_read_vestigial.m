
%  * \file gps_l1_read_vestigial.m
%  * \brief Read parameters used in vestigial signal defense into MATLAB
%  * \author Javier Arribas, 2011. jarribas(at)cttc.es
%  * \author Hildur Ólafsdóttir, 2015. ohildur(at)gmail.com
%  * -------------------------------------------------------------------------
%  *
%  * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
%  *
%  * GNSS-SDR is a software defined Global Navigation
%  *          Satellite Systems receiver
%  *
%  * This file is part of GNSS-SDR.
%  *
%  * GNSS-SDR is free software: you can redistribute it and/or modify
%  * it under the terms of the GNU General Public License as published by
%  * the Free Software Foundation, either version 3 of the License, or
%  * at your option) any later version.
%  *
%  * GNSS-SDR is distributed in the hope that it will be useful,
%  * but WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  * GNU General Public License for more details.
%  *
%  * You should have received a copy of the GNU General Public License
%  * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
%  *
%  * -------------------------------------------------------------------------
%  */             
function [GNSS_tracking] = gps_l1_ca_dll_pll_read_vestigial (filename, count)

  %% usage: gps_l1_ca_dll_pll_read_vestigial (filename, [count])
  %%
  %% open GNSS-SDR vestigial parameter binary log file .dat and return the contents
  %%

  m = nargchk (1,2,nargin);
  num_float_vars=5;
  num_double_vars=0;
  double_size_bytes=8;
  float_size_bytes=4;
  int_size_bytes=4;
  skip_bytes_each_read=float_size_bytes*num_float_vars+double_size_bytes*num_double_vars;
  bytes_shift=0;
  if (m)
    usage (m);
  end

  if (nargin < 2)
    count = Inf;
  end
    %loops_counter = fread (f, count, 'uint32',4*12);
  f = fopen (filename, 'rb');
  if (f < 0)
  else
    v1 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
            bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v2 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
            bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v3 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
            bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v4 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
            bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v5 = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);

    fclose (f);
    
    delta = v1;
    RT = v2;
    Extra_RT = v3;
    ELP = v4;
    MD = v5;
    
    GNSS_tracking.delta=delta;
    GNSS_tracking.RT=RT;
    GNSS_tracking.Extra_RT=Extra_RT;
    GNSS_tracking.ELP=ELP;
    GNSS_tracking.MD=MD;
  end
  
