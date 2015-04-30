% /*!
%  * \file gps_l1_ca_dll_pll_read_tracking_dump.m
%  * \brief Read GNSS-SDR Tracking dump binary file into MATLAB.
%  * \author Javier Arribas, 2011. jarribas(at)cttc.es
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
function [r] = read_flog(filename, count)

  %% usage: gps_l1_ca_dll_pll_read_tracking_dump_64bits (filename, [count])
  %%
  %% open GNSS-SDR tracking binary log file .dat and return the contents
  %%

  %m = nargchk (1,2,nargin);
  num_float_vars=6;
  num_unsigned_long_int_vars=1;
  num_double_vars=3;
  double_size_bytes=8;
  unsigned_long_int_size_bytes=8;
  float_size_bytes=4;
  skip_bytes_each_read=float_size_bytes*num_float_vars+unsigned_long_int_size_bytes*num_unsigned_long_int_vars+double_size_bytes*num_double_vars;
  bytes_shift=0;
  %if (m)
  %  usage (m);
  %end

  if (nargin < 2)
    count = Inf;
  end
    %loops_counter = fread (f, count, 'uint32',4*12);
  f = fopen (filename, 'rb');
  if (f < 0)
  else
    PRN = fread (f, count, 'int',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    sample_counter = fread (f, count, 'uint64',skip_bytes_each_read-unsigned_long_int_size_bytes);
        bytes_shift=bytes_shift+unsigned_long_int_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    timestamp = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    SNR = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    doppler = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
        bytes_shift=bytes_shift+double_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    delta = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    RT = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    Extra_RT = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    ELP = fread (f, count, 'float',skip_bytes_each_read-float_size_bytes);
        bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    MD = fread (f, count, 'uint64',skip_bytes_each_read-unsigned_long_int_size_bytes);
    fclose (f);
    
    r.PRN=PRN;
    r.sample_counter = sample_counter;
    r.timestamp = timestamp;
    r.SNR=SNR;
    r.Doppler=doppler;
    r.delta = delta;
    r.RT=RT;
    r.Extra_RT = Extra_RT;
    r.ELP = ELP;
    r.MD = MD;
  end
  
