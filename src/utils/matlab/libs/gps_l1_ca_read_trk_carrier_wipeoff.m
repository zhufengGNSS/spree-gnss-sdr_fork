function [GNSS_tracking] = gps_l1_ca_read_trk_carrier_wipeoff(filename, count)

  %% usage: gps_l1_ca_read_trk_carrier_wipeoff(filename, [count])
  %%
  %% open GNSS-SDR tracking binary log file .dat and return the contents
  %%

  m = nargchk (1,2,nargin);
  float_size_bytes=4;
  if (m)
    usage (m);
  end

  if (nargin < 2)
    count = Inf;
  end
    %loops_counter = fread (f, count, 'uint32',4*12);
  f = fopen (filename, 'rb');
  if (f < 0)
    'unable to open file'
  else
    bytes_shift=0;
    I = fread (f, count, 'float', float_size_bytes);
            bytes_shift=bytes_shift+float_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    Q = fread (f, count, 'float',float_size_bytes);
    fclose (f);
  end
    
    GNSS_tracking.in_I = I;
    GNSS_tracking.in_Q = Q;

end 
