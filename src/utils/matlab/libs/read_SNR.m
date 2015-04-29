function [GNSS_tracking] = read_SNR(filename, count)

  m = nargchk (1,2,nargin);
  num_float_vars=0;
  num_unsigned_long_int_vars=1;
  num_double_vars=1;
  double_size_bytes=8;
  unsigned_long_int_size_bytes=8;
  float_size_bytes=4;
  skip_bytes_each_read=float_size_bytes*num_float_vars+unsigned_long_int_size_bytes*num_unsigned_long_int_vars+double_size_bytes*num_double_vars;
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
    v1 = fread (f, count, 'uint64',skip_bytes_each_read-unsigned_long_int_size_bytes);
        bytes_shift=bytes_shift+unsigned_long_int_size_bytes;
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    v2 = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
    fclose (f);
    
    GNSS_tracking.sample=v1;
    GNSS_tracking.stdev=v2;
  end
  
