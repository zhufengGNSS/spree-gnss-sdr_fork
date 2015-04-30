function [r] = read_flog(filename, count)

  num_float_vars = 6;
  num_unsigned_long_int_vars = 1;
  num_double_vars = 2;
  double_size_bytes = 8;
  unsigned_long_int_size_bytes = 8;
  float_size_bytes = 4;
  skip_bytes_each_read = float_size_bytes*num_float_vars+ ...
                       unsigned_long_int_size_bytes*num_unsigned_long_int_vars+...
                       double_size_bytes*num_double_vars;
  bytes_shift = 0;
  
  if (nargin < 2)
    count = Inf;
  end
  
  f = fopen (filename, 'rb');
  
  if (f < 0)
    disp 'File not found';
  else
      
    PRN = fread (f, count, 'int',skip_bytes_each_read - float_size_bytes);
    bytes_shift = bytes_shift + float_size_bytes;
    
    fseek(f,bytes_shift,'bof'); % move to next interleaved float
    sample_counter = fread (f, count, 'uint64',skip_bytes_each_read-unsigned_long_int_size_bytes);
    bytes_shift=bytes_shift+unsigned_long_int_size_bytes;
    
%     fseek(f,bytes_shift,'bof'); % move to next interleaved float
%     timestamp = fread (f, count, 'double',skip_bytes_each_read-double_size_bytes);
%     bytes_shift=bytes_shift+double_size_bytes;
    
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
%     r.timestamp = timestamp;
    r.SNR=SNR;
    r.Doppler=doppler;
    r.delta = delta;
    r.RT=RT;
    r.Extra_RT = Extra_RT;
    r.ELP = ELP;
    r.MD = MD;
  end
  
