function [r]  = plot_SNR_stdev(path, count_, ws)
    count_
    count = count_;
    filename = strcat(path,'.dat')
    if exist(filename, 'file')
        if(count == 0) 
            s = dir(filename);
            count = s.bytes;
        end
        r = read_SNR(filename, count);
    else
        strcat('File ', filename, 'doesn not exists');
    end

    mv_avg = tsmovavg(r.stdev,'s', ws) 
    
    figure(1)
    plot(r.sample, m_avg); 
    xlabel('time [s]')
    ylabel('stdDev SNR')
    title('CN0_SNV_dB_Hz');

end
