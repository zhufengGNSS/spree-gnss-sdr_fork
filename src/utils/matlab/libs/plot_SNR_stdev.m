function [r]  = plot_SNR_stdev(path, count_, ws)

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

    mv_avg = tsmovavg(r.stdev','s', ws); 

    figure(1)
    plot(r.sample/1e3, mv_avg); 
    hold on
    plot(r.sample/1e3, 2*ones(1,length(r.sample)), 'r')
    xlabel('time [s]')
    ylabel('stdDev SNR')
    title('CN0\_SNV\_dB0\_Hz');

end
