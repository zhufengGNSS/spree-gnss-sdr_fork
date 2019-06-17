function [r]  = plot_SNR(path, channels, count_)
    x = 'PRN_start_sample'; 
    p = 'CN0_SNV_dB_Hz';

    j = 1;
    count = count_;
    length(channels)
    min_count = inf;
    for i = channels
        filename = strcat(path,'epl_tracking_ch_', num2str(i), '.dat')
        if exist(filename, 'file')
            if(count == 0) 
                s = dir(filename);
                count = s.bytes;
            end
            r(j) = gps_l1_ca_dll_pll_read_tracking_dump_64bits(filename, count);
            if( min_count > length(r(j).(p)))
                min_count = length(r(j).(p));
            end
            j = j+1;
        else
            strcat('File ', filename, 'doesn not exists');
        end
        count = count_;
    end
    
    cc= jet(32);
    cc = lines(j);
    y = [];
    figure(1)
    for i = 1:j-1
        h(i) = plot(r(i).(x), r(i).(p)); 
        set(h(i), 'Color', cc(i,:)) 
        hold on
        xlabel('time [s]')
        ylabel('SNR')
        title('CN0_SNV_dB_Hz');
        legend(h, num2str(channels'))
    end

end
