function r  = plot_tracking(path, channels, count_, p, rows, cols)

    j = 1;
    count = count_;
    length(channels)
    for i = channels
        filename = strcat(path,'epl_tracking_ch_', num2str(i), '.dat')
        if exist(filename, 'file')
            if(count == 0) 
                s = dir(filename);
                count = s.bytes;
            end
            r(j) = gps_l1_ca_dll_pll_read_tracking_dump_64bits(filename, count);
            j = j+1;
        end
        count = count_;
    end
    
        
    cc= jet(32);
    for i = 1:j-1
        if( length(r(i).(p)) < length(r(i).PRN))
            r(i).PRN = r(i).PRN(1:length(r(i).(p)));
        end

        %find all PRN that the channel tracked
        PRNs = unique(r(i).PRN)

        if( length(r(i).(p)) < 2)
            continue
        end

        subplot(rows, cols, i)
        k = 1;
        for j = 1:length(PRNs)
            x = r(i).PRN_start_sample( r(i).PRN == PRNs(j));
            y = r(i).(p)( r(i).PRN == PRNs(j));
            if(length(x) > 1e4)
                x = x/16e6;
                h(k) = plot(x,y,'.');
                set(h(k), 'Color', cc(PRNs(j),:)) 
                used_PRNs(k) = PRNs(j);
                k = k+1;
                hold on
            end
        end
        if(k>1)
            xlabel('time [s]')
            ylabel('SNR')
            title(strcat('Channel ', num2str(channels(i))))
            legend(h, num2str(used_PRNs'))
        end
    end
end
