function plot_vestigial(path, channels, count)

    j = 1;
    k = 1;

    for i = channels
        filename = strcat(path,'vsd_', num2str(i), '.dat')
        if exist(filename, 'file')
            if(count == 0) 
                s = dir(filename);
                s
                s.bytes
                count = s.bytes;
                count
            end
            r(j) = gps_l1_ca_read_vestigial(filename, count);
            j = j+1;
        end
        count = 0;
    end

    k =fieldnames(r)
    for i = 1:length(channels) 
        figure(i+1)
        for j = 1:length(k)    
            rf =  getfield(r(i), k{j});
            rf = rf( abs(rf) < 1);

            subplot(1,length(k), j) 
            hist(rf, 1e3)
            title(k{j})
            
        end
    end
end
