close all
path_ns = '/home/hildur/gnss-sdr/data/cleanStatic/'
path_mp100 = '/home/hildur/gnss-sdr/data/mp100/'
s = dir(strcat(path_ns, 'vsd_', num2str(0), '.dat'))

count = s.bytes;
j = 1;
k = 1;

for i = [0:8]
    Filename1  = strcat(path_ns, 'vsd_', num2str(i), '.dat');
    Filename2  = strcat(path_mp100, 'vsda_', num2str(i), '.dat');
    if exist(Filename1, 'file')
        r1(j) = gps_l1_ca_read_vestigial(Filename1, count);
        j = j+1;
    end
    if exist(Filename2, 'file')
        r2(k) = gps_l1_ca_read_vestigial(Filename2, count);
        k = k+1;
    end 
end

%%
k =fieldnames(r1)
for i = [0:8]
    
    figure(i+1)
for j = 1:length(k)    
    subplot(2,5, j) 
    r1_tmp =  getfield(r1(i+1), k{j});
    r1_tmp = r1_tmp(r1_tmp<10 & r1_tmp>-10);
    r2_tmp =  getfield(r2(i+1), k{j});
    r2_tmp = r2_tmp(r2_tmp<10 & r2_tmp>-10);


    hist(r1_tmp,1e2)
    axis([-4 4 0 max(hist(r1_tmp,1e2))])
    title(k{j})
    
    %hist(r1(i+1).RT(1e2:end),1e3)
    subplot(2,5,5+j)
    %hist(r2(i+1).RT(1e2:end),1e3)
    hist(r2_tmp,1e2)
    axis([-4 4 0 max(hist(r2_tmp,1e2))])
    title(k{j})
    
   
end
end