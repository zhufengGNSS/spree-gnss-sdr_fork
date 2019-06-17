function [x,y] = get_PRN_trace(r)
    PRNs = unique(r.PRN);
    for i = 1:length(PRNs)
        x(i) = r.PRN_start_sample( r.PRN == PRNs(i))
        y(i) = r.CN0_SNV_dB_Hz( r.PRN == PRNs(i))
    end
end
