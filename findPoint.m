function idx = findPoint(s,csp)
s_list = csp(:,3);
dis_temp = abs(s_list(1)-s);
for i=2:length(s_list)
    if  abs(s_list(i)-s) > dis_temp
        break
    end
end
idx =i-1;
end