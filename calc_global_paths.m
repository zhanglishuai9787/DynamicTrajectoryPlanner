function fplist = calc_global_paths(fplist, csp)
    for i = 1:1:size(fplist.s,2)
        [idx] = findPoint(fplist.s(1,i), csp);
        i_x = csp(idx,1);
        i_y = csp(idx,2);
        i_yaw = (csp(idx+1,2) - csp(idx,2))/(csp(idx+1,1) - csp(idx,1));
        di = fplist.d(1,i);
        fplist.x(1,end+1) = i_x - di * sin(i_yaw);
        fplist.y(1,end+1) = i_y + di * cos(i_yaw);
    end
    
    for i = 1:1:size(fplist.x,2)-1
        dx = fplist.x(1,i+1) - fplist.x(1,i);
        dy = fplist.y(1,i+1) - fplist.y(1,i);
        fplist.yaw(1,end+1) = atan(dy/dx);
        fplist.ds(1,end+1) = sqrt(dx^1 + dy^2);
    end
    
    fplist.yaw(1,end+1) = fplist.yaw(1,end);
    fplist.ds(1,end+1) = fplist.ds(1,end);
    
    for i=1:1:size(fplist.yaw,2)-1
        fplist.c(1,end+1) = (fplist.yaw(1,i+1) - fplist.yaw(1,i+1))/fplist.ds(1,i);
    end
    fplist.c(1,end+1) = fplist.c(1,end);
end