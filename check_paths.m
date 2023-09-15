function flag = check_paths(fplist, ob, MAX_ACCEL, MAX_SPEED, MAX_CURVATURE,ROBOT_RADIUS)
    flag = 1;
    for i=1:1:size(fplist.s_d ,2)
        if (fplist.s_d(1,i) > MAX_SPEED)
            flag = 0;
            break
        end
        
        if (fplist.s_dd(1,i) > MAX_ACCEL)
            flag = 0;
            break
        end
        
        if (fplist.c(1,i) > MAX_CURVATURE)
            flag = 0;
            break
        end
    end
    
    if (flag == 1)
        for i=1:1:size(fplist.x,2)
            for j=1:1:size(ob)
                d = sqrt((ob(j,1)-fplist.x(1,i))^2 + (ob(j,2)-fplist.y(1,i))^2);
                if(d <= ROBOT_RADIUS)
                    flag = 0;
                    break
                end
            end
            if (flag == 0)
                break
            end
            if (flag == 0)
                break
            end
        end
    end
end