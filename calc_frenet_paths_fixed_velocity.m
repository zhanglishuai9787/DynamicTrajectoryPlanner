function fplist = calc_frenet_paths_fixed_velocity(csp, c_speed, c_d, c_d_d, c_d_dd, ...
                               s0,MAX_ROAD_WIDTH,D_ROAD_W, MIN_T, MAX_T, DT, ...
                               FrenetPath, TARGET_SPEED,D_T_S, N_S_SAMPLE, ...
                               K_D, K_J, K_LAT, K_LON, K_T, ob, MAX_ACCEL, ...
                               MAX_SPEED, MAX_CURVATURE,ROBOT_RADIUS);
    min_cost = [inf,inf];
    cost = [];
    flag = 1;
    for di=-MAX_ROAD_WIDTH:D_ROAD_W:MAX_ROAD_WIDTH
        for Ti = MIN_T:DT:MAX_T
            fp = FrenetPath;
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0, 0, Ti);
            
            for t=0:DT:Ti-DT
                fp.t(1,end+1) = t;
            end
            
            for t=fp.t(1,1):DT:fp.t(1,end)
                fp.d(1,end+1)     = lat_qp.a0   + lat_qp.a1*t    + lat_qp.a2*t^2    + lat_qp.a3*t^3   + lat_qp.a4*t^4  + lat_qp.a5*t^5;
                fp.d_d(1,end+1)   = lat_qp.a1   + 2*lat_qp.a2*t  + 3*lat_qp.a3*t^2  + 4*lat_qp.a4*t^3 + 5*lat_qp.a5*t^4;
                fp.d_dd(1,end+1)  = 2*lat_qp.a2 + 6*lat_qp.a3*t  + 12*lat_qp.a4*t^2 + 20*lat_qp.a5*t^3;
                fp.d_ddd(1,end+1) = 6*lat_qp.a3 + 24*lat_qp.a4*t + 60*lat_qp.a5*t^2;
            end
            
            for tv = TARGET_SPEED - D_T_S * N_S_SAMPLE : D_T_S: TARGET_SPEED + D_T_S * N_S_SAMPLE
                tfp = fp;
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti);
                
                for t=fp.t(1,1):DT:fp.t(1,end)
                    tfp.s(1,end+1)     = lon_qp.b0   + lon_qp.b1*t   + lon_qp.b2*t^2   + lon_qp.b3*t^3  + lon_qp.b4*t^4;
                    tfp.s_d(1,end+1)   = lon_qp.b1   + 2*lon_qp.b2*t + 3*lon_qp.b3*t^2 + 4*lon_qp.b4*t^3;
                    tfp.s_dd(1,end+1)  = 2*lon_qp.b2 + 6*lon_qp.b3*t + 12*lon_qp.b4*t^2 ;
                    tfp.s_ddd(1,end+1) = 6*lon_qp.b3 + 24*lon_qp.b4*t;
                end
                Jp = sum(tfp.d_ddd .^2);
                Js = sum(tfp.s_ddd .^2);
                
                ds = (TARGET_SPEED - tfp.s_d(1,end))^ 2;
                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d(1,end)^2;
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds;
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv;
                
                tfp = calc_global_paths(tfp, csp);
                flag = check_paths(tfp, ob, MAX_ACCEL, MAX_SPEED, MAX_CURVATURE,ROBOT_RADIUS);  
 
                if (flag == 1)
                    cost(end+1,:) = [tfp.cf, di];
                    if min_cost(end,1) >= tfp.cf
                        min_cost(end+1,:) = [tfp.cf, di];
                        fplist = tfp;
                    end
                else
                    flag = 1;
                end
                
            end
        end
    end
end