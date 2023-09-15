SIM_LOOP = 500;
 
MAX_SPEED = 50.0 / 3.6;  % 最大速度 [m/s]
MAX_ACCEL = 2.0;  % 最大加速度 [m/ss]
MAX_CURVATURE = 1.0;  % 最大曲率 [1/m]
MAX_ROAD_WIDTH = 7.0;  % 最大采样横向距离 [m]
D_ROAD_W = 1.0;  % 横向距离采样间隔 [m]
DT = 0.2;  % 采样时间间隔 [s]
MAX_T = 5.0;  % 最大采样时间 [s]
MIN_T = 4.0;  % 最小采样时间 [s]
TARGET_SPEED = 30.0 / 3.6;  % 期望速度 [m/s]
D_T_S = 5.0 / 3.6;  % 纵向速度采样间隔 [m/s]
N_S_SAMPLE = 1;
ROBOT_RADIUS = 2;  % 碰撞检测阈值 [m]
robot_width = 1;   % 机器人宽 [m]
robot_length = 2;  % 机器人长 [m]
 
% 评价函数权重
K_J = 0.1;
K_T = 0.1;
K_D = 1.0;
K_LAT = 1.0;
K_LON = 1.0;
 
%% 参数范围确定
c_speed = 10.0 / 3.6;   % 当前速度
c_d = 2.0;              % 当前横向位移
c_d_d = 0.0;            % 当前横向速度
c_d_dd = 0.0;           % 当前横向加速度
s0 = 0.0;               % 当前沿车道线位移
area = 20.0;

%% 参考路径点
nodes = [0, 0;
        10, -6;
        20.5 5;
        35, 6.5;
        70.5, 0;
        100, 5];
 
%% 设置障碍物坐标点
ob = [20, 10
      30, 9;
      30, 6;
      35, 9;
      50, 3;
      75, 0];
%% 生成期望路径
% csp = Cubic_spline(nodes);
x=0:0.25:100;
csp(:,1) = x';
csp(:,2) = spline(nodes(:,1),nodes(:,2),0:0.25:100);
csp(1,3) = 0;
for i=2:length(csp(:,1))
    csp(i,3) = csp(i-1,3)+sqrt((csp(i,1)-csp(i-1,1))^2+(csp(i,2)-csp(i-1,2))^2);
end

%%  Frenet轨迹结构体
FrenetPath.t = [];
FrenetPath.d = [];
FrenetPath.d_d = [];
FrenetPath.d_dd = [];
FrenetPath.d_ddd = [];
FrenetPath.s = [];
FrenetPath.s_d = [];
FrenetPath.s_dd = [];
FrenetPath.s_ddd = [];
FrenetPath.cd = 0.0;
FrenetPath.cv = 0.0;
FrenetPath.cf = 0.0;
 
FrenetPath.x = [];
FrenetPath.y = [];
FrenetPath.yaw = [];
FrenetPath.ds = [];
FrenetPath.c = [];

path =  calc_frenet_paths_fixed_velocity(csp, c_speed, c_d, c_d_d, c_d_dd, ...
                               s0,MAX_ROAD_WIDTH,D_ROAD_W, MIN_T, MAX_T, DT, ...
                               FrenetPath, TARGET_SPEED,D_T_S, N_S_SAMPLE, ...
                               K_D, K_J, K_LAT, K_LON, K_T, ob, MAX_ACCEL, ...
                               MAX_SPEED, MAX_CURVATURE,ROBOT_RADIUS);


%% 循环过程
for i=1:1:SIM_LOOP
    path =  calc_frenet_paths_fixed_velocity(csp, c_speed, c_d, c_d_d, c_d_dd, s0,MAX_ROAD_WIDTH, ...
                               D_ROAD_W, MIN_T, MAX_T, DT, FrenetPath, TARGET_SPEED,...
                               D_T_S, N_S_SAMPLE,K_D, K_J, K_LAT, K_LON, K_T, ob,...
                               MAX_ACCEL, MAX_SPEED, MAX_CURVATURE,ROBOT_RADIUS);
 
    s0 = path.s(1,2);
    c_d = path.d(1,2);
    c_d_d = path.d_d(1,2);
    c_d_dd = path.d_dd(1,2);
    c_speed = path.s_d(1,2);
    
 
    plot(csp(:,1), csp(:,2),'-.b'); hold on
    plot(csp(:,1), csp(:,2)+8,'-k');
    plot(csp(:,1), csp(:,2)-8,'-k');
    plot(ob(:,1), ob(:,2),'*g');
%     plot_robot(robot_length, robot_width, path.yaw(1,1) , path.x(1,1), path.y(1,1));
    plot(path.x(1,1), path.y(1,1),'vc','MarkerFaceColor','c','MarkerSize',6);
    plot(path.x(1,:), path.y(1,:),'-r','LineWidth',2);
    plot(path.x(1,:), path.y(1,:),'ro','MarkerFaceColor','r','MarkerSize',4);    
    set(gca,'XLim',[path.x(1,1) - area, path.x(1,1) + area]);
    set(gca,'YLim',[path.y(1,1) - area, path.y(1,1) + area]);
    grid on
    title('Frenet');
    xlabel('横坐标 x'); 
    ylabel('纵坐标 y');
    pause(0.01);
    hold off
    
    if sqrt((path.x(1,1) - nodes(end,1))^2+ (path.y(1,1) - nodes(end,2))^2) <= 1.5
        disp("Goal");
        break
    end
end