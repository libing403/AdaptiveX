clear; clc; close all;

%% ========= 1. 起点和终点位姿 =========

p0 = [0 0 0]';
p1 = [3 2 1]';

% 起始姿态 (单位四元数)
q0 = [1 0 0 0]';  

% 目标姿态：绕Z轴旋转90度
% angle = pi/2;
% axis  = [0 0 1]';
% q1 = [cos(angle/2);
%       axis*sin(angle/2)];
angle = 0;
axis  = [0 0 1]';
q1 = [1;0;0;0];

%% ========= 2. 路径长度计算 =========

L = norm(p1 - p0);         % 位置距离
theta = 2*acos(abs(dot(q0,q1))); % 姿态角

k_rot = 0.1;  % 姿态权重 (m/rad)

L_total = max(L, k_rot*theta);

%% ========= 3. 时间规划 =========

T = 2;              % 总时间
dt = 0.002;
t = 0:dt:T;

% 五次多项式 s(t)
s = 10*(t/T).^3 ...
   -15*(t/T).^4 ...
   + 6*(t/T).^5;

s_dot = gradient(s, dt);

%% ========= 4. 插补 =========

p = zeros(3,length(t));
theta_traj = zeros(1,length(t));

for i = 1:length(t)
    
    % ---- 位置 ----
    p(:,i) = p0 + (p1-p0)*s(i);
    
    % ---- 姿态 (SLERP) ----
    q = slerp(q0,q1,s(i));
    
    % 当前姿态角
    theta_traj(i) = 2*acos(abs(dot(q0,q)));
end

%% ========= 5. 速度计算 =========

v_linear  = vecnorm(gradient(p,dt),2,1);
omega_mag = gradient(theta_traj, dt);

%% ========= 6. 画图 =========

figure;

subplot(3,2,1);
plot(t,p);
title('Position XYZ');
legend('x','y','z');

subplot(3,2,2);
plot(t,theta_traj);
title('Orientation Angle (rad)');

subplot(3,2,3);
plot(t,v_linear);
title('Linear Velocity');

subplot(3,2,4);
plot(t,omega_mag);
title('Angular Velocity');

subplot(3,2,5);
plot(t,s);
title('Path Parameter s(t)');

subplot(3,2,6);
plot(t,s_dot);
title('Path Speed s\_dot');

sgtitle('Position + Orientation Unified Planning');

%% ========= 7. SLERP函数 =========

function q = slerp(q0,q1,s)

    dotq = dot(q0,q1);

    if dotq < 0
        q1 = -q1;
        dotq = -dotq;
    end

    if dotq > 0.9995
        q = q0 + s*(q1-q0);
        q = q / norm(q);
        return;
    end

    theta = acos(dotq);
    sin_theta = sin(theta);
    w1 = sin((1-s)*theta)/sin_theta;
    w2 = sin(s*theta)/sin_theta;
    q = w1*q0 + w2*q1;
end
