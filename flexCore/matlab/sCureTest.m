clear; clc; close all;

%% 参数
D    = 1;
Vmax = 50.0;
Amax = 1000.0;
Jmax = 10000.0;

%% 时间计算（对称有匀速情况）
Tj = Amax/Jmax;
Ta = (Vmax/Amax) - Tj;

% 加速段位移
Dacc = Amax*(Tj^2 + 1.5*Tj*Ta + 0.5*Ta^2);

Tv = (D - 2*Dacc)/Vmax;

T_total = 4*Tj + 2*Ta + Tv;

%% 轨迹生成
dt = 0.0005;
t  = 0:dt:T_total;

x = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));
j = zeros(size(t));

x0=0; v0=0; a0=0;

for i=1:length(t)

    ti = t(i);

    if ti <= Tj
        J =  Jmax;

    elseif ti <= Tj+Ta
        J = 0;

    elseif ti <= 2*Tj+Ta
        J = -Jmax;

    elseif ti <= 2*Tj+Ta+Tv
        J = 0;

    elseif ti <= 3*Tj+Ta+Tv
        J = -Jmax;

    elseif ti <= 3*Tj+2*Ta+Tv
        J = 0;

    else
        J = Jmax;
    end

    % 数值积分（推荐工业做法）
    a0 = a0 + J*dt;
    v0 = v0 + a0*dt;
    x0 = x0 + v0*dt;

    j(i)=J;
    a(i)=a0;
    v(i)=v0;
    x(i)=x0;
end

%% 画图
figure;
subplot(4,1,1); plot(t,x); title('Position');
subplot(4,1,2); plot(t,v); title('Velocity');
subplot(4,1,3); plot(t,a); title('Acceleration');
subplot(4,1,4); plot(t,j); title('Jerk');
sgtitle('Correct Continuous 7-Segment S-Curve');
