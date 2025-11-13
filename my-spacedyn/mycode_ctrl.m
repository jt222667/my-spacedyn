function [sys,x0,str,ts] = mycode_ctrl(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs     = 84;
sizes.NumInputs      = 42;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0  = [];
str = [];
ts  = [0.001 0];

function sys = mdlOutputs(t,x,u)

% ---------- 读取测量量 ----------
qr  = u(1:21);      % 实际关节角度
dqr = u(22:42);     % 实际关节角速度

% ---------- 轨迹生成初始化 ----------
T  = 1;   % 轨迹时长（s）
nq = 21;  % 关节数
q0 = zeros(nq,1);
qf = zeros(nq,1);
qd   = zeros(nq,1); % 期望位置
dqd  = zeros(nq,1); % 期望速度
ddqd = zeros(nq,1); % 期望加速度

% ---------- 当下时刻指定关节i对应的期望轨迹（角度、角速度、角加速度） ----------
qf(1:21) = 0.5*pi;

for i = 1:nq
    [qd(i), dqd(i), ddqd(i)] = quintic_trajectory_mex(t, T, q0(i), qf(i));
end

% ---------- 动力学计算 ----------
[M, C, G, F] = calculate_dynamics_mex(qr, dqr);
show_kin(qd,3);
% ---------- 控制律 ----------
tau = M * ddqd + C + G + F;

sys = zeros(84,1);
sys(1:21)  = tau;
sys(22:42) = qr;
sys(43:63) = dqr;
sys(64:84) = qd;



