%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	February 4, 1999, Last modification by K.Yoshida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
% F_KINE_J	Forward Kinematics:
%		Caluculate the Position/Orientation of the joints
%		corresponding the point specified by connection vector 'joints.'
%		POS : 3x1 vector, ORI : 3x3 matrix,
%		POS_j : 3xn, ORI_j : 3x3n,
%		where n : number of joints between the End-point to link 0.
%
% F_KINE_J    正向运动学：计算条支链上所有关节的位置、方向
%        joints：有j_num计算得到，再由用户选取需要的支链
%        POS：3x1向量，ORI：3x3矩阵，
%        POS_j：3xn，ORI_j：3x3n，
%        其中n：终点到第0连杆之间的关节数。

function [ POS_j , ORI_j ] = f_kin_j( LP, SV, joints )

Ez = [0; 0; 1];       % 初始化 Ez


% Check the number of the corresponding joints
n = length( joints );


% Calculation of Orientation and Position of each joints
POS_j = [];
ORI_j = [];

for i = 1 : 1 : n
   % 全是旋转关节，PorR = 0
   PorR = ( LP.J_type(joints(i)) == 'P' );
   ORI_tmp = SV.AA(:,joints(i)*3-2:joints(i)*3);
   % SV.RR(:,1)表示连杆1的质心的向量
   % SV.RR(:,1，3)表示关节1的旋转向量
   % POS_tmp是在世界坐标系下
   POS_tmp = SV.RR(:,joints(i)) + ORI_tmp * ( LP.cc(:,joints(i),joints(i)) - PorR*Ez * SV.q(joints(i)) );
   
   POS_j = [POS_j POS_tmp];
   ORI_j = [ORI_j ORI_tmp];
   
end


%%%EOF
