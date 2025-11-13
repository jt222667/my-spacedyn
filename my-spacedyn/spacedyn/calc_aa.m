%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   version 1 // Oct 4, 1999, Last modification by K.Yoshida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
% CALC_AA 	Calculate the coordinate transfrom matrices in the Robotics convention.
%
%   CALC_TRN( A0 , q ) returns the coordinate tranformation matrices AA.
%   AA is a collection of A_I_1, A_I_2, ... A_I_n.
%
% CALC_AA     计算机器人学坐标系中的坐标变换矩阵。
%
%   CALC_TRN( A0 , q ) 返回坐标变换矩阵 AA。
%   AA 是由 A_I_1, A_I_2, ... A_I_n 组成的集合。

function SV = calc_aa( LP, SV )

% A_0_i_tmp = zeros(3,3);
% A_BB_i_tmp = zeros(3,3);

% 基座的旋转矩阵
A_I_0 = SV.A0;

for i = 1 : LP.num_q

    % 检查链接连接：此关节是否是0关节
    if LP.BB(i) == 0 % 第(i)个链节与第0个链节相连
        % 如果是旋转关节
        % Qi暂且认为是各个关节的初始状态
        if LP.J_type(i) == 'R'
            % IMPORTANT!!! 因为我的Qi设置的就是从∑i到∑i-1的映射，所以我把转置删除了
            A_0_i = rpy2dc( LP.Qi(1,i) , LP.Qi(2,i) , LP.Qi(3,i) )*cz( SV.q(i) );

            %             if LP.Qi(1,i)~=0
            %                 A_0_i_tmp = cx(LP.Qi(1,i));
            %             elseif LP.Qi(2,i)~=0
            %                 A_0_i_tmp = cy(LP.Qi(2,i));
            %             elseif mLP.Qi(3,i)~=0
            %                 A_0_i_tmp = cz(LP.Qi(3,i));
            %             end
            %             A_0_i = A_0_i_tmp * cz(SV.q(i));

        else % 单一平移自由度的运动副
            A_0_i = (rpy2dc( LP.Qi(:,i) ));

        end
        % SV.AA的第i*3-2到i*3列
        SV.AA(:,i*3-2:i*3) = A_I_0*A_0_i;

    else % 此关节是不是0关节

        % Current (i-th) link doesn't connect to the 0-th link

        % Rotational joint
        if LP.J_type(i) == 'R'
            % IMPORTANT!!! 因为我的Qi设置的就是从∑i到∑i-1的映射，所以我把转置删除了
            A_BB_i = rpy2dc( LP.Qi(1,i), LP.Qi(2,i), LP.Qi(3,i) )*cz( SV.q(i) );

            %             if LP.Qi(1,i)~=0
            %                 A_BB_i_tmp = cx(LP.Qi(1,i));
            %             elseif LP.Qi(2,i)~=0
            %                 A_BB_i_tmp = cy(LP.Qi(2,i));
            %             elseif mLP.Qi(3,i)~=0
            %                 A_BB_i_tmp = cz(LP.Qi(3,i));
            %             end
            %             A_BB_i = A_BB_i_tmp * cz(SV.q(i));

        else % 单一平移自由度的运动副
            A_BB_i = (rpy2dc( LP.Qi(:,i) ));

        end
        % 在 SpaceDyn 的数据结构里，每个编号的连杆只有一个唯一实例
        SV.AA(:,i*3-2:i*3) = SV.AA(:,LP.BB(i)*3-2:LP.BB(i)*3)*A_BB_i;

    end

end




%%%EOF
