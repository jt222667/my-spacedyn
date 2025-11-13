function show_kin(q,i)

% ---------- 初始化 ----------
LP = init_LP_1028();
SV = init_SV_1027();

SV.q = q;

SV = calc_aa(LP, SV);
SV = calc_pos(LP, SV);
joint = zeros(1,21);

joint(i*7-6:i*7) = j_num(  LP , i );
j = joint(i*7-6:i*7);
disp(j)
[ POS_j , ORI_j ] = f_kin_j( LP, SV, j );
disp(POS_j);

end