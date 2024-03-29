function B_handle = linear_SBR6voltage_B(I_W_a,I_W_d,I_y,I_z,R_m,d,k_m,l,m_B,m_W,r)
%LINEAR_SBR6VOLTAGE_B
%    B_HANDLE = LINEAR_SBR6VOLTAGE_B(I_W_A,I_W_D,I_Y,I_Z,R_M,D,K_M,L,M_B,M_W,R)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    10-May-2020 07:48:56

t2 = l.^2;
t3 = r.^2;
t4 = 1.0./R_m;
t5 = m_B.*t2;
t6 = l.*m_B.*r;
t7 = I_y+t5+t6;
t8 = I_W_a.*I_y.*2.0;
t9 = I_W_a.*m_B.*t2.*2.0;
t10 = I_y.*m_B.*t3;
t11 = I_y.*m_W.*t3.*2.0;
t12 = m_B.*m_W.*t2.*t3.*2.0;
t13 = t8+t9+t10+t11+t12;
t14 = 1.0./t13;
t15 = k_m.*r.*t4.*t7.*t14;
t16 = I_W_a.*2.0;
t17 = m_B.*t3;
t18 = m_W.*t3.*2.0;
t19 = t6+t16+t17+t18;
t20 = d.^2;
t21 = I_W_a.*t20;
t22 = I_W_d.*t3.*4.0;
t23 = I_z.*t3.*2.0;
t24 = m_W.*t3.*t20;
t25 = t21+t22+t23+t24;
t26 = 1.0./t25;
t27 = d.*k_m.*r.*t4.*t26;
B_handle = reshape([0.0,0.0,0.0,t15,-k_m.*t4.*t14.*t19,t27,0.0,0.0,0.0,t15,-k_m.*t4.*t14.*t19,-t27],[6,2]);
