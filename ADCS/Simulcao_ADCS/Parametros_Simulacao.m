%%Simulacao_CubeSat_2U

open('Simulacao_CubeSat_2U.slx');

%Parametros Motor

L = 0.05e-3;
R = 0.7;
J = 14e-6;
w = 14760;
Ve = 11.63;
Kv = w/Ve;
Km = 60/(2*pi*Kv);
Kb = 1/Kv;
Kf = 207.88e-9;

%Parametros CubeSat

Te = 0;
vo = 60;
Wo = [0 0 (2*vo*pi)/60]';
%Jc = [0.00653855 0 0;0 0.00635807 0;0 0 0.00278312];
Jc = [0.00653855 0.00000999 -0.00003341; 0.00000999 0.00635807 0.0000403; -0.00003341 0.00004037 0.00278312];
Jb = J + eye(3)*Jc;

