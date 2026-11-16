% Modelo numerico da bancada e condicoes estruturais, Secoes 4.1 e 4.2 do TCC.
clc; clear;
pkg load control

% Parametros medidos na bancada
m    = 0.178; % massa da haste [kg]
L    = 0.41261; % articulacao ate o centro de gravidade [m]
I    = 0.00382; % inercia da haste em relacao ao centro de gravidade [kg.m2]
g    = 9.81; % gravidade [m/s2]
Me   = 0.7071; % massa efetiva do carro [kg]
alfa = 1.239; % ganho do acionamento [N/V]
beta = 9.670; % amortecimento eletromagnetico [N.s/m]
Ts   = 0.010; % periodo de amostragem [s]
Vsat = 12; % saturacao da tensao [V]

% Modelo linearizado da equacao
mL    = m*L;
Ip    = I + m*L^2;
Delta = (Me + m)*I + Me*m*L^2;

A = [ 0                    1   0   0
      (Me+m)*m*g*L/Delta   0   0   mL*beta/Delta
      0                    0   0   1
     -mL*m*g*L/Delta       0   0  -Ip*beta/Delta ];

BF = [0; -mL/Delta;    0;  Ip/Delta]; % forca no carro [N]; sinais opostos
BT = [0; (Me+m)/Delta; 0; -mL/Delta]; % torque na haste [N.m]
B  = alfa*BF; % tensao [V]
C  = [1 0 0 0; 0 0 1 0]; % os dois encoders (angulo e posição)
D  = [0; 0];

printf('A =\n'); disp(A);
printf('B =\n'); disp(B');
printf('C =\n'); disp(C);
printf('autovalores [1/s]: '); printf('%.4f  ', sort(real(eig(A)))); printf('\n');
printf('polo instavel: %.4f 1/s, constante de tempo de %.0f ms\n', ...
max(real(eig(A))), 1000/max(real(eig(A))));

% Condicoes estruturais da planta. So os quatro estados fisicos; o sistema
% aumentado com o integrador e testado no b_integrador.m.
printf('posto de ctrb(A,B) = %d de 4\n', rank(ctrb(A,B)));
printf('posto de obsv(A,C) = %d de 4\n', rank(obsv(A,C)));
printf('so com theta = %d de 4\n', rank(obsv(A,C(1,:))));
printf('so com x = %d de 4\n', rank(obsv(A,C(2,:))));
