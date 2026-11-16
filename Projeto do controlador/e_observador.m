% Observador de estados de ordem plena
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

% Discretizacao exata com segurador de ordem zero
sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
Ad = sysd.a;
Bd = sysd.b;

% Observador
% Polos escolhidos pela varredura experimental da Secao 5.6 do TCC.
polos = [-55 -58 -61 -64];
Ke = place(Ad', C', exp(polos(:)*Ts))';

printf('polos escolhidos [1/s]: '); printf('%d  ', polos); printf('\n');
printf('|z| alocados: '); printf('%.4f  ', abs(exp(polos*Ts))); printf('\n\n');
printf('Ke =\n'); disp(Ke);
printf('|z| de (Ad - Ke C): '); printf('%.4f  ', sort(abs(eig(Ad - Ke*C)))); printf('\n');
