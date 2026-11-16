% Sistema aumentado com o integrador
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

% Integrador do erro de posicao como quinto estado
Cx = [0 0 1 0];
Aa = [A, zeros(4,1); -Cx, 0];
B2 = [B; 0];
B1 = [1.0*BF, 0.05*BT; 0, 0]; % forca de 1 N e torque de 0,05 N.m

printf('Aa =\n'); disp(Aa);
printf('B2 =\n'); disp(B2');

% O teste do Ogata vale porque a planta ja passou no a_modelo.m: o aumentado
% e controlavel se a planta for controlavel e esta matriz tiver posto n + 1.
printf('posto de [A B; -Cx 0]   = %d de 5\n', rank([A B; -Cx 0]));
printf('posto de ctrb(Aa,B2)    = %d de 5\n', rank(ctrb(Aa,B2)));

% A observabilidade cai para 4 e isso e esperado: o quinto estado nao e medido
% nem estimado, e sim somado pelo proprio programa a partir da posicao e da
% referencia. O observador estima so os quatro estados da planta.
printf('posto de obsv(Aa,[C 0]) = %d de 5\n', rank(obsv(Aa, [C zeros(2,1)])));
