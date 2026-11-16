% Discretizacao no periodo do microcontrolador
clc; clear;

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end

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

% Discretizacao exata com segurador de ordem zero
sysd = c2d(ss(Aa, B2, [C zeros(2,1)], D), Ts, 'zoh');
Ada = sysd.a;
Bda = sysd.b;

% O bloco da planta, que e o que vai para o observador e para o firmware
Ad = Ada(1:4,1:4);
Bd = Bda(1:4);

printf('Ad =\n'); disp(Ad);
printf('Bd =\n'); disp(Bd');
printf('|z|: '); printf('%.4f  ', sort(abs(eig(Ad)))); printf('\n');
printf('posto de ctrb(Ada,Bda)   = %d de 5\n', rank(ctrb(Ada,Bda)));
printf('posto de obsv(Ada,[C 0]) = %d de 5\n', rank(obsv(Ada, [C zeros(2,1)])));

% Deixa o modelo pronto para as rotinas seguintes, que nao o remontam.
save('-binary', fullfile(PASTA, 'modelo.mat'), 'm', 'L', 'I', 'g', 'Me', 'alfa', 'beta', 'Ts', ...
     'Vsat', 'A', 'B', 'BF', 'BT', 'C', 'D', 'Cx', 'Aa', 'B2', 'Ad', 'Bd', ...
     'Ada', 'Bda');
