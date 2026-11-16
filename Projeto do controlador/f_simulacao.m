% Malha fechada, Secoes 4.5 e 4.6 do TCC.
clc; clear; pkg load control

% Parametros medidos na bancada, Tabela 2 do TCC.
m = 0.178; % massa da haste [kg]
L = 0.41261; % articulacao ate o centro de gravidade [m]
I = 0.00382; % inercia da haste em relacao ao centro de gravidade [kg.m2]
g = 9.81; % gravidade [m/s2]
Me = 0.7071; % massa efetiva do carro [kg]
alfa = 1.239; % ganho do acionamento [N/V]
beta = 9.670; % amortecimento eletromagnetico [N.s/m]
Ts = 0.010; % periodo de amostragem [s]
Vsat = 12; % saturacao da tensao [V]

% Modelo linearizado da equacao (2.32). Delta e o determinante da matriz de
% massa; a terceira coluna de A e nula porque nada restaura a posicao do carro.
mL = m*L;
Ip = I + m*L^2;
Delta = (Me + m)*I + Me*m*L^2;

A = [ 0                    1   0   0
      (Me+m)*m*g*L/Delta   0   0   mL*beta/Delta
      0                    0   0   1
     -mL*m*g*L/Delta       0   0  -Ip*beta/Delta ];

BF = [0; -mL/Delta;    0;  Ip/Delta];   % forca no carro [N]; sinais opostos
BT = [0; (Me+m)/Delta; 0; -mL/Delta];   % torque na haste [N.m]
B  = alfa*BF;                           % tensao [V]
C  = [1 0 0 0; 0 0 1 0];                % os dois encoders
D  = [0; 0];

% Discretizacao exata com segurador de ordem zero. O c2d calcula por dentro a
% exponencial da matriz aumentada; a formula Bd = A\(Ad - I)B nao serviria,
% porque A tem autovalor nulo e nao e inversivel.
sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
Ad = sysd.a;
Bd = sysd.b;

% Integrador do erro de posicao como quinto estado. No discreto a soma
% acumulada equivale a regra retangular.
Cx  = [0 0 1 0];
Aa  = [A, zeros(4,1); -Cx, 0];
B2  = [B; 0];
B1  = [1.0*BF, 0.05*BT; 0, 0]; % forca de 1 N e torque de 0,05 N.m

% Saidas de desempenho normalizadas pelo maior valor admitido de cada uma.
w   = [0.10, 0.05, 0.05, 1.0]; % theta [rad], x [m], xi [m.s], u [V]
C1  = [1/w(1) 0 0 0 0
       0 0 1/w(2) 0 0
       0 0 0 0 1/w(3)
       0 0 0 0 0];
D12 = [0; 0; 0; 1/w(4)];
R   = D12'*D12;

% Sintese. Resolve Aa'X + XAa + C1'C1 + X(B1B1'/gam^2 - B2 R^-1 B2')X = 0 pelo
% subespaco estavel da Hamiltoniana. Os 80 primeiros passos sao a bissecao que
% acha o menor gamma; o passo 81 refaz a conta com o gamma adotado, 2 gmin.
lo = 1e-3;  hi = 1e3;  n = 5;
for passo = 1:81
  if passo <= 80
    gam = sqrt(lo*hi);
  else
    gmin = hi;  gam = 2*gmin;
  end

  Rt = B1*B1'/gam^2 - B2*(R\B2');
  H  = [Aa, Rt; -C1'*C1, -Aa'];
  [V_, E_] = eig(H);
  e_ = diag(E_);
  ok = false;  X = [];  Ka = [];
  if all(abs(real(e_)) >= 1e-8)             % nada no eixo imaginario
    sel = real(e_) < 0;
    if nnz(sel) == n
      V1 = V_(1:n, sel);  V2 = V_(n+1:end, sel);
      if rcond(V1) >= 1e-12
        X = real(V2/V1);  X = (X + X')/2;
        if min(eig(X)) >= -1e-9             % solucao semidefinida positiva
          Ka = R\(B2'*X);
          ok = max(real(eig(Aa - B2*Ka))) < 0;
        end
      end
    end
  end

  if passo <= 80
    if ok, hi = gam; else, lo = gam; end
  end
end
K  = Ka(1:4);
kI = -Ka(5);

% Observador. Polos do erro de estimacao bem a esquerda dos de malha fechada,
% com o limite dado pela amplificacao do ruido de medicao.
% Polos escolhidos pela varredura experimental da Secao 5.6 do TCC.
polos = [-55 -58 -61 -64];
Ke = place(Ad', C', exp(polos(:)*Ts))';

% Simulacao discreta. Haste partindo de 3 graus e degrau de 50 mm na referencia
% em t = 4 s. Roda duas vezes: estados medidos e observador. A integracao e
% condicional, ou seja, para enquanto a tensao esta saturada no sentido que
% pioraria o erro.
N  = 1200;  tr = 4;
r  = zeros(N,1);  r(round(tr/Ts)+1:end) = 0.050;
x0 = [3*pi/180; 0; 0; 0];

for caso = 0:1
  x = x0(:);  xh = [x0(1); 0; x0(3); 0];  xi = 0;
  t_ = (0:N-1)'*Ts;  xs = zeros(N,4);  vs = zeros(N,1);
  for k = 1:N
    xs(k,:) = x';
    if caso, xu = xh; else, xu = x; end
    vc = -Ka*[xu; xi];
    v  = max(-Vsat, min(Vsat, vc));
    vs(k) = v;
    e = r(k) - x(3);
    if !((v != vc) && (sign(-Ka(5)*e) == sign(vc)))
      xi = xi + Ts*e;
    end
    xh = Ad*xh + Bd*v + Ke*(C*x - C*xh);
    x  = Ad*x + Bd*v;
  end
  if caso == 0, sm.t = t_; sm.x = xs; sm.v = vs; else, so.t = t_; so.x = xs; so.v = vs; end
end

for caso = 0:1
  if caso, s = so; rot = 'observador'; else, s = sm; rot = 'estados medidos'; end
  th = s.x(:,1)*180/pi;  xc = s.x(:,3)*1000;
  ini = s.t < tr;  deg = s.t >= tr;  td = s.t(deg);
  i = find(abs(th(ini)) > 0.1, 1, 'last');
  j = find(abs(xc(deg) - 50) > 1.0, 1, 'last');
  printf('\n%s\n', rot);
  printf('   tensao maxima        : %.1f V\n', max(abs(s.v)));
  printf('   angulo abaixo de 0,1 : %.1f s\n', s.t(i));
  printf('   inclinacao no degrau : %.2f graus\n', max(abs(th(deg))));
  printf('   acomodacao do degrau : %.1f s\n', td(j) - tr);
  printf('   erro final           : %.2f mm\n', xc(end) - 50);
end

M = [sm.t, sm.x(:,1)*180/pi, sm.x(:,3)*1000, sm.v, ...
     so.x(:,1)*180/pi, so.x(:,3)*1000, so.v, r*1000];
% Figura da simulacao, com os estados supostos medidos e com o observador.
tt = sm.t;
fg = figure();
subplot(3,1,1);  hold on;
plot(tt, r*1000, 'k--', 'linewidth', 1.0);
plot(tt, sm.x(:,3)*1000, 'b-', 'linewidth', 1.6);
plot(tt, so.x(:,3)*1000, 'r--', 'linewidth', 1.4);
hold off;  grid on;  ylabel('Posicao [mm]');
legend('Referencia', 'Estados medidos', 'Com observador', 'location', 'southeast');
title('Simulacao em malha fechada: haste a 3 graus e degrau de 50 mm em t = 4 s');
subplot(3,1,2);  hold on;
plot(tt, sm.x(:,1)*180/pi, 'b-', 'linewidth', 1.6);
plot(tt, so.x(:,1)*180/pi, 'r--', 'linewidth', 1.4);
hold off;  grid on;  ylabel('Angulo [grau]');
subplot(3,1,3);  hold on;
plot(tt, sm.v, 'b-', 'linewidth', 1.6);
plot(tt, so.v, 'r--', 'linewidth', 1.4);
hold off;  grid on;  ylabel('Tensao [V]');  xlabel('Tempo [s]');

% A figura tambem e gravada em PNG, na pasta figuras ao lado desta rotina.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
FIGURAS = fullfile(PASTA, 'figuras');
if ~exist(FIGURAS, 'dir'), mkdir(FIGURAS); end
cd(FIGURAS);   % o print grava pelo nome curto: o Ghostscript nao aceita
               % o acento de 'Area de Trabalho' no caminho completo
print(fg, 'f_simulacao.png', '-dpng', '-S1000,750');
printf('\n  imagem gravada em %s\n\n', fullfile(FIGURAS, 'f_simulacao.png'));
cd(PASTA);     % volta para a pasta da rotina
