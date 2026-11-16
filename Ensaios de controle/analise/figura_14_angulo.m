
% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS   = fullfile(PASTA, '..', 'dados');
FIGURAS = fullfile(PASTA, '..', 'figuras');
if ~exist(FIGURAS, 'dir'), mkdir(FIGURAS); end
cd(FIGURAS);   % o print grava pelo nome curto: o Ghostscript nao aceita
              % o acento de 'Area de Trabalho' no caminho completo

ARQUIVO   = fullfile(DADOS, 'onda60_polos_55_a_64.csv');
SAIDA     = 'figura_14_angulo.png';
SAIDA_ZUM = 'figura_14_angulo_zoom.png';

T_ANALISE = 100.0;
T_ZOOM    = 2.0;    % largura da janela ampliada [s]

pkg load control

% Parâmetros medidos na bancada, Tabela 2.
m = 0.178; % massa da haste [kg]
L = 0.41261; % articulação até o centro de gravidade [m]
I = 0.00382; % inércia da haste em relação ao centro de gravidade [kg.m2]
g = 9.81; % gravidade [m/s2]
Me = 0.7071; % massa efetiva do carro [kg]
alfa = 1.239; % ganho do acionamento [N/V]
beta = 9.670; % amortecimento eletromagnético [N.s/m]
Ts   = 0.010; % período de amostragem do controle [s]
Vsat = 10; % saturação usada no firmware [V]

% Modelo linearizado da equação (2.32). Delta é o determinante da matriz de
% massa; a terceira coluna de A é nula porque nada restaura a posição do carro.
mL    = m*L;
Ip    = I + m*L^2;
Delta = (Me + m)*I + Me*m*L^2;

A = [ 0                    1   0   0
      (Me+m)*m*g*L/Delta   0   0   mL*beta/Delta
      0                    0   0   1
     -mL*m*g*L/Delta       0   0  -Ip*beta/Delta ];

BF = [0; -mL/Delta;    0;  Ip/Delta];   % força no carro [N]; sinais opostos
BT = [0; (Me+m)/Delta; 0; -mL/Delta];   % torque na haste [N.m]
B  = alfa*BF;                           % tensão [V]
C  = [1 0 0 0; 0 0 1 0];                % os dois encoders
D  = [0; 0];

% Discretização exata com segurador de ordem zero.
sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
Ad = sysd.a;
Bd = sysd.b;

% Integrador do erro de posição como quinto estado.
Cx  = [0 0 1 0];
Aa  = [A, zeros(4,1); -Cx, 0];
B2  = [B; 0];
B1  = [1.0*BF, 0.05*BT; 0, 0];          % força de 1 N e torque de 0,05 N.m

% Saídas de desempenho normalizadas pelo maior valor admitido de cada uma.
w   = [0.10, 0.05, 0.05, 1.0];          % theta [rad], x [m], xi [m.s], u [V]
C1  = [1/w(1) 0 0 0 0
       0 0 1/w(2) 0 0
       0 0 0 0 1/w(3)
       0 0 0 0 0];
D12 = [0; 0; 0; 1/w(4)];
R   = D12'*D12;

% Síntese. Para cada gamma resolve a equação de Riccati do problema H∞ com a
% função care do pacote control, escrevendo a perturbação como um segundo canal
% de entrada com peso negativo, -gamma². A solução só vale se for positiva
% semidefinida, e a bisseção procura o menor gamma que ainda atende.
lo = 1e-3;  hi = 1e3;
for passo = 1:40
  gam = sqrt(lo*hi);
  try
    X  = care(Aa, [B2, B1], C1'*C1, blkdiag(R, -gam^2*eye(2)));
    ok = min(eig(X)) >= 0;
  catch
    ok = false;     % gamma pequeno demais: nem solução existe
  end
  if ok, hi = gam; else, lo = gam; end
end

% Adotado o dobro do menor gamma, que mantém a tensão longe da saturação.
X  = care(Aa, [B2, B1], C1'*C1, blkdiag(R, -(2*hi)^2*eye(2)));
Ka = R\(B2'*X);

% Observador. Polos do erro de estimação bem à esquerda dos de malha fechada.
Ke = place(Ad', C', exp([-55 -58 -61 -64](:)*Ts))';

% Ensaio da bancada. A terceira coluna é o ângulo, em radianos.
Bm = dlmread(ARQUIVO, ',', 1, 0);
Bm = Bm(Bm(:,1) <= T_ANALISE, :);
tb = Bm(:,1);  rb = Bm(:,2)*1000;  thb = Bm(:,3)*180/pi;

% A onda da simulação é lida do próprio ensaio, para os dois serem iguais.
ib = find(abs(diff(rb)) > 1.0) + 1;
A_ONDA   = max(abs(rb))/1000;
T_INICIO = tb(ib(1));
T_ONDA   = 2*mean(diff(tb(ib)));
printf('\n  onda do ensaio: +-%.0f mm, período de %.0f s, começando em %.0f s\n', ...
       A_ONDA*1000, T_ONDA, T_INICIO);

% Simulação em malha fechada, com o observador e o integrador condicional.
N  = round((tb(end) + 1)/Ts);
ts = (0:N-1)'*Ts;
ths = zeros(N,1);
x  = [0;0;0;0];  xh = x;  xi = 0;
for k = 1:N
  ths(k) = x(1)*180/pi;

  r = 0;                                   % a referência do passo atual
  if ts(k) >= T_INICIO
    if mod(ts(k) - T_INICIO, T_ONDA) < 0.5*T_ONDA
      r =  A_ONDA;
    else
      r = -A_ONDA;
    end
  end

  vc = -Ka*[xh; xi];
  v  = max(-Vsat, min(Vsat, vc));
  e  = r - x(3);
  if !((v != vc) && (sign(-Ka(5)*e) == sign(vc)))
    xi = xi + Ts*e;
  end
  xh = Ad*xh + Bd*v + Ke*(C*x - C*xh);
  x  = Ad*x  + Bd*v;
end

printf('\n  ângulo da haste\n');
printf('     bancada   : máximo %.2f graus, rms %.3f graus\n', ...
       max(abs(thb)), sqrt(mean(thb.^2)));
printf('     simulação : máximo %.2f graus, rms %.3f graus\n', ...
       max(abs(ths)), sqrt(mean(ths.^2)));

% Figura cheia. O encoder da haste mede em degraus de 0,088 grau, e em 100 s a
% curva crua vira uma faixa. Por isso o sinal medido e mostrado filtrado por um
% passa-baixa Butterworth de 2a ordem com corte em 3 Hz, aplicado nos dois
% sentidos (filtfilt), que nao atrasa a curva. O filtro e so para a figura: os
% valores impressos acima sao calculados sem filtro.
pkg load signal
F_CORTE = 3;                                  % [Hz]
[bf, af] = butter(2, F_CORTE/(0.5/(tb(2) - tb(1))));
thf = filtfilt(bf, af, thb);
fg = figure();
hold on;
h2 = plot(tb, thf, 'b-', 'linewidth', 1.2);
h1 = plot(ts, ths, 'r-', 'linewidth', 1.2);
hold off;  grid on;
xlim([0 tb(end)]);  ylim([-1.5 1.5]);
xlabel('Tempo [s]');  ylabel('Ângulo [graus]');
legend([h2 h1], {'Real, filtro passa-baixa de 3 Hz', 'Simulado'}, 'location', 'southeast');
destino = SAIDA;
print(fg, destino, '-dpng', '-S1000,440');
printf('\n  %s\n', destino);

% Figura ampliada, em um patamar, longe das bordas, onde se vê a oscilação.
if numel(ib) > 1
  centro = 0.5*(tb(ib(1)) + tb(ib(2)));
else
  centro = 0.5*tb(end);
end
jan = [centro - T_ZOOM/2, centro + T_ZOOM/2];

fz = figure();
hold on;
h2 = plot(tb, thb, 'b-', 'linewidth', 1.2);
h1 = plot(ts, ths, 'r-', 'linewidth', 1.2);
hold off;  grid on;
xlim(jan);
% O eixo vertical é refeito para a janela, senão a curva fica achatada na
% escala da corrida inteira.
sel = tb >= jan(1) & tb <= jan(2);
alt = max(thb(sel)) - min(thb(sel));
ylim([min(thb(sel)) - 0.2*alt, max(thb(sel)) + 0.2*alt]);
xlabel('Tempo [s]');  ylabel('Ângulo [graus]');
legend([h1 h2], {'Simulado', 'Real'}, 'location', 'southwest');
destino = SAIDA_ZUM;
print(fz, destino, '-dpng', '-S1000,440');
printf('  %s\n\n', destino);

printf('\n  imagens gravadas em %s\n\n', FIGURAS);
cd(PASTA);     % volta para a pasta da rotina
