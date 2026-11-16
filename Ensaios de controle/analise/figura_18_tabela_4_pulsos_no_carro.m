% Valida a rejeicao de perturbacao: ensaio contra modelo, pulso por pulso.

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS   = fullfile(PASTA, '..', 'dados');
FIGURAS = fullfile(PASTA, '..', 'figuras');
if ~exist(FIGURAS, 'dir'), mkdir(FIGURAS); end
cd(FIGURAS);   % o print grava pelo nome curto: o Ghostscript nao aceita
              % o acento de 'Area de Trabalho' no caminho completo

ARQUIVO   = fullfile(DADOS, 'pulsos_no_carro.csv');
SAIDA     = 'figura_18_pulsos_no_carro_registro.png';
SAIDA_RSP = 'figura_18_pulsos_no_carro.png';

T_JANELA = 4.0;            % quanto de cada resposta se analisa [s]

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

mL = m*L;
Ip = I + m*L^2;
Delta = (Me + m)*I + Me*m*L^2;

A = [ 0                    1   0   0
      (Me+m)*m*g*L/Delta   0   0   mL*beta/Delta
      0                    0   0   1
     -mL*m*g*L/Delta       0   0  -Ip*beta/Delta ];

B = alfa*[0; -mL/Delta; 0; Ip/Delta];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
Ad = sysd.a;  Bd = sysd.b;

Ka = [-114.1031 -24.4779 -57.9692 -43.3083 32.7786];
Ke = place(Ad', C', exp([-55 -58 -61 -64](:)*Ts))';

% ------------------------------------------------------------------ ensaio ---
if ~exist(ARQUIVO, 'file')
  printf('nao achei %s\n', ARQUIVO);
  printf('grave o registro do ensaio com esse nome e rode de novo.\n');
  return
end
Bm = dlmread(ARQUIVO, ',', 1, 0);
if columns(Bm) < 10
  printf('o registro tem %d colunas; o ensaio de perturbacao grava 10.\n', columns(Bm));
  printf('confira se o firmware gravado e o controle_hinf_perturbacao.\n');
  return
end
tb = Bm(:,1);  thb = Bm(:,3)*180/pi;  xb = Bm(:,5)*1000;  ub = Bm(:,8);
db = Bm(:,9);
Tb = tb(2) - tb(1);

ini = find(db ~= 0 & [0; db(1:end-1)] == 0);
Vp  = db(ini);
printf('\n%d pulsos no registro: ', numel(ini));  printf('%+.0f ', Vp);  printf('V\n');

% ------------------------------------------------------- ruido de fundo -----
% medido nos trechos entre pulsos, longe do transitorio de cada um
q = false(size(tb));
for i = 1:numel(ini)
  q = q | (tb >= tb(ini(i)) + 5 & tb < tb(ini(i)) + 9.5);
end
thq = thb(q) - mean(thb(q));
n = numel(thq);  F = abs(fft(thq))*2/n;  f = (0:n-1)'/(n*Tb);
fx = f > 0.3 & f < 10;  [amp, j] = max(F(fx));  ff = f(fx);
printf('\nfundo entre os pulsos\n');
printf('   angulo medio  %+.2f graus   eficaz %.3f graus\n', mean(thb(q)), std(thb(q)));
printf('   oscilacao residual de %.3f grau a %.2f Hz\n', amp, ff(j));
if abs(mean(thb(q))) > 1.0
  printf('   ATENCAO: a haste esta parada longe do zero do encoder. O integrador\n');
  printf('   absorve o desvio, mas convem rezerar a haste antes do proximo ensaio.\n');
end


% --------------------------------------------- resposta do modelo ao pulso --
function [tt, yth, yx] = resposta(V, Ad, Bd, Ke, Ka, C, Ts, Vsat, T)
  N = round(T/Ts);
  xx = [0;0;0;0];  xh = xx;  xi = 0;
  yth = zeros(N,1);  yx = zeros(N,1);
  for k = 1:N
    yth(k) = xx(1)*180/pi;  yx(k) = xx(3)*1000;
    vc = -Ka*[xh; xi];  v = max(-Vsat, min(Vsat, vc));
    e = -xx(3);
    if !((v != vc) && (sign(-Ka(5)*e) == sign(vc))), xi = xi + Ts*e; end
    dd = ((k-1)*Ts < 0.100) * V;
    vm = max(-Vsat, min(Vsat, v + dd));
    xh = Ad*xh + Bd*v + Ke*(C*xx - C*xh);
    xx = Ad*xx + Bd*vm;
  end
  tt = (0:N-1)'*Ts;
end


% --------------------------------------------------- pares de mesmo modulo --
amps = unique(abs(Vp));
tau  = (0:Tb:T_JANELA)';
ENS  = {};  MOD = {};  LAB = {};

printf('\nresposta ao pulso, com o fundo cancelado\n');
printf('\n%6s %7s   %8s %8s %6s    %8s %8s %6s\n', 'pulso', 'forca', ...
       'th ens', 'th mod', 'razao', 'x ens', 'x mod', 'razao');
printf('%6s %7s   %8s %8s %6s    %8s %8s %6s\n', '[V]', '[N]', ...
       '[grau]', '[grau]', '', '[mm]', '[mm]', '');
printf('%s\n', repmat('-', 1, 74));

for a = amps(:)'
  kp = ini(abs(Vp - a) < 0.01);
  km = ini(abs(Vp + a) < 0.01);
  if isempty(kp) || isempty(km)
    printf('%6.1f   sem o par de sinal oposto, pulado\n', a);
    continue
  end
  gp = interp1(tb - tb(kp(1)), [thb xb], tau);
  gm = interp1(tb - tb(km(1)), [thb xb], tau);
  jp = tb >= tb(kp(1)) - 1 & tb < tb(kp(1));
  jm = tb >= tb(km(1)) - 1 & tb < tb(km(1));
  bp = [mean(thb(jp)), mean(xb(jp))];
  bm = [mean(thb(jm)), mean(xb(jm))];
  y = ((gp - bp) - (gm - bm)) / 2;

  [tm, mth, mx] = resposta(a, Ad, Bd, Ke, Ka, C, Ts, Vsat, T_JANELA);
  pe = max(abs(y(:,1)));  pm = max(abs(mth));
  qe = max(abs(y(:,2)));  qm = max(abs(mx));
  printf('%6.1f %7.2f   %8.2f %8.2f %6.2f    %8.1f %8.1f %6.2f\n', ...
         a, alfa*a, pe, pm, pe/pm, qe, qm, qe/qm);

  ENS{end+1} = y;  MOD{end+1} = [tm mth mx];  LAB{end+1} = sprintf('%.0f V', a);
end

if max(abs(ub)) > 9.9
  printf('\nATENCAO: o comando encostou nos %g V. Acima disso a comparacao com\n', Vsat);
  printf('o modelo linear deixa de valer; reduza a amplitude dos pulsos.\n');
end

% ------------------------------------------------------- figura do registro -
fg = figure();
a1 = subplot(3,1,1);  plot(tb, thb, 'b-');  grid on;  ylabel('Angulo [grau]');
title('Ensaio de perturbacao, registro completo');
a2 = subplot(3,1,2);  plot(tb, xb, 'b-');   grid on;  ylabel('Posicao [mm]');
a3 = subplot(3,1,3);  plot(tb, db, 'k-');   grid on;  ylabel('Pulso [V]');
xlabel('Tempo [s]');
set([a1 a2 a3], 'xlim', [0 tb(end)]);
print(fg, SAIDA, '-dpng', '-r150');

% ---------------------------------------------- figura das respostas limpas -
% Uma linha por amplitude: angulo a esquerda, posicao a direita, todas na mesma
% escala para que as linhas possam ser comparadas.
n   = numel(ENS);
lim_th = 1.1*max(cellfun(@(e, m) max([abs(e(:,1)); abs(m(:,2))]), ENS, MOD));
lim_x  = 1.1*max(cellfun(@(e, m) max([abs(e(:,2)); abs(m(:,3))]), ENS, MOD));
fz = figure();
for i = 1:n
  forca = strrep(sprintf('%.2f', alfa*sscanf(LAB{i}, '%f')), '.', ',');   % vírgula decimal
  subplot(n, 2, 2*i - 1);  hold on;
  hb = plot(tau, ENS{i}(:,1), 'b-', 'linewidth', 1.0);
  hm = plot(MOD{i}(:,1), MOD{i}(:,2), 'r-', 'linewidth', 1.4);
  hold off;  grid on;  ylabel('Ângulo [grau]');
  set(gca, 'xlim', [0 T_JANELA], 'ylim', [-lim_th lim_th]);
  title(sprintf('Pulso de %s (%s N): ângulo', LAB{i}, forca));
  if i == 1, legend([hb hm], {'Bancada', 'Modelo'}, 'location', 'northeast'); end
  if i == n, xlabel('Tempo após o pulso [s]'); end

  subplot(n, 2, 2*i);  hold on;
  plot(tau, ENS{i}(:,2), 'b-', 'linewidth', 1.0);
  plot(MOD{i}(:,1), MOD{i}(:,3), 'r-', 'linewidth', 1.4);
  hold off;  grid on;  ylabel('Posição [mm]');
  set(gca, 'xlim', [0 T_JANELA], 'ylim', [-lim_x lim_x]);
  title(sprintf('Pulso de %s (%s N): posição do carro', LAB{i}, forca));
  if i == n, xlabel('Tempo após o pulso [s]'); end
end
print(fz, SAIDA_RSP, '-dpng', '-S1000,1200');

printf('\nfiguras em %s e %s\n', SAIDA, SAIDA_RSP);

printf('\n  imagens gravadas em %s\n\n', FIGURAS);
cd(PASTA);     % volta para a pasta da rotina
