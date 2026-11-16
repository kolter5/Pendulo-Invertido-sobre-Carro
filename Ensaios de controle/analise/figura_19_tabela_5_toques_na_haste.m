% Valida a rejeicao do torque na haste, o canal nao casado do w.

1;

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS   = fullfile(PASTA, '..', 'dados');
FIGURAS = fullfile(PASTA, '..', 'figuras');
if ~exist(FIGURAS, 'dir'), mkdir(FIGURAS); end
cd(FIGURAS);   % o print grava pelo nome curto: o Ghostscript nao aceita
              % o acento de 'Area de Trabalho' no caminho completo


% Primeiro instante a partir do qual o sinal fica dentro da faixa por pelo
% menos o tempo pedido. Devolve NaN se isso nunca acontece na janela. Serve
% para medir recuperacao: o criterio da ultima saida da faixa mediria a
% oscilacao residual de 1 Hz, que cruza o limiar muito depois de a haste ter
% voltado.
function s = dentro(t, y, faixa, firme)
  s = NaN;
  n = numel(t);
  for k = 1:n
    if abs(y(k)) <= faixa
      j = find(t >= t(k) + firme, 1);
      if isempty(j), j = n; end
      if all(abs(y(k:j)) <= faixa)
        s = t(k);
        return
      end
    end
  end
end

ARQUIVO = fullfile(DADOS, 'toques_na_haste.csv');
SAIDA   = 'figura_19_toques_na_haste.png';

TH_GATILHO = 1.5;          % inclinacao que caracteriza um peteleco [grau]
T_QUIETO   = 1.5;          % quanto a haste precisa estar quieta antes [s]
T_JANELA   = 8.0;          % quanto da recuperacao se analisa [s]
T_ZERO     = 1.0;          % quanto antes do golpe define o zero do carro [s]

pkg load control

% Parametros medidos na bancada, Tabela 2.
m    = 0.178;      % massa da haste [kg]
L    = 0.41261;    % articulacao ate o centro de gravidade [m]
I    = 0.00382;    % inercia da haste em relacao ao centro de gravidade [kg.m2]
g    = 9.81;       % gravidade [m/s2]
Me   = 0.7071;     % massa efetiva do carro [kg]
alfa = 1.239;      % ganho do acionamento [N/V]
beta = 9.670;      % amortecimento eletromagnetico [N.s/m]
Ts   = 0.010;      % periodo de amostragem [s]
Vsat = 10;         % saturacao usada no firmware [V]

mL    = m*L;
Ip    = I + m*L^2;
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
tb = Bm(:,1);  thb = Bm(:,3)*180/pi;  xb = Bm(:,5)*1000;
thpb = Bm(:,4);  xpb = Bm(:,6);  xib = Bm(:,7);  ub = Bm(:,8);
Tb = tb(2) - tb(1);

% ------------------------------------------------------ acha os petelecos ---
% um peteleco e uma inclinacao acima do gatilho depois de um periodo quieto
base = median(thb);
alto = abs(thb - base) > TH_GATILHO;
ini = [];
for k = 2:numel(tb)
  if alto(k) && ~alto(k-1)
    jq = tb >= tb(k) - T_QUIETO & tb < tb(k) - 0.1;
    if sum(jq) > 20 && max(abs(thb(jq) - base)) < TH_GATILHO*0.6
      if isempty(ini) || tb(k) - tb(ini(end)) > T_JANELA
        ini(end+1) = k;
      end
    end
  end
end

if isempty(ini)
  printf('\nnenhum peteleco acima de %.1f grau encontrado.\n', TH_GATILHO);
  printf('o maior desvio do registro foi de %.2f grau.\n', max(abs(thb - base)));
  printf('se os petelecos foram mais fracos, baixe o TH_GATILHO.\n');
  return
end
printf('\n%d petelecos encontrados\n', numel(ini));

% Cada janela de analise termina quando vem o peteleco seguinte, senao a
% recuperacao que se mede ja esta contaminada pelo proximo golpe. Um registro
% com golpes emendados produz numeros que parecem validos e nao sao.
livre = repmat(T_JANELA, numel(ini), 1);
for i = 1:numel(ini)-1
  livre(i) = min(T_JANELA, tb(ini(i+1)) - tb(ini(i)) - 0.1);
end
livre(end) = min(T_JANELA, tb(end) - tb(ini(end)));
if sum(livre < T_JANELA - 0.05) > 0
  printf('ATENCAO: %d deles tem menos de %.1f s livres ate o golpe seguinte.\n', ...
         sum(livre < T_JANELA - 0.05), T_JANELA);
  printf('A recuperacao so pode ser medida ate onde a janela e limpa.\n');
end

printf('\n%4s %7s %8s %7s  %-19s  %-21s\n', 'n', 'inicio', 'pico', ...
       'janela', 'volta a 0,5 grau [s]', 'excursao do carro [mm]');
printf('%4s %7s %8s %7s  %8s %8s  %9s %9s\n', '', '[s]', '[grau]', '[s]', ...
       'ensaio', 'modelo', 'ensaio', 'modelo');
printf('%s\n', repmat('-', 1, 82));

ENS = {};  MOD = {};  PIC = [];
for i = 1:numel(ini)
  % o pico da inclinacao, onde a velocidade angular passa por zero
  faixa = tb >= tb(ini(i)) & tb < tb(ini(i)) + 1.0;
  seg = find(faixa);
  [~, kp] = max(abs(thb(faixa) - base));
  k0 = seg(kp);
  t0 = tb(k0);

  % Condicao inicial. A planta parte do pico: angulo medido e velocidade
  % angular nula, que e o que define o pico. O observador parte do que ele
  % acreditava naquele instante, ou seja, das velocidades que o firmware
  % registrou. A diferenca entre os dois e o erro de estimacao que o peteleco
  % deixou, e faz parte do que se quer medir.
  x0  = [ (thb(k0) - base)*pi/180; 0;          xb(k0)/1000; xpb(k0) ];
  xh0 = [ (thb(k0) - base)*pi/180; thpb(k0);   xb(k0)/1000; xpb(k0) ];
  xi0 = xib(k0);

  N = round(livre(i)/Ts);
  ths = zeros(N,1);  xs = zeros(N,1);
  xx = x0;  xh = xh0;  xi = xi0;
  for k = 1:N
    ths(k) = xx(1)*180/pi;  xs(k) = xx(3)*1000;
    vc = -Ka*[xh; xi];  v = max(-Vsat, min(Vsat, vc));
    e = -xx(3);
    if !((v != vc) && (sign(-Ka(5)*e) == sign(vc))), xi = xi + Ts*e; end
    xh = Ad*xh + Bd*v + Ke*(C*xx - C*xh);
    xx = Ad*xx + Bd*v;
  end
  tm = (0:N-1)'*Ts;

  % O zero do carro e onde ele estava ANTES do golpe, e nao no instante do
  % pico do angulo: naquele momento o carro ja havia comecado a correr, e
  % tomar aquilo como origem faz a curva parecer que nunca volta.
  x0 = mean(xb(tb >= t0 - T_ZERO & tb < t0));
  jb = tb >= t0 & tb < t0 + livre(i);
  te = tb(jb) - t0;  ye = thb(jb) - base;  xe = xb(jb) - x0;

  % Tempo ate a inclinacao entrar em meio grau e ali ficar por meio segundo.
  % O criterio anterior, a ultima saida da faixa, media a oscilacao residual
  % de 1 Hz e nao a recuperacao.
  ae = dentro(te, ye, 0.5, 0.5);
  am = dentro(tm, ths, 0.5, 0.5);

  aviso = '';
  if livre(i) < 1.2
    aviso = '  janela curta demais';
  elseif abs(ye(1)) > 6.0
    aviso = '  fora da faixa linear';
  elseif livre(i) < T_JANELA - 0.05
    aviso = '  janela cortada';
  end
  % a simulacao parte do carro onde ele estava no pico, entao o deslocamento
  % dela conta a partir do mesmo zero do ensaio
  xsd = xs - xs(1) + (xb(k0) - x0);
  % Na tabela, o deslocamento conta a partir do instante inicial da simulacao,
  % porque e o que o modelo de fato preve a partir daquela condicao. Contar a
  % partir de antes do toque incluiria o trecho anterior ao pico, que e igual
  % nos dois por construcao e tornaria a comparacao vazia. Na figura vale o
  % zero de antes do toque, que mostra o carro voltando a origem.
  printf('%4d %7.1f %8.2f %7.2f  %8.1f %8.1f  %9.1f %9.1f%s\n', i, t0, ye(1), ...
         livre(i), ae, am, max(abs(xb(jb) - xb(k0))), max(abs(xs - xs(1))), aviso);

  ENS{end+1} = [te ye xe];  MOD{end+1} = [tm ths xsd];  PIC(end+1) = ye(1);
end

printf('\npicos aplicados: ');  printf('%+.2f ', PIC);  printf('grau\n');
bons = sum(abs(PIC(:)) <= 6 & livre(1:numel(PIC)) >= 1.2);
printf('aproveitaveis para comparar com o modelo: %d de %d\n', bons, numel(PIC));
if bons < 3
  printf('\nE pouco. Para a comparacao valer, repita o ensaio com golpes de\n');
  printf('2 a 5 graus, espacados de 8 a 10 s, deixando a haste assentar\n');
  printf('entre um e outro.\n');
end

% ---------------------------------------------------------------- figura -----
% Uma linha por toque: angulo a esquerda, deslocamento do carro a direita, todas
% na mesma escala para que as linhas possam ser comparadas.
% Entram na figura so os toques com a janela inteira; o ultimo do registro
% termina antes dos T_JANELA segundos e fica apenas na tabela.
idx = find(cellfun(@(e) e(end,1) >= 0.95*T_JANELA, ENS));
n   = numel(idx);
lim_th = 1.1*max(cellfun(@(e, m) max([abs(e(:,2)); abs(m(:,2))]), ENS(idx), MOD(idx)));
lim_x  = 1.1*max(cellfun(@(e, m) max([abs(e(:,3)); abs(m(:,3))]), ENS(idx), MOD(idx)));
fg = figure();
for k = 1:n
  i = idx(k);
  subplot(n, 2, 2*k - 1);  hold on;
  hb = plot(ENS{i}(:,1), ENS{i}(:,2), 'b-', 'linewidth', 1.0);
  hm = plot(MOD{i}(:,1), MOD{i}(:,2), 'r-', 'linewidth', 1.4);
  hold off;  grid on;  ylabel('Ângulo [grau]');
  set(gca, 'xlim', [0 T_JANELA], 'ylim', [-lim_th lim_th], 'ytick', -6:3:6);
  title(sprintf('Toque %d: ângulo', i));
  if k == 1, legend([hb hm], {'Bancada', 'Modelo'}, 'location', 'northeast'); end
  if k == n, xlabel('Tempo a partir da inclinação máxima [s]'); end

  subplot(n, 2, 2*k);  hold on;
  plot(ENS{i}(:,1), ENS{i}(:,3), 'b-', 'linewidth', 1.0);
  plot(MOD{i}(:,1), MOD{i}(:,3), 'r-', 'linewidth', 1.4);
  hold off;  grid on;  ylabel('Carro [mm]');
  set(gca, 'xlim', [0 T_JANELA], 'ylim', [-lim_x lim_x]);
  title(sprintf('Toque %d: deslocamento do carro', i));
  if k == n, xlabel('Tempo a partir da inclinação máxima [s]'); end
end
print(fg, SAIDA, '-dpng', '-S1000,1400');
printf('\nfigura em %s\n', SAIDA);

printf('\n  imagens gravadas em %s\n\n', FIGURAS);
cd(PASTA);     % volta para a pasta da rotina
