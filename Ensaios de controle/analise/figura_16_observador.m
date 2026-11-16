
% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS   = fullfile(PASTA, '..', 'dados');
FIGURAS = fullfile(PASTA, '..', 'figuras');
if ~exist(FIGURAS, 'dir'), mkdir(FIGURAS); end
cd(FIGURAS);   % o print grava pelo nome curto: o Ghostscript nao aceita
              % o acento de 'Area de Trabalho' no caminho completo

ARQUIVO   = fullfile(DADOS, 'onda60_polos_55_a_64.csv');
SAIDA     = 'figura_16_observador_100s.png';
SAIDA_ZUM = 'figura_16_observador.png';

T_ANALISE = 100.0;
T_ZOOM    = 2.0;    % largura da janela ampliada [s]
JAN       = 11;     % amostras da janela usada na derivada e na média

% Ensaio da bancada. As colunas são t, ref, theta, theta_p, x, x_p, xi, u, pwm.
Bm = dlmread(ARQUIVO, ',', 1, 0);
Bm = Bm(Bm(:,1) <= T_ANALISE, :);
tb  = Bm(:,1);
rb  = Bm(:,2)*1000;
thb = Bm(:,3); % ângulo medido [rad]
thp = Bm(:,4); % velocidade angular estimada [rad/s]
xb  = Bm(:,5); % posição medida [m]
xp  = Bm(:,6); % velocidade do carro estimada [m/s]

% O passo vem do próprio arquivo.
Tl = tb(2) - tb(1);
printf('\n  registro de %.0f s a %.0f Hz\n', tb(end)-tb(1), 1/Tl);

% Derivada por reta ajustada em uma janela deslizante. A diferença entre duas
% amostras seguidas amplifica a quantização do encoder, de 0,088 grau por
% pulso. Como a amostragem é uniforme, a reta vira um filtro de coeficientes
% fixos e sai de uma convolução.
i    = -floor(JAN/2):floor(JAN/2);
peso = i / (Tl * sum(i.^2));
dth  = conv(thb, fliplr(peso)(:), 'same');
dx   = conv(xb,  fliplr(peso)(:), 'same');

% A derivada carrega a suavização da janela, de 220 ms, que corta o que há
% acima de uns 2 Hz. A estimativa não tem esse corte, então ela é comparada
% também depois da mesma média móvel, senão a conta culpa o observador por uma
% diferença que é só do filtro.
mth = conv(thp, ones(JAN,1)/JAN, 'same');
mx  = conv(xp,  ones(JAN,1)/JAN, 'same');

% Descarta as pontas, onde a janela fica incompleta.
c = 20:(numel(tb)-20);
printf('\n  observador, contra a derivada do encoder\n');
printf('     velocidade angular  : erro rms %.4f rad/s, correlação %.3f\n', ...
       sqrt(mean((mth(c)-dth(c)).^2)), corr(mth(c), dth(c)));
printf('     velocidade do carro : erro rms %.4f m/s,   correlação %.3f\n', ...
       sqrt(mean((mx(c)-dx(c)).^2)), corr(mx(c), dx(c)));

% Janela ampliada em um patamar, longe das bordas da onda.
ib = find(abs(diff(rb)) > 1.0) + 1;
if numel(ib) > 1
  centro = 0.5*(tb(ib(1)) + tb(ib(2)));
else
  centro = 0.5*tb(end);
end
jan = [centro - T_ZOOM/2, centro + T_ZOOM/2];

% As duas figuras são iguais; a segunda só limita o eixo do tempo.
for caso = 1:2
  fg = figure();

  a1 = subplot(2,1,1);
  hold on;
  h1 = plot(tb, mth, 'b-',  'linewidth', 1.4);
  h2 = plot(tb, dth, 'r--', 'linewidth', 1.2);
  hold off;  grid on;
  ylabel('Vel. angular [rad/s]');
  legend([h1 h2], {'Estimada pelo observador', 'Derivada do encoder'}, ...
         'location', 'northeast');

  a2 = subplot(2,1,2);
  hold on;
  h1 = plot(tb, mx, 'b-',  'linewidth', 1);
  h2 = plot(tb, dx, 'r--', 'linewidth', 1);
  hold off;  grid on;
  xlabel('Tempo [s]');  ylabel('Vel. do carro [m/s]');
  legend([h1 h2], {'Estimada pelo observador', 'Derivada do encoder'}, ...
         'location', 'northeast');

  % Só os dois eixos do gráfico levam o limite de tempo; mexer em todos os
  % filhos da figura apagaria as legendas, que também são eixos.
  if caso == 1
    nome = SAIDA;
    set([a1 a2], 'xlim', [0 tb(end)]);
  else
    nome = SAIDA_ZUM;
    set([a1 a2], 'xlim', jan);
    % Na ampliação o eixo vertical também é refeito, com folga em cima para a
    % legenda, senão as curvas ficam achatadas na escala da corrida inteira.
    sel = tb >= jan(1) & tb <= jan(2);
    for par = {{a1, [mth(sel); dth(sel)]}, {a2, [mx(sel); dx(sel)]}}
      v = par{1}{2};
      alt = max(v) - min(v);
      set(par{1}{1}, 'ylim', [min(v) - 0.1*alt, max(v) + 0.6*alt]);
    end
  end

  destino = nome;
  print(fg, destino, '-dpng', '-S1000,560');
  printf('\n  %s', destino);
end
printf('\n\n');

printf('\n  imagens gravadas em %s\n\n', FIGURAS);
cd(PASTA);     % volta para a pasta da rotina
