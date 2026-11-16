% Compara, no tempo, os cinco ganhos do observador ensaiados na bancada.

clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS   = fullfile(PASTA, '..', 'dados');
FIGURAS = fullfile(PASTA, '..', 'figuras');
if ~exist(FIGURAS, 'dir'), mkdir(FIGURAS); end
cd(FIGURAS);   % o print grava pelo nome curto: o Ghostscript nao aceita
              % o acento de 'Area de Trabalho' no caminho completo


SAIDA = 'figura_17_polos_do_observador.png';

ENS = {'onda60_polos_26_a_32.csv',                'polos de -26 a -32';
       'onda60_polos_38_a_44.csv',  'polos de -38 a -44 (projeto inicial)';
       'onda60_polos_55_a_64.csv',              'polos de -55 a -64 (adotado)';
       'onda60_polos_90_a_105.csv',                    'polos de -90 a -105';
       'onda60_polos_115_a_133.csv',                  'polos de -115 a -133'};

JAN_ANG = [12 20];          % um patamar inteiro da onda [s]
JAN_TEN = [14 15];          % um segundo, para ver o tremor do comando [s]
LIM_ANG = [-2.5 2.5];       % mesma escala nos tres [grau]
LIM_TEN = [-6 6];           % mesma escala nos tres [V]

ENS(:,1) = cellfun(@(a) fullfile(DADOS, a), ENS(:,1), 'UniformOutput', false);
fg = figure();

for i = 1:rows(ENS)
  if ~exist(ENS{i,1}, 'file')
    printf('nao achei %s\n', ENS{i,1});
    return
  end
  M = dlmread(ENS{i,1}, ',', 1, 0);
  t = M(:,1);  th = M(:,3)*180/pi;  u = M(:,8);

  ja = t >= JAN_ANG(1) & t <= JAN_ANG(2);
  jt = t >= JAN_TEN(1) & t <= JAN_TEN(2);
  trocas = sum(abs(diff(sign(u(jt)))) > 0) / diff(JAN_TEN);
  printf('%-30s  angulo eficaz no patamar %.3f grau   trocas de sinal %.0f por s\n', ...
         ENS{i,2}, std(th(ja)), trocas);

  subplot(rows(ENS), 2, 2*i - 1);
  plot(t(ja), th(ja), 'b-', 'linewidth', 1.0);
  grid on;  set(gca, 'xlim', JAN_ANG, 'ylim', LIM_ANG);
  ylabel('Angulo [grau]');
  title(ENS{i,2});
  if i == rows(ENS), xlabel('Tempo [s]'); end

  subplot(rows(ENS), 2, 2*i);
  plot(t(jt), u(jt), 'b-', 'linewidth', 1.0);
  grid on;  set(gca, 'xlim', JAN_TEN, 'ylim', LIM_TEN);
  ylabel('Tensao [V]');
  title(ENS{i,2});
  if i == rows(ENS), xlabel('Tempo [s]'); end
end

print(fg, SAIDA, '-dpng', '-S1000,1350');
printf('\nfigura em %s\n', SAIDA);

printf('\n  imagens gravadas em %s\n\n', FIGURAS);
cd(PASTA);     % volta para a pasta da rotina
