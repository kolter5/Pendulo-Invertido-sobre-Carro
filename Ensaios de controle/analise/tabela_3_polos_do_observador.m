% Tabela 3 do TCC, varredura dos polos do observador (Secao 5.6).
%
% Le os cinco ensaios com a onda de +-60 mm e mede, na mesma janela de 80 s,
% o angulo eficaz, a amplitude da oscilacao de 1 Hz, o erro de posicao, a
% tensao eficaz e as trocas de sinal do comando por segundo.
clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS   = fullfile(PASTA, '..', 'dados');


JANELA = 80.0;              % mesma janela nos cinco

ENS = {"onda60_polos_26_a_32.csv",                -26;
       "onda60_polos_38_a_44.csv",  -38;
       "onda60_polos_55_a_64.csv",              -55;
       "onda60_polos_90_a_105.csv",                    -90;
       "onda60_polos_115_a_133.csv",                 -115};

ADOTADO = 55;               % o conjunto escolhido, em modulo

ENS(:,1) = cellfun(@(a) fullfile(DADOS, a), ENS(:,1), 'UniformOutput', false);

n = rows(ENS);
pol = zeros(n,1);  th = zeros(n,1);  osc = zeros(n,1);
erx = zeros(n,1);  uef = zeros(n,1);  trc = zeros(n,1);

for i = 1:n
  if ~exist(ENS{i,1}, 'file')
    printf('nao achei %s\n', ENS{i,1});
    return
  end
  M = dlmread(ENS{i,1}, ',', 1, 0);
  M = M(M(:,1) <= JANELA, :);
  t = M(:,1);  a = M(:,3)*180/pi;  r = M(:,2)*1000;  x = M(:,5)*1000;  u = M(:,8);

  % erro de seguimento, fora dos 3 s que seguem cada degrau
  ib = find(abs(diff(r)) > 1.0) + 1;
  ok = t > 6;
  for k = ib(:)'
    ok = ok & !(t >= t(k) & t < t(k) + 3);
  end

  % amplitude da oscilacao residual, pelo pico do espectro acima de 0,3 Hz
  ac = a - mean(a);
  N = numel(ac);
  F = abs(fft(ac)) * 2/N;
  f = (0:N-1)'/(N*(t(2)-t(1)));
  fx = f > 0.3 & f < 10;
  Ff = F(fx);

  pol(i) = abs(ENS{i,2});
  th(i)  = std(a);
  osc(i) = max(Ff);
  erx(i) = sqrt(mean((x(ok) - r(ok)).^2));
  uef(i) = std(u);
  trc(i) = sum(abs(diff(sign(u))) > 0) / (t(end) - t(1));
end

j = find(pol == ADOTADO);
if isempty(j)
  printf('o conjunto adotado nao esta na lista\n');
  return
end

printf('\n%-8s %8s %8s %8s %8s %8s\n', 'polo', 'th ef', 'oscil', 'erro x', 'u ef', 'troc/s');
printf('%s\n', repmat('-', 1, 54));
for i = 1:n
  printf('%-8d %8.3f %8.3f %8.2f %8.3f %8.1f\n', ...
         -pol(i), th(i), osc(i), erx(i), uef(i), trc(i));
end
printf('\n');
