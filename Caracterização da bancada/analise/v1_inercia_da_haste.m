% v1  Momento de inercia da haste, do ensaio 1.

clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS = fullfile(PASTA, '..', 'dados');

m = 0.178;  L = 0.41261;  g = 9.81; % pesagem e medicao direta

arq = fullfile(DADOS, 'ensaio1_oscilacao_livre_da_haste.csv');
d  = dlmread(arq, ',', 1, 0); % colunas: alvo, t, theta
t  = d(:,2);  th = d(:,3);
phi = th - pi; % desvio da posicao pendente

% cruzamentos por zero descendentes, com interpolacao linear entre amostras
k = find(phi(1:end-1) > 0 & phi(2:end) <= 0);
f = phi(k) ./ (phi(k) - phi(k+1));
tz = t(k) + f .* (t(k+1) - t(k));
T = diff(tz);

% amplitude de cada ciclo, para separar os de amplitude pequena
amp = zeros(numel(T), 1);
for i = 1:numel(T)
  amp(i) = max(abs(phi(k(i):k(i+1))));
end

Tp = mean(T(end-29:end));% ultimos 30 ciclos, amplitude pequena
w0 = 2*pi/Tp;
Ip = m*g*L/w0^2;
I  = Ip - m*L^2;

printf('ciclos detectados            : %d\n', numel(T));
printf('amplitude inicial            : %.1f graus\n', amp(1)*180/pi);
printf('amplitude dos ultimos ciclos : %.1f graus\n', mean(amp(end-29:end))*180/pi);
printf('periodo para amplitude pequena: %.4f s\n', Tp);
printf('frequencia natural w0        : %.4f rad/s\n', w0);
printf('\nIp = m g L / w0^2            = %.6f kg.m2\n', Ip);
printf('I  = Ip - m L^2              = %.6f kg.m2   <-- Tabela 2: 0,00382\n', I);

% o periodo cresce com a amplitude: so o modelo nao linear preve isso
printf('\nperiodo em funcao da amplitude:\n');
for faixa = [1 5; 6 20; 21 60; 61 numel(T)]'
  a = faixa(1); b = min(faixa(2), numel(T));
  printf('   ciclos %3d a %3d: %.1f graus  ->  T = %.4f s\n', ...
         a, b, mean(amp(a:b))*180/pi, mean(T(a:b)));
end
printf('o modelo linearizado preve %.4f s para qualquer amplitude\n', Tp);

% o que esta medida determina bem, e o que nao determina
printf('\nIp medido difere do valor adotado em %.1f%%\n', 100*(Ip - 0.034382)/0.034382);
printf('mas I = Ip - m L^2 e a diferenca de dois numeros proximos:\n');
printf('   m L^2 = %.6f responde por %.0f%% de Ip\n', m*L^2, 100*m*L^2/Ip);
printf('entao um erro de 1%% em Ip vira cerca de %.0f%% em I\n', Ip/(Ip - m*L^2));
printf('a grandeza que o ensaio determina bem e Ip, que e a que entra no modelo\n');
