1;

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS   = fullfile(PASTA, '..', 'dados');

% Números das Seções 5.2 a 5.5: confronta o ensaio da bancada com a
% simulação. Lê o arquivo gravado pelo grava_controle.py e o simula_onda.csv
% produzido pelo simula_onda.m, que precisa ter sido rodado antes com a mesma
% amplitude e o mesmo período do ensaio.
%
% Basta trocar o nome em ARQUIVO e rodar.

ARQUIVO = fullfile(DADOS, 'onda60_polos_55_a_64.csv');

% Segundos descartados no fim do registro. A corrida termina com o operador
% segurando o carro ou a haste, e esses instantes nao sao do controle: a tensao
% satura e a haste cai porque o carro foi impedido de responder. Deixar esse
% trecho dentro das contas estraga todos os maximos.
CORTE_FIM = 1.0;

GRAU = 180/pi;
JAN  = 11;      % amostras da janela usada na derivada e na média

% Derivada por reta ajustada em uma janela deslizante. A diferença entre duas
% amostras seguidas amplifica a quantização do encoder, de 0,088 grau por
% pulso. Como a amostragem é uniforme, a reta vira um filtro de coeficientes
% fixos e sai de uma convolução.
function d = derivada(y, Ts, j)
  m = floor(j/2);
  i = (-m:m);
  w = i / (Ts * sum(i.^2));
  d = conv(y(:), fliplr(w)(:), 'same');
end

% Média móvel da mesma largura, para comparar o observador na mesma banda.
function s = media(y, j)
  s = conv(y(:), ones(j,1)/j, 'same');
end

% Instantes em que a referência muda de patamar.
function i = bordas(ref)
  i = find(abs(diff(ref)) > 1.0) + 1;
end

function metricas(nome, t, ref, x, th, u)
  ib = bordas(ref);
  printf('\n  %s\n', nome);
  if isempty(ib)
    printf('     sem borda de referência no registro\n');
    return
  end
  k0 = ib(1);
  if numel(ib) > 1, k1 = ib(2); else, k1 = numel(t); end
  alvo  = ref(k0);
  salto = abs(alvo - ref(k0-1));
  trecho = x(k0:k1);
  if alvo > ref(k0-1), pico = max(trecho); else, pico = min(trecho); end
  fora = find(abs(trecho - alvo) > 0.02*salto);
  printf('     sobressinal na 1a borda : %.0f %%\n', 100*abs(pico-alvo)/salto);
  if !isempty(fora) && fora(end) < numel(trecho) - 5
    printf('     acomodação em 2 %%       : %.1f s\n', t(k0+fora(end)) - t(k0));
  else
    printf('     acomodação em 2 %%       : não acomodou antes da borda seguinte\n');
  end
  printf('     erro médio no patamar   : %.1f mm\n', mean(x(k1-100:k1)) - alvo);
  printf('     inclinação máxima       : %.2f graus\n', max(abs(th(k0:end))));
  printf('     ângulo rms              : %.3f graus\n', sqrt(mean(th(k0:end).^2)));
  printf('     tensão máxima           : %.2f V\n', max(abs(u(k0:end))));
  printf('     tensão rms              : %.2f V\n', sqrt(mean(u(k0:end).^2)));
end

warning('off', 'all');

B = dlmread(ARQUIVO, ',', 1, 0);
S = dlmread(fullfile(PASTA, 'simula_onda.csv'), ',', 0, 0);

B = B(B(:,1) <= B(end,1) - CORTE_FIM, :);

tb = B(:,1);  rb = B(:,2)*1000;  thb = B(:,3)*GRAU;
thpb = B(:,4);  xb = B(:,5)*1000;  xpb = B(:,6);  ub = B(:,8);

ts = S(:,1);  rs = S(:,2)*1000;  ths = S(:,3)*GRAU;  xs = S(:,5)*1000;  us = S(:,7);

% A simulação é mais longa que o ensaio; corta no mesmo instante final.
corte = ts <= tb(end) + 1;
ts = ts(corte);  rs = rs(corte);  ths = ths(corte);  xs = xs(corte);  us = us(corte);

% O passo vem do próprio arquivo. O controle roda a 100 Hz dentro da
% interrupção, mas o loop só consegue imprimir uma amostra a cada 20 ms.
Ts = tb(2) - tb(1);
ib = bordas(rb);

printf('\n  registro de %.1f s, %d amostras a %.0f Hz, %d bordas de referência\n', ...
       tb(end)-tb(1), numel(tb), 1/Ts, numel(ib));

metricas('bancada', tb, rb, xb, thb, ub);
metricas('simulação', ts, rs, xs, ths, us);

% Quanto a posição medida se afasta da prevista, com os dados como vieram.
xsi = interp1(ts, xs, tb, 'linear', 'extrap');
c   = 60:(numel(tb)-60);
printf('\n  posição medida contra a simulação\n');
printf('     erro rms                : %.1f mm\n', sqrt(mean((xb(c)-xsi(c)).^2)));
printf('     maior desvio            : %.1f mm\n', max(abs(xb(c)-xsi(c))));

% Frequência do ciclo limite. A energia fica espalhada em uma faixa, então a
% maior raia sozinha não serve: o espectro é suavizado antes de procurar o pico.
n = numel(c);
f = (0:floor(n/2)) / (n*Ts);
P = abs(fft(xb(c)-xsi(c)));  P = P(1:numel(f));   % o que sobra da medida tirando o previsto
Ps = media(P, 21);  Ps(f < 0.5) = 0;   % abaixo de 0,5 Hz ficam as trocas de patamar
[~, k] = max(Ps);
printf('     frequência da oscilação : %.1f Hz\n', f(k));

Pu = abs(fft(ub(c)-mean(ub(c))));  Pu = Pu(1:numel(f));
[~, ku] = max(media(Pu, 21));
printf('     frequência da tensão    : %.1f Hz\n', f(ku));
printf('     trocas de sinal da tensão: %.1f por segundo\n', ...
       sum(diff(sign(ub)) != 0) / (tb(end)-tb(1)));

% Observador: a velocidade estimada contra a derivada do encoder (Seção 5.5).
dth = derivada(thb/GRAU, Ts, JAN);
dx  = derivada(xb/1000,  Ts, JAN);
mth = media(thpb, JAN);
mx  = media(xpb,  JAN);

% A derivada vem de uma janela de 220 ms, que corta o que há acima de uns 2 Hz.
% A estimativa não tem esse corte, então ela é comparada também depois da mesma
% média móvel.
d = 20:(numel(tb)-20);
nomes = {'velocidade angular', 'velocidade do carro'};
unid  = {'rad/s', 'm/s'};
est   = {thpb, xpb};
suav  = {mth, mx};
med   = {dth, dx};
for i = 1:2
  printf('\n  observador, %s\n', nomes{i});
  e1 = est{i}(d) - med{i}(d);
  e2 = suav{i}(d) - med{i}(d);
  printf('     estimativa direta : erro rms %.4f %s, correlação %.3f\n', ...
         sqrt(mean(e1.^2)), unid{i}, corr(est{i}(d), med{i}(d)));
  printf('     na mesma banda    : erro rms %.4f %s, correlação %.3f\n', ...
         sqrt(mean(e2.^2)), unid{i}, corr(suav{i}(d), med{i}(d)));
end
printf('\n');
