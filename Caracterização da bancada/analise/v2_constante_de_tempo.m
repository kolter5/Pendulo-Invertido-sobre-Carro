% v2  Constante de tempo do carro, do ensaio 3.
%
% Com a haste removida, o carro responde a um degrau de tensao como sistema de
% primeira ordem:  v(t) = v_ss (1 - exp(-t/tau)).  Ajustando o logaritmo de
% (1 - v/v_ss) contra o tempo, a inclinacao vale -1/tau.
clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS = fullfile(PASTA, '..', 'dados');


V0 = 1.91;                                   % tensao de limiar, do proprio ensaio
arq = fullfile(DADOS, 'ensaio3_patamares_de_tensao.csv');
fid = fopen(arq);
C = textscan(fid, '%s %f %f %f %f %f', 'Delimiter', ',', 'HeaderLines', 1);
fclose(fid);
fase = C{1}; sent = C{2}; pwm = C{3}; t = C{4}; v = C{6};
eh = strcmp(fase, 'degrau');

printf('%-7s %-5s %-8s %-9s %-9s\n', 'sentido', 'pwm', 'V-V0', 'v_ss', 'tau [s]');
tau = [];
for s = [1 -1]
  for p = unique(pwm(eh))'
    k = eh & sent == s & pwm == p;
    if sum(k) < 20, continue; end
    tt = t(k) - t(find(k, 1));
    vv = abs(v(k));
    vss = mean(vv(end-14:end));
    j = vv > 0.03*vss & vv < 0.92*vss;       % descarta o inicio e a cauda
    if sum(j) < 8, continue; end
    c = polyfit(tt(j), log(max(1 - vv(j)/vss, 1e-6)), 1);
    tau(end+1) = -1/c(1);
    printf('%-7d %-5d %-8.2f %-9.4f %-9.4f\n', s, p, p*12/255 - V0, vss, tau(end));
  end
end
printf('\ntau medio = %.4f s   (desvio %.4f entre os %d degraus)\n', ...
       mean(tau), std(tau), numel(tau));
save('-text', fullfile(PASTA, 'tau.txt'), 'tau');
