% v3  As duas razoes que o ensaio 3 mede diretamente.
%
% Em regime,  alfa (va - V0) = beta v  ->  beta/alfa = (va - V0)/v_ss.
% No transitorio, a aceleracao inicial vale a0 = v_ss/tau, e da equacao do carro
% Me a0 = alfa (va - V0)  ->  Me/alfa = (va - V0)/a0.
%
% Essas duas razoes saem do registro sem supor nenhuma massa. A conferencia e
% que Me/alfa deve ser igual a tau vezes beta/alfa.

clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS = fullfile(PASTA, '..', 'dados');

V0 = 1.91; % tensao consumida pelo atrito (conferida no fim)
arq = fullfile(DADOS, 'ensaio3_patamares_de_tensao.csv');
fid = fopen(arq);
C = textscan(fid, '%s %f %f %f %f %f', 'Delimiter', ',', 'HeaderLines', 1);
fclose(fid);
fase = C{1}; sent = C{2}; pwm = C{3}; t = C{4}; v = C{6};
eh = strcmp(fase, 'degrau');

printf('%-7s %-5s %-9s %-13s %-13s\n', 'sentido', 'pwm', 'v_ss', 'beta/alfa', 'Me/alfa');
ba = []; Ma = []; tau = []; Vl = []; vl = [];
for s = [1 -1]
  for p = unique(pwm(eh))'
    k = eh & sent == s & pwm == p;
    if sum(k) < 20, continue; end
    tt = t(k) - t(find(k, 1));  vv = abs(v(k));
    vss = mean(vv(end-14:end));
    j = vv > 0.03*vss & vv < 0.92*vss;
    if sum(j) < 8, continue; end
    c = polyfit(tt(j), log(max(1 - vv(j)/vss, 1e-6)), 1);
    tt_ = -1/c(1);
    U = p*12/255 - V0;
    ba(end+1) = U/vss;  Ma(end+1) = U/(vss/tt_);  tau(end+1) = tt_;
    Vl(end+1) = p*12/255;  vl(end+1) = vss;
    printf('%-7d %-5d %-9.4f %-13.2f %-13.4f\n', s, p, vss, ba(end), Ma(end));
  end
end
printf('\nbeta/alfa = %.2f V.s/m    (desvio %.2f)\n', mean(ba), std(ba));
printf('Me/alfa   = %.4f V.s2/m  (desvio %.4f)\n', mean(Ma), std(Ma));
printf('\nconferencia  Me/alfa = tau * beta/alfa:  %.4f  contra  %.4f\n', ...
mean(Ma), mean(tau)*mean(ba));
% conferencia de V0: a reta da velocidade final em funcao da tensao cruza o zero
c = polyfit(Vl, vl, 1);
printf('conferencia  reta v_ss x tensao cruza o zero em %.2f V  (adotado %.2f V)\n', ...
-c(2)/c(1), V0);
razoes.ba = mean(ba);  razoes.Ma = mean(Ma);  razoes.tau = mean(tau);
save('-text', fullfile(PASTA, 'razoes.txt'), 'razoes');
