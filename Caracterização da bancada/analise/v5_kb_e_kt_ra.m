% v5  Constantes do motor, Kb e Kt/Ra.
% De (2.19) do TCC:
%     alfa = Kt/(r Ra)  ->  Kt/Ra = r alfa
%     beta = Kt Kb/(r^2 Ra) ->  Kb    = r beta/alfa
% O raio da polia vem da medicao direta do diametro, 50,38 mm, e e o mesmo
% valor usado na conversao de pulsos para posicao no firmware.

clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS = fullfile(PASTA, '..', 'dados');


if ~exist(fullfile(PASTA, 'alfa_beta.txt'), 'file')
  error('rode antes o v4_ganho_e_amortecimento');
end
load(fullfile(PASTA, 'alfa_beta.txt'));
r = 0.02519; % raio da polia [m]

KtRa = r * alfa;
Kb   = r * beta / alfa;

printf('r     = %.5f m\n', r);
printf('alfa  = %.3f N/V      beta = %.2f N.s/m\n\n', alfa, beta);
printf('Kb    = r beta/alfa        = %.4f V.s/rad   <-- Tabela 2: 0,197\n', Kb);
printf('Kt/Ra = r alfa             = %.4f N.m/V     <-- Tabela 2: 0,0312\n', KtRa);
printf('\nconferencia pelo caminho inverso:\n');
printf('   alfa = (Kt/Ra)/r        = %.3f N/V\n', KtRa/r);
printf('   beta = (Kt/Ra) Kb/r^2   = %.2f N.s/m\n', KtRa*Kb/r^2);
printf('\nsupondo Kt = Kb no SI, motor ideal: Ra = %.2f ohm, Kt = %.4f N.m/A\n', Kb/KtRa, Kb);
printf('(essa separacao exige medir Ra ou Kb de forma independente)\n');
