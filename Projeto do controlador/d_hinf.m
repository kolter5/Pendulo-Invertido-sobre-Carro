% Controlador H-infinito
clc; clear; pkg load control

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end


% Modelo, sistema aumentado e matrizes discretas, gravados pelo c_discreto.m.
load(fullfile(PASTA, 'modelo.mat'));

% Perturbacoes: forca no carro e torque na haste.
B1 = [1.0*BF, 0.05*BT; 0, 0]; % forca de 1 N e torque de 0,05 N.m

% Saidas de desempenho
w   = [0.10, 0.05, 0.05, 1.0]; % theta [rad], x [m], xi [m.s], u [V]
C1  = [1/w(1) 0 0 0 0
       0 0 1/w(2) 0 0
       0 0 0 0 1/w(3)
       0 0 0 0 0];
D12 = [0; 0; 0; 1/w(4)];
R   = D12'*D12;

% Ganho obtido pelo sintese_do_ganho.m, com gamma adotado de 6,1691, o dobro
% do minimo de 3,0845. A ordem e [theta, theta_p, x, x_p, xi].
Ka = [-114.1031 -24.4779 -57.9692 -43.3083 32.7786];

K  = Ka(1:4);
kI = -Ka(5);

% Norma H-infinito da malha fechada, por varredura em frequencia.
Af = Aa - B2*Ka;  Cf = C1 - D12*Ka;
wv = logspace(-2, 3, 4000);  nrm = 0;
for k = 1:numel(wv)
  nrm = max(nrm, max(svd(Cf*((1i*wv(k)*eye(5) - Af)\B1))));
end

printf('K  = [%.2f  %.2f  %.2f  %.2f]\n', K);
printf('kI = %.2f\n\n', kI);
printf('polos de malha fechada [1/s]: '); printf('%.2f  ', sort(real(eig(Af)))); printf('\n');
printf('norma de Tzw = %.2f   (gamma adotado 6,17)\n', nrm);
printf('maior |z| no discreto = %.4f\n', max(abs(eig(Ada - Bda*Ka))));
