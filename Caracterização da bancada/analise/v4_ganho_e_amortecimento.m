% v4  Ganho do acionamento e amortecimento, alfa e beta.

% O ensaio 3 entrega as razoes Me/alfa e beta/alfa. Para separar alfa de Me e
% preciso uma medida independente da massa efetiva:

% Me = M + J/r^2

% M e a massa do carro, obtida por pesagem, e J a inercia do rotor, da polia e
% do redutor referida ao eixo da polia. Com Me conhecida:

% alfa = Me / (Me/alfa) beta = (beta/alfa) * alfa
clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS = fullfile(PASTA, '..', 'dados');

if ~exist(fullfile(PASTA, 'razoes.txt'), 'file')
  error('rode antes o v3_razoes_do_acionamento');
end
load(fullfile(PASTA, 'razoes.txt'));
Me = 0.707; % pesagem do carro mais inercia do eixo

alfa = Me / razoes.Ma;
beta = razoes.ba * alfa;

printf('do ensaio 3:  Me/alfa = %.4f V.s2/m    beta/alfa = %.2f V.s/m\n', ...
       razoes.Ma, razoes.ba);
printf('da pesagem :  Me      = %.3f kg\n\n', Me);
printf('alfa = Me/(Me/alfa)        = %.3f N/V      <-- Secao 3.4: 1,24\n', alfa);
printf('beta = (beta/alfa) alfa    = %.2f N.s/m   <-- Secao 3.4: 9,67\n', beta);
printf('\nvelocidade maxima do carro a 12 V: %.2f m/s\n', (12 - 1.91)*alfa/beta);
save('-text', fullfile(PASTA, 'alfa_beta.txt'), 'alfa', 'beta', 'Me');
