% Parametros da haste pela geometria, como conferencia independente do ensaio.

clc; clear

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end
DADOS = fullfile(PASTA, '..', 'dados');

% tubo
Lt = 0.507; % comprimento, do eixo do encoder a face externa do peso(m)
De = 0.010; % diametro externo(m)
par = 0.0005; % espessura da parede(m)

% disco de aço na extremidade
md = 0.116; % massa (kg)
ed = 0.0101; % espessura [m]
dd = 0.502; % distancia do centro do disco ao eixo (m)

m_total = 0.178;  % massa da haste completa, por pesagem (kg)
rho = 7850; % massa especifica do aço (kg/m3)

% massa do tubo por dois caminhos: pela geometria e por diferença
V = pi*((De/2)^2 - (De/2 - par)^2)*Lt;
mt_geo = V*rho;
mt = m_total - md;

printf('massa do tubo pela geometria : %.4f kg\n', mt_geo);
printf('massa do tubo por diferenca  : %.4f kg   (diferenca de %.0f%%)\n', ...
mt, 100*(mt - mt_geo)/mt_geo);

% centro de gravidade do conjunto
L = (mt*Lt/2 + md*dd)/m_total;
printf('\ncentro de gravidade L = %.5f m   <-- Tabela 2: 0,4126\n', L);

% momento de inercia em relacao ao eixo do encoder
rd = sqrt(md/rho/(pi*ed)); % raio do disco, da massa e da espessura
Ip = mt*Lt^2/3 + md*rd^2/4 + md*dd^2; % barra pelo extremo + disco deslocado
printf('raio do disco = %.4f m\n', rd);
printf('Ip = %.6f kg.m2   <-- do ensaio (v1): 0,034121\n', Ip);
printf('   barra: %.6f   disco: %.6f (%.0f%% do total)\n', ...
       mt*Lt^2/3, md*rd^2/4 + md*dd^2, 100*(md*rd^2/4 + md*dd^2)/Ip);

I = Ip - m_total*L^2;
printf('\nI = Ip - m L^2 = %.6f kg.m2\n', I);
printf('o v1, pela frequencia medida, da 0,003817\n');
printf('os dois caminhos concordam dentro de %.1f%%\n', abs(100*(I - 0.003817)/0.003817));
