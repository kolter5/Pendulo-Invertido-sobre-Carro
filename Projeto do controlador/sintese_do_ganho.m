% Calcula o ganho do controlador H-infinito.
%
% Este arquivo existe para ser rodado SOZINHO, quando algum parametro da
% bancada ou alguma ponderacao mudar. Ele imprime no fim a linha pronta para
% ser colada no d_hinf.m, que e onde o ganho e usado.
%
% Fora daqui, nenhum outro codigo refaz a sintese: todos usam o valor gravado.
clc; clear; pkg load control

% Parametros medidos na bancada, Tabela 2 do TCC.
m = 0.178; % massa da haste [kg]
L = 0.41261; % articulacao ate o centro de gravidade [m]
I = 0.00382; % inercia da haste em relacao ao centro de gravidade [kg.m2]
g = 9.81; % gravidade [m/s2]
Me = 0.7071; % massa efetiva do carro [kg]
alfa = 1.239; % ganho do acionamento [N/V]
beta = 9.670; % amortecimento eletromagnetico [N.s/m]

% Modelo linearizado da equacao (2.36).
mL = m*L;
Ip = I + m*L^2;
Delta = (Me + m)*I + Me*m*L^2;

A = [ 0                    1   0   0
      (Me+m)*m*g*L/Delta   0   0   mL*beta/Delta
      0                    0   0   1
     -mL*m*g*L/Delta       0   0  -Ip*beta/Delta ];

BF = [0; -mL/Delta;    0;  Ip/Delta];   % forca no carro [N]
BT = [0; (Me+m)/Delta; 0; -mL/Delta];   % torque na haste [N.m]
B  = alfa*BF;                           % tensao [V]

% Planta generalizada da equacao (2.38), ja aumentada com o integrador.
% B1 descreve a perturbacao, B2 a entrada de controle.
Cx = [0 0 1 0];                         % seleciona a posicao do carro
Aa = [A, zeros(4,1); -Cx, 0];
B2 = [B; 0];
B1 = [1.0*BF, 0.05*BT; 0, 0];           % forca de 1 N e torque de 0,05 N.m

% Saidas de desempenho.
w   = [0.10, 0.05, 0.05, 1.0]; % theta [rad], x [m], xi [m.s], u [V]
C1  = [1/w(1) 0 0 0 0
       0 0 1/w(2) 0 0
       0 0 0 0 1/w(3)
       0 0 0 0 0];
D12 = [0; 0; 0; 1/w(4)];
R   = D12'*D12;

% A bissecao. Cada passo faz uma pergunta de sim ou nao: existe controlador
% que garanta amplificacao abaixo deste gamma? Para gamma grande existe; para
% gamma pequeno nao. Oitenta passos encontram a fronteira entre os dois casos.
%
% O passo 81 e o unico que produz resultado aproveitavel: ele refaz a conta
% com o dobro do menor gamma, que e o valor adotado. O controlador da
% fronteira seria agressivo demais, com 277 V/rad, e saturaria o motor com
% 2,5 graus de inclinacao.
lo = 1e-3;  hi = 1e3;  n = 5;
for passo = 1:81
  if passo <= 80
    gam = sqrt(lo*hi); % meio do intervalo em escala log
  else
    gmin = hi;  gam = 2*gmin;
  end

  % Monta a matriz hamiltoniana para este gamma. Ela carrega a equacao de
  % Riccati Aa'X + XAa + C1'C1 + X(B1B1'/gam^2 - B2 R^-1 B2')X = 0 numa forma
  % que se resolve por autovetores.
  Rt = B1*B1'/gam^2 - B2*(R\B2');
  H  = [Aa, Rt; -C1'*C1, -Aa'];
  [V_, E_] = eig(H);
  e_ = diag(E_);

  % Quatro verificacoes. Falhando qualquer uma, este gamma nao serve.
  ok = false;  Ka = [];
  if all(abs(real(e_)) >= 1e-8)         % 1. nada em cima do eixo imaginario
    sel = real(e_) < 0;
    if nnz(sel) == n                    % 2. metade dos autovalores e estavel
      V1 = V_(1:n, sel);  V2 = V_(n+1:end, sel);
      if rcond(V1) >= 1e-12             % 3. V1 inversivel
        X = real(V2/V1);  X = (X + X')/2;
        if min(eig(X)) >= -1e-9         % 4. solucao positiva semidefinida
          Ka = R\(B2'*X);               % daqui sai o ganho
          ok = max(real(eig(Aa - B2*Ka))) < 0;   % e ele estabiliza mesmo?
        end
      end
    end
  end

  if passo <= 80
    if ok, hi = gam; else, lo = gam; end   % serviu? tente menor. falhou? aceite maior
  end
end

printf('\ngamma minimo  = %.4f\n', gmin);
printf('gamma adotado = %.4f   (o dobro)\n\n', gam);
printf('K  = [%.2f  %.2f  %.2f  %.2f]\n', Ka(1:4));
printf('kI = %.2f\n', -Ka(5));
printf('\nCole a linha abaixo no e_hinf.m, no lugar da que esta la:\n\n');
printf('Ka = [%.4f %.4f %.4f %.4f %.4f];\n\n', Ka);
