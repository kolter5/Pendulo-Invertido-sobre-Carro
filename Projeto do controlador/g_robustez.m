% Robustez, Secao 4.5 do TCC.
% O mesmo ganho aplicado a 75 modelos. Com a tensao como entrada, o modelo
% depende principalmente das razoes Me/alfa e beta/alfa; as faixas cobrem a
% dispersao medida na caracterizacao.
clc; clear; pkg load control

% Parametros medidos na bancada, Tabela 2 do TCC.
m = 0.178; % massa da haste [kg]
L = 0.41261; % articulacao ate o centro de gravidade [m]
I = 0.00382; % inercia da haste em relacao ao centro de gravidade [kg.m2]
g = 9.81; % gravidade [m/s2]
Me = 0.7071; % massa efetiva do carro [kg]
alfa = 1.239; % ganho do acionamento [N/V]
beta = 9.670; % amortecimento eletromagnetico [N.s/m]
Ts = 0.010; % periodo de amostragem [s]
Vsat = 12; % saturacao da tensao [V]

% Modelo linearizado da equacao (2.32). Delta e o determinante da matriz de
% massa; a terceira coluna de A e nula porque nada restaura a posicao do carro.
mL    = m*L;
Ip    = I + m*L^2;
Delta = (Me + m)*I + Me*m*L^2;

A = [ 0                    1   0   0
      (Me+m)*m*g*L/Delta   0   0   mL*beta/Delta
      0                    0   0   1
     -mL*m*g*L/Delta       0   0  -Ip*beta/Delta ];

BF = [0; -mL/Delta;    0;  Ip/Delta];   % forca no carro [N]; sinais opostos
BT = [0; (Me+m)/Delta; 0; -mL/Delta];   % torque na haste [N.m]
B  = alfa*BF;                           % tensao [V]
C  = [1 0 0 0; 0 0 1 0];                % os dois encoders
D  = [0; 0];

% Discretizacao exata com segurador de ordem zero. O c2d calcula por dentro a
% exponencial da matriz aumentada; a formula Bd = A\(Ad - I)B nao serviria,
% porque A tem autovalor nulo e nao e inversivel.
sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
Ad = sysd.a;
Bd = sysd.b;

% Integrador do erro de posicao como quinto estado. No discreto a soma
% acumulada equivale a regra retangular.
Cx  = [0 0 1 0];
Aa  = [A, zeros(4,1); -Cx, 0];
B2  = [B; 0];
B1  = [1.0*BF, 0.05*BT; 0, 0];          % forca de 1 N e torque de 0,05 N.m

% Saidas de desempenho normalizadas pelo maior valor admitido de cada uma.
w   = [0.10, 0.05, 0.05, 1.0];          % theta [rad], x [m], xi [m.s], u [V]
C1  = [1/w(1) 0 0 0 0
       0 0 1/w(2) 0 0
       0 0 0 0 1/w(3)
       0 0 0 0 0];
D12 = [0; 0; 0; 1/w(4)];
R   = D12'*D12;

% Sintese. Resolve Aa'X + XAa + C1'C1 + X(B1B1'/gam^2 - B2 R^-1 B2')X = 0 pelo
% subespaco estavel da Hamiltoniana. Os 80 primeiros passos sao a bissecao que
% acha o menor gamma; o passo 81 refaz a conta com o gamma adotado, 2 gmin.
lo = 1e-3;  hi = 1e3;  n = 5;
for passo = 1:81
  if passo <= 80
    gam = sqrt(lo*hi);
  else
    gmin = hi;  gam = 2*gmin;
  end

  Rt = B1*B1'/gam^2 - B2*(R\B2');
  H  = [Aa, Rt; -C1'*C1, -Aa'];
  [V_, E_] = eig(H);
  e_ = diag(E_);
  ok = false;  X = [];  Ka = [];
  if all(abs(real(e_)) >= 1e-8) % nada no eixo imaginario
    sel = real(e_) < 0;
    if nnz(sel) == n
      V1 = V_(1:n, sel);  V2 = V_(n+1:end, sel);
      if rcond(V1) >= 1e-12
        X = real(V2/V1);  X = (X + X')/2;
        if min(eig(X)) >= -1e-9 % solucao semidefinida positiva
          Ka = R\(B2'*X);
          ok = max(real(eig(Aa - B2*Ka))) < 0;
        end
      end
    end
  end

  if passo <= 80
    if ok, hi = gam; else, lo = gam; end
  end
end
K  = Ka(1:4);
kI = -Ka(5);

Me_a = Me/alfa;  be_a = beta/alfa;
fMe = [-0.12 -0.06 0 0.06 0.12];   % dispersao de Me/alfa no ensaio 3 (Tabela 3)
fbe = [-0.04 -0.02 0 0.02 0.04];   % dispersao de beta/alfa no ensaio 3 (Tabela 3)
fal = [0.93 1.00 1.07];          % diferenca de 7% medida entre os sentidos

pior_polo = -Inf;  pior_norma = 0;  n_ = 0;  instaveis = 0;
queda = [];                      % polo instavel de malha aberta de cada modelo
Cf = C1 - D12*Ka;
for fa = fal
  for u = fMe
    for v = fbe
      al = alfa*fa;  Mv = Me_a*(1+u)*al;  bv = be_a*(1+v)*al;
      Dv = (Mv + m)*I + Mv*m*L^2;
      Av = [ 0                   1  0  0
             (Mv+m)*m*g*L/Dv     0  0  mL*bv/Dv
             0                   0  0  1
            -mL*m*g*L/Dv         0  0  -Ip*bv/Dv ];
      queda(end+1) = max(real(eig(Av)));
      BFv = [0; -mL/Dv; 0; Ip/Dv];
      BTv = [0; (Mv+m)/Dv; 0; -mL/Dv];
      Afv = [Av, zeros(4,1); -Cx, 0] - [al*BFv; 0]*Ka;
      B1v = [1.0*BFv, 0.05*BTv; 0, 0];
      p = max(real(eig(Afv)));
      n_++;
      if p >= 0, instaveis++; end
      pior_polo = max(pior_polo, p);
      for k = 1:400
        wk = 10^(-2 + 5*(k-1)/399);
        pior_norma = max(pior_norma, max(svd(Cf*((1i*wk*eye(5) - Afv)\B1v))));
      end
    end
  end
end

printf('%d combinacoes, %d instaveis\n', n_, instaveis);
printf('polo mais a direita : %.2f 1/s\n', pior_polo);
printf('maior norma de Tzw  : %.2f   (gamma = %.2f)\n', pior_norma, gam);
% Secao 4.1: quanto a queda da haste, o autovalor positivo de A, muda na faixa
printf('polo instavel de A  : %.3f a %.3f 1/s nos %d modelos\n', min(queda), max(queda), n_);
