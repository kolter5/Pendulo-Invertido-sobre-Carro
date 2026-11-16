% Mesma rotina da pasta controle_hinf, com o observador de -55 a -64,

clc; clear; pkg load control

% Pastas usadas pela rotina, a partir de onde ela esta gravada.
PASTA = fileparts(mfilename('fullpath'));
if isempty(PASTA), PASTA = pwd; end


% Parâmetros medidos na bancada, Tabela 2.
m = 0.178; % massa da haste [kg]
L = 0.41261; % articulação até o centro de gravidade [m]
I = 0.00382; % inércia da haste em relação ao centro de gravidade [kg.m2]
g = 9.81; % gravidade [m/s2]
Me = 0.7071; % massa efetiva do carro [kg]
alfa = 1.239; % ganho do acionamento [N/V]
beta = 9.670; % amortecimento eletromagnético [N.s/m]
Ts   = 0.010; % período de amostragem do controle [s]
Vsat = 10; % saturação usada no firmware [V]

mL = m*L;
Ip = I + m*L^2;
Delta = (Me + m)*I + Me*m*L^2;

A = [ 0                    1   0   0
      (Me+m)*m*g*L/Delta   0   0   mL*beta/Delta
      0                    0   0   1
     -mL*m*g*L/Delta       0   0  -Ip*beta/Delta ];

BF = [0; -mL/Delta;    0;  Ip/Delta];
BT = [0; (Me+m)/Delta; 0; -mL/Delta];
B  = alfa*BF;
C  = [1 0 0 0; 0 0 1 0];
D  = [0; 0];

sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
Ad = sysd.a;
Bd = sysd.b;

% ---------------------------------------------------------------- sintese
Cx  = [0 0 1 0];
Aa  = [A, zeros(4,1); -Cx, 0];
B2  = [B; 0];
B1  = [1.0*BF, 0.05*BT; 0, 0];
w   = [0.10, 0.05, 0.05, 1.0];
C1  = [1/w(1) 0 0 0 0
       0 0 1/w(2) 0 0
       0 0 0 0 1/w(3)
       0 0 0 0 0];
D12 = [0; 0; 0; 1/w(4)];
R   = D12'*D12;

lo = 1e-3;  hi = 1e3;  n = 5;
for passo = 1:81
  if passo <= 80
    gam = sqrt(lo*hi);
  else
    gam = 2*hi;
  end

  Rt = B1*B1'/gam^2 - B2*(R\B2');
  H  = [Aa, Rt; -C1'*C1, -Aa'];
  [V_, E_] = eig(H);
  e_ = diag(E_);
  ok = false;  Ka = [];
  if all(abs(real(e_)) >= 1e-8)
    sel = real(e_) < 0;
    if nnz(sel) == n
      V1 = V_(1:n, sel);  V2 = V_(n+1:end, sel);
      if rcond(V1) >= 1e-12
        X = real(V2/V1);  X = (X + X')/2;
        if min(eig(X)) >= -1e-9
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

Ke = place(Ad', C', exp([-55 -58 -61 -64](:)*Ts))';

% ---------------------------------------------------------------- simulacao
N = round(T_FIM/Ts);
t = (0:N-1)'*Ts;
r = zeros(N,1);
for k = 1:N
  if t(k) >= T_INICIO
    fase = mod(t(k) - T_INICIO, T_ONDA);
    if fase < 0.5*T_ONDA, r(k) = A_ONDA; else, r(k) = -A_ONDA; end
  end
end

x  = [0; 0; 0; 0];
xh = [0; 0; 0; 0];
xi = 0;
xs = zeros(N,4);  hs = zeros(N,4);  vs = zeros(N,1);

for k = 1:N
  xs(k,:) = x';
  hs(k,:) = xh';
  vc = -Ka*[xh(1); xh(2); xh(3); xh(4); xi];
  v  = max(-Vsat, min(Vsat, vc));
  vs(k) = v;
  e = r(k) - x(3);
  if !((v != vc) && (sign(-Ka(5)*e) == sign(vc)))
    xi = xi + Ts*e;
  end
  xh = Ad*xh + Bd*v + Ke*(C*x - C*xh);
  x  = Ad*x  + Bd*v;
end

%resultados
th = xs(:,1)*180/pi;
xc = xs(:,3)*1000;

% primeira borda da onda: sobressinal e acomodacao
k0 = round(T_INICIO/Ts) + 1;
k1 = k0 + round(0.5*T_ONDA/Ts) - 1;
alvo = A_ONDA*1000;
pico = max(xc(k0:k1));
jan  = find(abs(xc(k0:k1) - alvo) > 0.02*alvo);

printf('\nsimulacao da onda quadrada de +-%.0f mm\n', A_ONDA*1000);
printf('   sobressinal          : %.0f %%\n', 100*(pico - alvo)/alvo);
printf('   acomodacao em 2 %%    : %.1f s\n', Ts*jan(end));
printf('   inclinacao maxima    : %.2f graus\n', max(abs(th(k0:end))));
printf('   angulo rms           : %.3f graus\n', sqrt(mean(th(k0:end).^2)));
printf('   tensao maxima        : %.2f V\n', max(abs(vs)));
printf('   tensao rms           : %.2f V\n', sqrt(mean(vs.^2)));

M = [t, r, xs(:,1), hs(:,2), xs(:,3), hs(:,4), vs];
csvwrite(fullfile(PASTA, 'simula_onda.csv'), M);
printf('\nsimula_onda.csv gravado, %d amostras\n', N);
