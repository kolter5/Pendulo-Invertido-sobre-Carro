// Felipe Kolter - Inverted Pendulum
// CONTROLE H-INFINITO COM OBSERVADOR DE ESTADOS

// pinos
#define x_A 18   // PD3  encoder do carro, canal A
#define x_B 19   // PD2  encoder do carro, canal B
#define P_A 2    // PE4  encoder do pendulo, canal A
#define P_B 3    // PE5  encoder do pendulo, canal B
#define FIM_1 4  // chaves de fim de curso, com pull-up: nivel baixo = acionada
#define FIM_2 5

int IN1 = 8;     // ponte H
int IN2 = 9;

// Ganhos do Capitulo 4, equacao (4.7).
// Ka = [K, -kI], na ordem [theta, dtheta, x, dx, xi].  u = -Ka * estado.
const float Ka[5] = {-114.10f, -24.48f, -57.97f, -43.31f, 32.78f};

// Modelo discretizado com Ts = 10 ms, usado pelo observador.
const float Ad[4][4] = {
  { 1.001276f,  0.010004f,  0.000000f,  0.001370f},
  { 0.254233f,  1.001276f,  0.000000f,  0.268167f},
  {-0.000102f, -0.000000f,  1.000000f,  0.009363f},
  {-0.019981f, -0.000102f,  0.000000f,  0.875449f}
};
const float Bd[4] = {-0.000176f, -0.034360f, 0.000082f, 0.015959f};

// Ganho do observador, polos em -26, -28, -30 e -32 1/s. 

const float Ke[4][2] = {
  { 0.587425f, 0.011542f},
  { 9.186766f, 0.480513f},
  { 0.011542f, 0.296773f},
  { 0.107732f, 0.751256f}
};

// ajustes da bancada
const float TS         = 0.010f;   // periodo de amostragem [s]
const float V_SAT      = 10.0f;    // maior tensao comandada [V]; com o limiar da 12
const float V_LIMIAR   = 2.6f;     // compensacao da zona morta [V]; o ensaio 3 mediu
                                   // 1,9 V de limiar, mas perto do equilibrio o comando
                                   // e pequeno e 1,9 V se mostrou insuficiente
const float ASSIMETRIA = 0.946f;   // diferenca de ganho entre os dois sentidos
const int   SENTIDO    = +1;       // +1 ou -1: ver a nota no fim do arquivo

const float TH_ARMA   = 3.0f  * PI / 180.0f;   // janela para armar
const float W_ARMA    = 0.5f;                  // rad/s, haste firme para armar
const float TH_ABORTA = 15.0f * PI / 180.0f;   // fora disso, desliga
const float X_ABORTA  = 0.35f;                 // curso maximo a partir do centro [m]

// referencia de posicao. Com ONDA ligado o carro segue uma onda quadrada, que
// e o ensaio de validacao; com ONDA desligado vale o degrau unico.
const float REF      = 0.0f;       // centro da referencia [m]
const bool  ONDA     = true;       // onda quadrada de +-A_ONDA
const float A_ONDA   = 0.060f;     // amplitude da onda [m]
const float T_ONDA   = 20.0f;      // periodo da onda [s]
const float T_INICIO = 6.0f;       // a onda comeca depois deste tempo [s]
const bool  DEGRAU   = false;      // degrau unico, como na Figura 13
const float T_DEGRAU = 4.0f;
const float A_DEGRAU = 0.050f;

// estado
volatile long valor_encoder_x = 0L;
volatile long ult_valor_encoder_x = 0L;
volatile long valor_encoder_P = 0L;
volatile long ult_valor_encoder_P = 0L;
volatile bool dentro = false;      // trava contra reentrada da ISR do controle

const float PI2 = 2.0f * PI;

float xh[4] = {0, 0, 0, 0};        // estado estimado pelo observador
float xi = 0.0f;                   // integrador do erro de posicao
float theta = 0.0f, x = 0.0f;      // medidos
float theta_ant = 0.0f;            // so para a checagem de armar
float u = 0.0f;                    // tensao comandada [V]
float t = 0.0f;                    // tempo desde o armamento [s]
bool  armado = false;
int   firme = 0;                   // ciclos dentro da janela de armamento
long  n = 0;

// Fotografia do ciclo, preenchida pela ISR e impressa pelo loop. A impressao
// nao fica na ISR porque a serial do Arduino tambem usa interrupcao.
volatile bool  tem_linha = false;
volatile bool  r_armado = false;   // false = linha de posicionamento, nao e dado
volatile float r_t, r_ref, r_theta, r_thp, r_x, r_xp, r_xi, r_u;

// conversoes
float obter_x(long pulsos) {
  return PI2 * pulsos * 0.02519f / 4096.0f;
}

float obter_angulo(long pulsos) {
  float ang = PI + PI2 * pulsos / 4096.0f;
  while (ang > PI)  ang -= PI2;
  while (ang < -PI) ang += PI2;
  return ang;
}

// acionamento, recebe a tensao desejada em volts
void Motor(float v) {
  if (v >  V_SAT) v =  V_SAT;
  if (v < -V_SAT) v = -V_SAT;

  if (fabs(v) < 0.02f) {                 // zona neutra, desliga os dois pinos
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    return;
  }

  float vm = fabs(v) + V_LIMIAR;         // compensa o limiar de acionamento
  if (vm > 12.0f) vm = 12.0f;
  int pwm = (int)(vm * 255.0f / 12.0f + 0.5f);
  if (pwm > 255) pwm = 255;

  if (v > 0) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, pwm);
  } else {
    digitalWrite(IN2, LOW);
    analogWrite(IN1, (int)(pwm * ASSIMETRIA));
  }
}

// encoders, sem mudanca em relacao ao firmware do LQR
void manipulador_encoder_x() {
  int MSB = (PIND & (1 << PD3)) >> PD3;
  int LSB = (PIND & (1 << PD2)) >> PD2;
  int encoded = (MSB << 1) | LSB;
  int sum = (ult_valor_encoder_x << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) valor_encoder_x--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) valor_encoder_x++;
  ult_valor_encoder_x = encoded;
}

void manipulador_encoder_P() {
  int MSB = (PINE & (1 << PE4)) >> PE4;
  int LSB = (PINE & (1 << PE5)) >> PE5;
  int encoded = (MSB << 1) | LSB;
  int sum = (ult_valor_encoder_P << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) valor_encoder_P++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) valor_encoder_P--;
  ult_valor_encoder_P = encoded;
}

// Controle a cada 10 ms, dentro da interrupcao do Timer1.
ISR(TIMER1_COMPA_vect, ISR_NOBLOCK) {
  if (dentro) return;
  dentro = true;

  // Leitura protegida dos contadores: um long nao e lido de uma vez no AVR.
  long pulsos_x, pulsos_P;
  uint8_t s = SREG;
  cli();
  pulsos_x = valor_encoder_x;
  pulsos_P = valor_encoder_P;
  SREG = s;

  theta = obter_angulo(pulsos_P);
  x     = obter_x(pulsos_x);

  // antes de armar
  if (!armado) {
    float w = (theta - theta_ant) / TS;        // so para decidir o armamento
    theta_ant = theta;
    if (fabs(theta) < TH_ARMA && fabs(w) < W_ARMA) firme++;
    else firme = 0;

    if (firme > 50) {                          // meio segundo firme
      s = SREG;
      cli();
      valor_encoder_x = 0;                     // zera a posicao no ponto de partida
      SREG = s;
      xh[0] = theta;  xh[1] = 0;  xh[2] = 0;  xh[3] = 0;
      xi = 0;  t = 0;  n = 0;
      armado = true;
    }
    Motor(0);

    // enquanto nao arma, manda o angulo a 5 Hz para o operador se guiar
    if ((n++ % 20) == 0 && !tem_linha) {
      r_armado = false;
      r_theta = theta;
      r_thp = (float)firme;
      tem_linha = true;
    }
    dentro = false;
    return;
  }

  // seguranca
  if (fabs(theta) > TH_ABORTA || fabs(x) > X_ABORTA ||
      digitalRead(FIM_1) == LOW || digitalRead(FIM_2) == LOW) {
    Motor(0);
    armado = false;
    firme = 0;
    dentro = false;
    return;
  }

  // referencia
  float ref = REF;
  if (ONDA) {
    if (t >= T_INICIO) {
      float fase = fmod(t - T_INICIO, T_ONDA);
      ref += (fase < 0.5f * T_ONDA) ? A_ONDA : -A_ONDA;
    }
  } else if (DEGRAU && t >= T_DEGRAU) {
    ref += A_DEGRAU;
  }

  // lei de controle: angulo e posicao medidos, velocidades do observador
  float u_calc = -(Ka[0]*theta + Ka[1]*xh[1] + Ka[2]*x + Ka[3]*xh[3] + Ka[4]*xi);
  u = u_calc;
  if (u >  V_SAT) u =  V_SAT;
  if (u < -V_SAT) u = -V_SAT;
  Motor(SENTIDO * u);

  // integrador condicional
  float erro = ref - x;
  bool saturou = (u != u_calc);
  bool piora   = ((-Ka[4] * erro > 0) == (u_calc > 0));
  if (!(saturou && piora)) xi += TS * erro;

  // observador:  xh[k+1] = Ad xh + Bd u + Ke (y - C xh),  y = [theta; x]
  float ey = theta - xh[0];
  float ex = x     - xh[2];
  float novo[4];
  for (int i = 0; i < 4; i++) {
    novo[i] = Bd[i] * u + Ke[i][0] * ey + Ke[i][1] * ex;
    for (int j = 0; j < 4; j++) novo[i] += Ad[i][j] * xh[j];
  }
  for (int i = 0; i < 4; i++) xh[i] = novo[i];

  // fotografia para o registro, a 50 Hz
  t += TS;
  if ((n++ % 2) == 0 && !tem_linha) {
    r_armado = true;
    r_t = t;    r_ref = ref;  r_theta = theta;  r_thp = xh[1];
    r_x = x;    r_xp = xh[3]; r_xi = xi;        r_u = u;
    tem_linha = true;
  }

  dentro = false;
}

void setup() {
  Serial.begin(115200);

  pinMode(x_A, INPUT_PULLUP);
  pinMode(x_B, INPUT_PULLUP);
  pinMode(P_A, INPUT_PULLUP);
  pinMode(P_B, INPUT_PULLUP);
  pinMode(FIM_1, INPUT_PULLUP);
  pinMode(FIM_2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(x_A), manipulador_encoder_x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(x_B), manipulador_encoder_x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(P_A), manipulador_encoder_P, CHANGE);
  attachInterrupt(digitalPinToInterrupt(P_B), manipulador_encoder_P, CHANGE);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  Serial.println(F("# H-infinito completo, lei (4.7), com observador LENTO"));
  Serial.print(F("# polos do observador: -26 -28 -30 -32 1/s, Ke dominante = "));
  Serial.println(Ke[1][0], 4);
  if (ONDA) {
    Serial.print(F("# onda quadrada de +-"));
    Serial.print(A_ONDA * 1000.0f, 0);
    Serial.print(F(" mm, periodo de "));
    Serial.print(T_ONDA, 0);
    Serial.println(F(" s, comecando aos 6 s"));
  }
  Serial.println(F("t,ref,theta,theta_p,x,x_p,xi,u,pwm"));
  delay(3000);                       // so entao o timer e habilitado

  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);               // CTC por OCR1A
  TCCR1B |= (1 << CS11) | (1 << CS10);  // prescaler 64
  OCR1A = 2499;                         // 16 MHz / 64 * 0,01 - 1
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

// O loop so imprime. O controle inteiro esta na ISR do Timer1.
void loop() {
  if (!tem_linha) return;

  float t_, ref_, th_, thp_, x_, xp_, xi_, u_;
  bool arm_;
  uint8_t s = SREG;
  cli();
  arm_ = r_armado;
  t_ = r_t;   ref_ = r_ref;  th_ = r_theta;  thp_ = r_thp;
  x_ = r_x;   xp_ = r_xp;    xi_ = r_xi;     u_ = r_u;
  tem_linha = false;
  SREG = s;

  // Enquanto nao armou, sai so o angulo, como comentario, para o operador
  // saber quando esta dentro da janela. Comentario nao entra no registro.
  if (!arm_) {
    Serial.print(F("# posicione: theta = "));
    Serial.print(th_ * 180.0f / PI, 2);
    Serial.print(F(" graus, firme "));
    Serial.print((int)thp_);
    Serial.println(F("/50"));
    return;
  }

  Serial.print(t_, 2);    Serial.print(',');
  Serial.print(ref_, 4);  Serial.print(',');
  Serial.print(th_, 4);   Serial.print(',');
  Serial.print(thp_, 3);  Serial.print(',');
  Serial.print(x_, 4);    Serial.print(',');
  Serial.print(xp_, 3);   Serial.print(',');
  Serial.print(xi_, 4);   Serial.print(',');
  Serial.print(u_, 3);    Serial.print(',');
  Serial.println((int)(u_ * 255.0f / 12.0f));
}
