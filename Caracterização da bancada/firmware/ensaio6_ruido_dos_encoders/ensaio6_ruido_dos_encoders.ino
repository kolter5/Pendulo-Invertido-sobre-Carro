// Felipe Kolter - Inverted Pendulum
// ENSAIO 5: ruido dos encoders nas condicoes reais de operacao

#define P_A 2
#define P_B 3
#define x_A 18
#define x_B 19
#define FIM_ESQ 4
#define FIM_DIR 5

int IN1 = 8;
int IN2 = 9;

const float PI2 = 2.0 * PI;
const float R_POLIA = 0.02519;
const unsigned long TS_US = 10000UL;      // 100 Hz, igual ao controle
const unsigned long FASE_MS = 10000UL;    // 10 s por fase

const int PWM_ABAIXO = 45;    // abaixo do destaque medido (65)
const int PWM_MOVE   = 120;   // move o carro de fato
const float X_LIM    = 0.04;  // m: excursao da fase 3

volatile long enc_P = 0, ult_P = 0;
volatile long enc_x = 0, ult_x = 0;
bool abortado = false;

float obter_angulo(long p) { return PI + PI2 * p / 4096; }
float obter_x(long p)      { return PI2 * p * R_POLIA / 4096; }

void isr_P() {
  int MSB = (PINE & (1 << PE4)) >> PE4;
  int LSB = (PINE & (1 << PE5)) >> PE5;
  int e = (MSB << 1) | LSB;
  int s = (ult_P << 2) | e;
  if (s == 0b1101 || s == 0b0100 || s == 0b0010 || s == 0b1011) enc_P++;
  if (s == 0b1110 || s == 0b0111 || s == 0b0001 || s == 0b1000) enc_P--;
  ult_P = e;
}

void isr_x() {
  int MSB = (PIND & (1 << PD3)) >> PD3;
  int LSB = (PIND & (1 << PD2)) >> PD2;
  int e = (MSB << 1) | LSB;
  int s = (ult_x << 2) | e;
  if (s == 0b1101 || s == 0b0100 || s == 0b0010 || s == 0b1011) enc_x--;
  if (s == 0b1110 || s == 0b0111 || s == 0b0001 || s == 0b1000) enc_x++;
  ult_x = e;
}

long ler_P() { noInterrupts(); long p = enc_P; interrupts(); return p; }
long ler_x() { noInterrupts(); long p = enc_x; interrupts(); return p; }

bool fim_de_curso() {
  return (digitalRead(FIM_ESQ) == LOW || digitalRead(FIM_DIR) == LOW);
}

void motor(int pwm) {
  if (pwm > 200) pwm = 200;
  if (pwm < -200) pwm = -200;
  if (pwm > 0)      { digitalWrite(IN1, LOW); analogWrite(IN2, pwm); }
  else if (pwm < 0) { digitalWrite(IN2, LOW); analogWrite(IN1, -pwm); }
  else              { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
}

// Registra uma fase inteira a 100 Hz. O comando do motor e decidido pela
// funcao passada, que recebe o tempo decorrido e a posicao do carro.
void fase(int id, const char *nome, int (*comando)(unsigned long, float),
          unsigned long t0_global) {
  if (abortado) return;
  Serial.print(F("# fase ")); Serial.print(id);
  Serial.print(F(": ")); Serial.println(nome);

  unsigned long ini = millis();
  unsigned long prox = micros();
  while (millis() - ini < FASE_MS) {
    unsigned long agora = micros();
    if ((long)(agora - prox) < 0) continue;
    prox += TS_US;

    float x = obter_x(ler_x());
    if (fim_de_curso() || fabs(x) > 0.20) {
      motor(0); abortado = true;
      Serial.println(F("# fim de curso - interrompido"));
      return;
    }

    int pwm = comando(millis() - ini, x);
    motor(pwm);

    Serial.print(id); Serial.print(',');
    Serial.print((agora - t0_global) / 1.0e6, 4); Serial.print(',');
    Serial.print(obter_angulo(ler_P()), 6); Serial.print(',');
    Serial.print(x, 6); Serial.print(',');
    Serial.println(pwm);
  }
  motor(0);
}

// --- comandos de cada fase ---
int cmd_off(unsigned long ms, float x)  { return 0; }

int cmd_pwm(unsigned long ms, float x) {
  // alterna de sentido a cada 0,5 s: a ponte chaveia, o carro nao anda
  return ((ms / 500) % 2 == 0) ? PWM_ABAIXO : -PWM_ABAIXO;
}

int cmd_move(unsigned long ms, float x) {
  // vaivem dentro de +-X_LIM
  static int sentido = 1;
  if (x >  X_LIM) sentido = -1;
  if (x < -X_LIM) sentido = +1;
  return sentido * PWM_MOVE;
}

void setup() {
  Serial.begin(115200);

  pinMode(P_A, INPUT_PULLUP); pinMode(P_B, INPUT_PULLUP);
  pinMode(x_A, INPUT_PULLUP); pinMode(x_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(P_A), isr_P, CHANGE);
  attachInterrupt(digitalPinToInterrupt(P_B), isr_P, CHANGE);
  attachInterrupt(digitalPinToInterrupt(x_A), isr_x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(x_B), isr_x, CHANGE);

  pinMode(FIM_ESQ, INPUT_PULLUP);
  pinMode(FIM_DIR, INPUT_PULLUP);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  motor(0);

  Serial.println(F("# ENSAIO DE RUIDO DOS ENCODERS"));
  Serial.println(F("# haste PENDURADA e parada; carro no centro do trilho."));

  if (fim_de_curso()) {
    Serial.println(F("# ERRO: fim de curso acionado. Cancelado."));
    Serial.println(F("# fim do ensaio"));
    return;
  }

  Serial.println(F("# pronto - aguardando o gravador"));
  while (!Serial.available()) delay(50);
  while (Serial.available()) Serial.read();

  noInterrupts(); enc_P = 0; enc_x = 0; interrupts();
  unsigned long t0 = micros();

  Serial.println(F("fase,t,theta,x,pwm"));

  Serial.println(F("# NAO ENCOSTE na bancada durante as fases 1 e 2."));
  fase(1, "PARADO_OFF  (nada se move, motor sem acionamento)", cmd_off,  t0);
  fase(2, "PARADO_PWM  (ponte chaveando, carro parado)",       cmd_pwm,  t0);
  fase(3, "CARRO_MOV   (vaivem com corrente real)",            cmd_move, t0);

  motor(0);
  Serial.println(F("# fase 4 em 5 s: solte a haste de ~10 graus quando comecar"));
  delay(5000);
  fase(4, "PENDULO     (haste oscilando, motor desligado)",    cmd_off,  t0);

  motor(0);
  Serial.println(F("# fim do ensaio"));
}

void loop() {
  motor(0);
}
