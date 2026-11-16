// Felipe Kolter - Inverted Pendulum
// ENSAIO 2: caracterizacao do acionamento (motor CC + redutor + correia)
//
// Objetivo: levantar experimentalmente os parametros que hoje aparecem como
//           constantes fixas no firmware de controle, sem deducao no texto:

#define x_A 18  // PD3  encoder do carro, canal A
#define x_B 19  // PD2  encoder do carro, canal B

// Chaves de fim de curso KW12-3-1, entradas com pull-up interno: acionadas
// levam o pino a nivel BAIXO. Sao a protecao de ultima instancia deste ensaio,
// que aciona o motor em malha aberta.
#define FIM_ESQ 4
#define FIM_DIR 5

int IN1 = 8;
int IN2 = 9;

bool abortado = false;

const float PI2 = 2.0 * PI;
const float R_POLIA = 0.02519;      // m, raio efetivo da polia GT2
const unsigned long TS_US = 10000UL;  // 100 Hz

// >>> MODO DE OPERACAO <<<
//   true  = SO LEITURA: o motor NUNCA e acionado. O sketch fica imprimindo a
//           posicao do carro e o estado das chaves de fim de curso, para
//           conferir a instrumentacao empurrando o carro com a mao.
//   false = ensaio completo, com acionamento em malha aberta.
const bool SO_LEITURA = false;

// Limites de seguranca (ajustar ao curso util do trilho)
const float X_MIN = -0.45;
const float X_MAX = 0.45;
const int PWM_MAX = 200;

volatile long valor_encoder_x = 0;
volatile long ult_valor_encoder_x = 0;

float x_ant = 0.0, v_filt = 0.0;

float obter_x(long pulsos) {
  return PI2 * pulsos * R_POLIA / 4096;
}

void manipulador_encoder_x() {
  int MSB = (PIND & (1 << PD3)) >> PD3;
  int LSB = (PIND & (1 << PD2)) >> PD2;
  int encoded = (MSB << 1) | LSB;
  int sum = (ult_valor_encoder_x << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) valor_encoder_x--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) valor_encoder_x++;
  ult_valor_encoder_x = encoded;
}

// PWM bruto, SEM as compensacoes do firmware: o ensaio existe justamente
// para medir aquilo que as compensacoes tentam corrigir.
void motor_bruto(int pwm) {
  if (pwm > PWM_MAX)  pwm = PWM_MAX;
  if (pwm < -PWM_MAX) pwm = -PWM_MAX;

  if (pwm > 0) {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN2, LOW);
    analogWrite(IN1, -pwm);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void parar() {
  motor_bruto(0);
  delay(700);
}

float ler_x() {
  noInterrupts();
  long p = valor_encoder_x;
  interrupts();
  return obter_x(p);
}

bool fim_de_curso() {
  return (digitalRead(FIM_ESQ) == LOW || digitalRead(FIM_DIR) == LOW);
}

bool fora_do_curso(float x) {
  if (fim_de_curso()) {
    if (!abortado) {
      motor_bruto(0);
      abortado = true;
      Serial.println(F("# CHAVE DE FIM DE CURSO ACIONADA - ensaio interrompido"));
    }
    return true;
  }
  return (x < X_MIN || x > X_MAX);
}

void amostra(const char *fase, int sentido, int pwm, unsigned long t0) {
  float x = ler_x();
  float v = (x - x_ant) / (TS_US / 1.0e6);
  v_filt = 0.7 * v_filt + 0.3 * v;      // filtro de 1a ordem no ruido do encoder
  x_ant = x;

  Serial.print(fase);      Serial.print(',');
  Serial.print(sentido);   Serial.print(',');
  Serial.print(pwm);       Serial.print(',');
  Serial.print((micros() - t0) / 1.0e6, 4); Serial.print(',');
  Serial.print(x, 6);      Serial.print(',');
  Serial.println(v_filt, 5);
}

// --- Fase A: rampa lenta ate o carro destacar ------------------------------
void fase_rampa(int sentido) {
  unsigned long t0 = micros();
  x_ant = ler_x();
  v_filt = 0.0;
  float x_ini = x_ant;

  for (int pwm = 0; pwm <= PWM_MAX && !abortado; pwm++) {
    for (int k = 0; k < 15 && !abortado; k++) {   // 150 ms em cada degrau
      if (fim_de_curso()) { fora_do_curso(ler_x()); break; }
      motor_bruto(sentido * pwm);
      delayMicroseconds(TS_US);
      amostra("rampa", sentido, pwm, t0);
    }
    float x = ler_x();
    if (fabs(x - x_ini) > 0.010 || fora_do_curso(x)) break;  // destacou 10 mm
  }
  parar();
}

// --- Fase B: patamares de PWM, velocidade em regime ------------------------
void fase_degraus(int sentido) {
  int lista[] = {90, 110, 130, 150, 170, 190};
  for (unsigned int i = 0; i < sizeof(lista) / sizeof(lista[0]) && !abortado; i++) {
    // reposiciona o carro na extremidade oposta antes de cada patamar
    unsigned long t0 = micros();
    x_ant = ler_x();
    v_filt = 0.0;
    while (!fora_do_curso(ler_x())) {
      motor_bruto(sentido * lista[i]);
      delayMicroseconds(TS_US);
      amostra("degrau", sentido, lista[i], t0);
    }
    parar();
    // volta devagar ate o centro
    unsigned long t1 = micros();
    while (fabs(ler_x()) > 0.05 && (micros() - t1) < 6000000UL && !fim_de_curso()) {
      motor_bruto(-sentido * 120);
      delayMicroseconds(TS_US);
    }
    parar();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(x_A, INPUT_PULLUP);
  pinMode(x_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(x_A), manipulador_encoder_x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(x_B), manipulador_encoder_x, CHANGE);

  pinMode(FIM_ESQ, INPUT_PULLUP);
  pinMode(FIM_DIR, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  parar();

  Serial.println(F("# ENSAIO DE CARACTERIZACAO DO ATUADOR"));
  Serial.println(F("# REMOVA O PENDULO. Carro deve partir do centro do trilho."));

  // Se uma chave ja estiver acionada, o carro esta encostado no batente:
  // nao ha como saber para que lado seria seguro andar.
  if (fim_de_curso()) {
    Serial.println(F("# ERRO: chave de fim de curso ja acionada."));
    Serial.println(F("# Afaste o carro do batente e reinicie. Ensaio cancelado."));
    Serial.println(F("# fim do ensaio"));
    abortado = true;
    return;
  }

  Serial.println(F("# chaves de fim de curso OK."));

  // O ensaio NAO comeca sozinho. Ele espera um caractere pela serial, que o
  // grava_serial.py envia assim que abre a porta. Assim o carro so se move
  // com a gravacao ja rodando - gravar o sketch pela IDE nao aciona nada.
  Serial.println(F("# pronto - aguardando o gravador (envie qualquer tecla)"));
  while (!Serial.available()) {
    delay(50);
  }
  while (Serial.available()) {
    Serial.read();                   // descarta o que veio
  }

  noInterrupts();
  valor_encoder_x = 0;
  interrupts();
  x_ant = 0.0;

  if (SO_LEITURA) {
    Serial.println(F("# MODO SO LEITURA - o motor nao sera acionado."));
    Serial.println(F("# empurre o carro com a mao e confira x e as chaves."));
    Serial.println(F("fase,sentido,pwm,t,x,v"));
    unsigned long t0 = micros();
    while (true) {
      delay(100);
      float x = ler_x();
      Serial.print(F("# x = "));
      Serial.print(x, 4);
      Serial.print(F(" m | fim esq "));
      Serial.print(digitalRead(FIM_ESQ) == LOW ? F("ACIONADA") : F("livre  "));
      Serial.print(F(" | fim dir "));
      Serial.println(digitalRead(FIM_DIR) == LOW ? F("ACIONADA") : F("livre  "));
      (void)t0;
    }
  }

  Serial.println(F("# iniciando em 3 s..."));
  delay(3000);

  Serial.println(F("fase,sentido,pwm,t,x,v"));

  fase_rampa(+1);
  fase_rampa(-1);
  fase_degraus(+1);
  fase_degraus(-1);

  Serial.println(F("# fim do ensaio"));
  parar();
}

void loop() {
  parar();
}
