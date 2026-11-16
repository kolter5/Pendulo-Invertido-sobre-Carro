// Felipe Kolter - Inverted Pendulum
// ENSAIO 1: queda livre do pendulo (carro travado)
//
// Objetivo: comprovar experimentalmente a equacao (4.16) do TCC,
//           fn = (1/2pi) sqrt(m g L / Ip), identificar o atrito no pivo e
//           verificar a dependencia do periodo com a amplitude, que so o
//           modelo NAO LINEAR preve.
//
// O ensaio e repetido automaticamente para cada amplitude da lista ALVOS.
// Enquanto voce posiciona a haste, o angulo atual e impresso em GRAUS na
// mesma linha, para dar para acertar o alvo com precisao. Quando a haste
// fica parada perto do alvo o sketch se arma sozinho; a gravacao comeca no
// instante em que voce solta.
//
// Procedimento:
//   1. Travar o carro (segurar o patim ou prender a correia).
//   2. Gravar este sketch, deixar a haste PENDENTE e parada.
//   3. python grava_serial.py COMx ensaio_queda_livre.csv
//   4. Seguir as instrucoes que aparecem no terminal, uma amplitude por vez.
//   5. Ao final: python valida_experimental.py
//
// Saida: CSV "alvo,t,theta"
//   alvo  amplitude nominal daquela repeticao, em graus
//   t     segundos desde a soltura
//   theta radianos a partir da vertical superior (pi = pendente), na mesma
//         convencao de obter_angulo() do firmware de controle

#define P_A 2   // PE4  encoder do pendulo, canal A
#define P_B 3   // PE5  encoder do pendulo, canal B

const float PI2 = 2.0 * PI;
const unsigned long TS_US = 10000UL;   // 100 Hz, mesmo periodo do controle

// Amplitudes de soltura, em graus, medidas a partir da posicao PENDENTE.
// Registro unico e longo de 90 graus: uma soltura so varre toda a faixa de
// amplitude enquanto decai, o que da a curva T(amplitude) inteira com eixo
// de tempo medido por micros().
const float ALVOS[] = {90.0};
// Serie de varias amplitudes (descomentar se quiser repetir o ensaio curto):
// const float ALVOS[] = {90.0, 60.0, 30.0, 15.0, 5.0};
const int N_ALVOS = sizeof(ALVOS) / sizeof(ALVOS[0]);

// Duracao do registro de cada repeticao, em ms  (200 s = 3 min 20 s)
const unsigned long DURACAO_MS = 200000UL;

// Faixa de aceite em torno do alvo, em graus. Estreita de proposito: o
// objetivo e soltar EM 90 graus, nao "por volta de". Alargue se achar dificil
// de segurar; o angulo real de soltura e registrado de qualquer forma.
const float TOL_ALVO = 1.5;

// Piso: alem de estar na faixa, a haste precisa estar levantada. Impede que a
// posicao de REPOUSO seja aceita como se ja fosse a posicao de soltura.
float piso_alvo(float alvo)   { return 0.60 * alvo; }
float tol_soltura(float alvo) { return min(2.0, 0.30 * alvo); }  // queda = soltura

const float TOL_PARADO = 0.6;               // graus: deriva total tolerada
const unsigned long T_PARADO_MS = 1200UL;   // tempo firme exigido para armar

volatile long valor_encoder_P = 0;
volatile long ult_valor_encoder_P = 0;

// theta a partir da vertical superior, SEM wrap: o pendulo oscila em torno
// de pi e o wrap para [-pi, pi] produziria saltos no meio do registro
float obter_angulo(long pulsos) {
  return PI + PI2 * pulsos / 4096;
}

// desvio da posicao pendente, em graus (0 = pendurado, positivo = um lado)
float obter_graus(long pulsos) {
  return 360.0 * pulsos / 4096;
}

long ler_pulsos() {
  noInterrupts();
  long p = valor_encoder_P;
  interrupts();
  return p;
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

// Linha de posicionamento: comeca com "# pos:" e o gravador a reescreve
// sempre na mesma linha do terminal, em vez de rolar a tela. Alem do angulo,
// diz o que fazer e quanto falta para armar, para dar para acertar o alvo.
void mostrar_posicao(float alvo, float graus, bool dentro, bool firme,
                     unsigned long falta_ms) {
  float erro = fabs(graus) - alvo;   // positivo = passou do alvo

  Serial.print(F("# pos: "));
  if (graus >= 0) Serial.print('+');
  Serial.print(graus, 2);
  Serial.print(F(" / "));
  Serial.print(alvo, 1);
  Serial.print(F(" graus | "));

  if (!dentro) {
    Serial.print(erro < 0 ? F("SUBA  ") : F("DESCA "));
    Serial.print(fabs(erro), 2);
    Serial.println(F(" graus"));
  } else if (!firme) {
    Serial.print(F("na faixa (erro "));
    Serial.print(erro, 2);
    Serial.println(F(") - pare de mexer"));
  } else {
    Serial.print(F("SEGURE ASSIM - armando em "));
    Serial.print(falta_ms / 1000.0, 1);
    Serial.println(F(" s"));
  }
}

// Espera a haste ficar parada perto do alvo; devolve o angulo em que armou
float aguardar_posicionamento(float alvo) {
  // g_ref so e atualizado quando a haste sai da faixa: assim TOL_PARADO limita
  // a deriva TOTAL na janela, e nao a variacao entre duas leituras seguidas
  // (0,5 grau entre amostras de 20 ms deixaria passar 25 graus por segundo).
  float g_ref = obter_graus(ler_pulsos());
  unsigned long firme_desde = millis();
  unsigned long ultimo_print = 0;

  while (true) {
    delay(20);
    float g = obter_graus(ler_pulsos());

    if (fabs(g - g_ref) > TOL_PARADO) {
      g_ref = g;                   // ainda se mexendo
      firme_desde = millis();
    }

    bool dentro = fabs(fabs(g) - alvo) < TOL_ALVO;
    bool levantada = fabs(g) > piso_alvo(alvo);

    // A contagem so corre com a haste DENTRO da faixa: sair dela reinicia,
    // senao uma pausa no meio da subida ja dispararia o ensaio.
    if (!dentro || !levantada) {
      firme_desde = millis();
    }

    unsigned long segurando = millis() - firme_desde;
    unsigned long falta = (segurando < T_PARADO_MS) ? (T_PARADO_MS - segurando) : 0;
    bool firme = segurando > 200;

    if (millis() - ultimo_print > 150) {
      mostrar_posicao(alvo, g, dentro && levantada, firme && dentro && levantada,
                      falta);
      ultimo_print = millis();
    }

    if (dentro && levantada && segurando > T_PARADO_MS) {
      return g;
    }
  }
}

// Espera a soltura e registra DURACAO_MS de oscilacao
void registrar(float alvo, float g_armado) {
  Serial.print(F("# armado em "));
  Serial.print(g_armado, 1);
  Serial.println(F(" graus - pode soltar"));

  int sinal = (g_armado >= 0) ? 1 : -1;
  float queda = tol_soltura(fabs(g_armado));
  while (sinal * obter_graus(ler_pulsos()) > sinal * g_armado - queda) {
    delay(2);
  }

  unsigned long t0 = micros();
  unsigned long prox = t0;
  unsigned long fim = millis() + DURACAO_MS;

  Serial.println(F("# gravando..."));
  while (millis() < fim) {
    unsigned long agora = micros();
    if ((long)(agora - prox) < 0) continue;
    prox += TS_US;

    Serial.print(alvo, 0);
    Serial.print(',');
    Serial.print((agora - t0) / 1.0e6, 4);
    Serial.print(',');
    Serial.println(obter_angulo(ler_pulsos()), 6);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(P_A, INPUT_PULLUP);
  pinMode(P_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(P_A), manipulador_encoder_P, CHANGE);
  attachInterrupt(digitalPinToInterrupt(P_B), manipulador_encoder_P, CHANGE);

  // Motor desligado durante todo o ensaio
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);

  Serial.println(F("# ENSAIO DE QUEDA LIVRE - carro travado, motor desligado"));
  Serial.println(F("# deixe a haste PENDENTE e parada; zerando em 5 s"));
  delay(5000);

  noInterrupts();
  valor_encoder_P = 0;      // pulsos = 0 -> theta = pi (posicao pendente)
  interrupts();
  Serial.println(F("# zerado"));

  Serial.println(F("alvo,t,theta"));

  for (int i = 0; i < N_ALVOS; i++) {
    Serial.print(F("# --- repeticao "));
    Serial.print(i + 1);
    Serial.print('/');
    Serial.print(N_ALVOS);
    Serial.print(F(": levante a haste ate "));
    Serial.print(ALVOS[i], 0);
    Serial.println(F(" graus"));

    // repouso antes de cada repeticao: mostra o angulo de descanso, que
    // deve voltar a ~0; desvio persistente indica escorregamento do acoplador
    Serial.print(F("# angulo de repouso antes da repeticao: "));
    Serial.print(obter_graus(ler_pulsos()), 2);
    Serial.println(F(" graus"));

    float g_armado = aguardar_posicionamento(ALVOS[i]);
    registrar(ALVOS[i], g_armado);

    Serial.println(F("# repeticao concluida - aguarde a haste parar"));
    delay(4000);
  }

  Serial.println(F("# fim do ensaio"));
}

void loop() {
}
