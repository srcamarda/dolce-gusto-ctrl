
/*
 * ================================================================
 *   Dolce Gusto Mini Me — Controlador Arduino Micro  v1.6
 * ================================================================
 *
 * PINAGEM:
 *  A0  — NTC 100K B3950 (NTC entre VCC e A0; resistor 10K entre A0 e GND)
 *  D2  — Chave de nível fio 1
 *  D3  — Chave de nível fio 2
 *  D4  — Chave de nível fio 3
 *  D5  — Botão ON/OFF original
 *  D6  — LED Vermelho original
 *  D7  — Fluxômetro, sensor Hall 3144 (INT6 — borda de subida)
 *        No Arduino Micro os pinos com interrupção são D0, D1, D2, D3 e D7
 *  D8  — IN1 módulo relay → caldeira 1400W AC
 *  D9  — IN2 módulo relay → bomba AC
 *  D10 — LED Verde original
 *
 * CONECTOR 3p (chave de nível):
 *  roxo   (marrom) → D3  (INPUT_PULLUP)
 *  cinza  (preto)  → GND
 *  branco (branco) → D2  (INPUT_PULLUP)
 *
 * CONECTOR 4p (LEDs e ON/OFF):
 *  amarelo (amarelo)  → D5  (BTN_START)
 *  verde   (lilás)    → D10 (LED verde - com resistor 330 ohms)
 *  azul    (vermelho) → D6  (LED vermelho - com resistor 330 ohms)
 *  roxo    (verde)    → D4  (INPUT_PULLUP)
 *
 * HISTÓRICO:
 *  v1.6 (08/10/2026)
 *   - Timeout de preparo e erro por falta de fluxo (bomba ligada sem pulsos)
 *   - NTC aberto, em curto ou fora de faixa gera ERRO em qualquer estado
 *   - Aquecimento: a bomba parte no alvo OU quando a temperatura passa do
 *     pico depois de a caldeira desligar (não depende mais do timeout)
 *   - Cancelamento desliga as saídas ANTES de esperar soltar o botão;
 *     botão preso por mais de 10 s gera ERRO
 *   - Relés definidos como desligados antes de virarem saída (sem pulso no boot)
 *   - Watchdog de 2 s; ERRO e CONCLUÍDO sem delay bloqueante
 *   - Combinação inválida da chave de nível mantém o último nível válido
 *   - Média de 4 leituras no NTC; leitura atômica do contador de pulsos
 *
 * ================================================================
 */

#include <math.h>
#include <avr/wdt.h>

// ── Pinos
const uint8_t PIN_NTC            = A0;
const uint8_t PIN_1_NIVEL        = 2;
const uint8_t PIN_2_NIVEL        = 3;
const uint8_t PIN_3_NIVEL        = 4;
const uint8_t BTN_START          = 5;
const uint8_t PIN_LED_VERMELHO   = 6;
const uint8_t PIN_FLUXO          = 7;   // INT6
const uint8_t PIN_RELAY_CALDEIRA = 8;   // CH1
const uint8_t PIN_RELAY_BOMBA    = 9;   // CH2
const uint8_t PIN_LED_VERDE      = 10;

// ── Relay (módulo ativo em LOW)
const uint8_t RELAY_ON  = LOW;
const uint8_t RELAY_OFF = HIGH;

// ── NTC 100K — equação β
const float   NTC_NOMINAL  = 100000.0f; // Ω a 25 °C
const float   NTC_BETA     = 3950.0f;   // coeficiente B — ajuste após calibração
const float   NTC_T0       = 298.15f;   // 25 °C em Kelvin
const float   R_DIVISOR    = 10000.0f;  // resistor série 10 KΩ
const float   NTC_T_MIN    =   1.0f;    // °C — abaixo disso: sensor aberto/solto
const float   NTC_T_MAX    = 150.0f;    // °C — acima disso: sensor em curto
const uint8_t NTC_AMOSTRAS = 4;

// ── Temperatura (°C)
const float TEMP_DESLIGA    = 75.0f;  // desliga a caldeira no aquecimento (antecipa o overshoot)
const float TEMP_ALVO       = 88.0f;  // alvo de aquecimento
const float TEMP_HISTERESE  =  5.0f;  // banda liga/desliga da caldeira no preparo
const float TEMP_QUEDA_PICO =  1.0f;  // queda abaixo do pico que indica que o pico passou
const float TEMP_SEGURANCA  = 110.0f; // desliga por segurança
const unsigned long T_MAX_AQUECIMENTO = 120000UL; // 2 min

// ── Fluxo e preparo
const float         PULSOS_POR_LITRO = 3375.0f;
const unsigned long T_MAX_PREPARO    = 90000UL; // 90 s — ajuste à vazão real da bomba
const unsigned long T_SEM_FLUXO      = 3000UL;  // 3 s sem pulsos com a bomba ligada

// ── Interface
const unsigned long T_CONCLUIDO  = 4000UL;
const unsigned long T_PISCA_ERRO = 300UL;
const unsigned long T_MAX_BOTAO  = 10000UL; // botão pressionado além disso = preso

// ── Tamanhos de café (mL) — níveis 1 a 7
const float VOLUME_ML[7] = {
   45.0f,  // nível 1 — espresso curto
   55.0f,  // nível 2
   90.0f,  // nível 3
  120.0f,  // nível 4
  140.0f,  // nível 5
  160.0f,  // nível 6
  210.0f   // nível 7 — copo grande
};
const uint8_t TOTAL_NIVEIS   = 7;
const uint8_t NIVEL_INVALIDO = 255;

// ── Estado da máquina
enum Estado : uint8_t {
  IDLE,
  AQUECENDO,
  PREPARANDO,
  CONCLUIDO,
  ERRO
};

Estado        estado          = IDLE;
uint8_t       nivel_atual     = 0;
unsigned long t_estado        = 0;   // millis() na entrada do estado atual
unsigned long t_ultimo_pulso  = 0;
unsigned long pulsos_anterior = 0;
float         t_pico          = 0;
bool          caldeira_ligada = false;
const char*   msg_erro        = "";

volatile unsigned long pulsos = 0;

// ── ISR do fluxômetro
void ISR_fluxo() { pulsos++; }

// ── Leitura atômica do contador de pulsos
unsigned long ler_pulsos() {
  noInterrupts();
  unsigned long p = pulsos;
  interrupts();
  return p;
}

// ── Volume dispensado em mL
float volume_ml() {
  return (ler_pulsos() * 1000.0f) / PULSOS_POR_LITRO;
}

// ── Leitura de temperatura (°C) — NAN se o sensor estiver em falha
float temperatura() {
  unsigned int soma = 0;
  for (uint8_t i = 0; i < NTC_AMOSTRAS; i++) soma += analogRead(PIN_NTC);
  float raw = soma / (float)NTC_AMOSTRAS;

  if (raw < 1.0f || raw > 1022.0f) return NAN; // aberto (≈0) ou em curto (≈1023)

  float R   = R_DIVISOR * (1023.0f - raw) / raw;
  float inv = (1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(R / NTC_NOMINAL);
  float t   = (1.0f / inv) - 273.15f;

  if (t < NTC_T_MIN || t > NTC_T_MAX) return NAN;
  return t;
}

// ── Define o nível de café usando 3 pinos (0..6, ou NIVEL_INVALIDO)
uint8_t ler_nivel() {
  bool P1 = digitalRead(PIN_1_NIVEL) == LOW;
  bool P2 = digitalRead(PIN_2_NIVEL) == LOW;
  bool P3 = digitalRead(PIN_3_NIVEL) == LOW;

  if (!P1 && !P2 && !P3) return 0;
  if (P1 && !P2 && !P3)  return 1;
  if (P1 && !P2 && P3)   return 2;
  if (!P1 && !P2 && P3)  return 3;
  if (!P1 && P2 && P3)   return 4;
  if (!P1 && P2 && !P3)  return 5;
  if (P1 && P2 && !P3)   return 6;
  return NIVEL_INVALIDO; // P1 && P2 && P3 — não existe na chave
}

// ── Controle de saídas
void caldeira(bool on) {
  digitalWrite(PIN_RELAY_CALDEIRA, on ? RELAY_ON : RELAY_OFF);
  caldeira_ligada = on;
}
inline void bomba(bool on) { digitalWrite(PIN_RELAY_BOMBA, on ? RELAY_ON : RELAY_OFF); }

void parar_tudo() {
  caldeira(false);
  bomba(false);
}

// ── LEDs
void leds(bool verde, bool vermelho) {
  digitalWrite(PIN_LED_VERDE,    verde    ? HIGH : LOW);
  digitalWrite(PIN_LED_VERMELHO, vermelho ? HIGH : LOW);
}

// ── Leitura de botão com debounce
bool btn_pressionado(uint8_t pino) {
  if (digitalRead(pino) != LOW) return false;
  delay(20);
  return digitalRead(pino) == LOW;
}

// ── Aguarda soltar o botão. Retorna false se continuar pressionado
//    além de T_MAX_BOTAO (botão preso).
bool aguarda_soltar(uint8_t pino) {
  unsigned long t0 = millis();
  while (digitalRead(pino) == LOW) {
    wdt_reset();
    if (millis() - t0 >= T_MAX_BOTAO) return false;
    delay(10);
  }
  delay(20);
  return true;
}

// ── Imprime nível atual no Serial
void print_nivel() {
  Serial.print(F(">> Nivel: "));
  Serial.print(nivel_atual + 1);
  Serial.print(F("/7 — "));
  Serial.print(VOLUME_ML[nivel_atual], 0);
  Serial.println(F(" mL"));
}

// ── Transições de estado
void mudar_estado(Estado novo) {
  estado   = novo;
  t_estado = millis();
}

void entrar_erro(const char* msg) {
  parar_tudo();
  msg_erro = msg;
  Serial.print(F("ERRO: "));
  Serial.println(msg_erro);
  mudar_estado(ERRO);
}

// ── Cancelamento pelo operador: desliga primeiro, espera o botão depois
void cancelar() {
  parar_tudo();
  Serial.println(F(">> Preparo cancelado."));
  if (!aguarda_soltar(BTN_START)) {
    entrar_erro("BOTAO PRESO");
    return;
  }
  leds(true, false);
  mudar_estado(IDLE);
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  wdt_disable();

  // Relés: define o nível "desligado" ANTES de virar saída.
  // Com o módulo ativo em LOW, pinMode(OUTPUT) primeiro deixaria o pino
  // em LOW por um instante e acionaria a caldeira a cada reset.
  digitalWrite(PIN_RELAY_CALDEIRA, RELAY_OFF);
  digitalWrite(PIN_RELAY_BOMBA,    RELAY_OFF);
  pinMode(PIN_RELAY_CALDEIRA, OUTPUT);
  pinMode(PIN_RELAY_BOMBA,    OUTPUT);
  caldeira_ligada = false;

  pinMode(PIN_LED_VERDE,    OUTPUT);
  pinMode(PIN_LED_VERMELHO, OUTPUT);
  leds(false, false);

  Serial.begin(9600);

  // Entradas com pull-up interno (botão → GND quando pressionado)
  pinMode(BTN_START,   INPUT_PULLUP);
  pinMode(PIN_1_NIVEL, INPUT_PULLUP);
  pinMode(PIN_2_NIVEL, INPUT_PULLUP);
  pinMode(PIN_3_NIVEL, INPUT_PULLUP);

  // Fluxômetro
  pinMode(PIN_FLUXO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLUXO), ISR_fluxo, RISING);

  Serial.println(F("=== Dolce Gusto Ctrl v1.6 ==="));
  uint8_t n = ler_nivel();
  if (n != NIVEL_INVALIDO) nivel_atual = n;
  print_nivel();

  leds(true, false); // verde = pronto
  mudar_estado(IDLE);

  wdt_enable(WDTO_2S);
}

// ─────────────────────────────────────────────────────────────
//  LOOP — máquina de estados
// ─────────────────────────────────────────────────────────────
void loop() {
  wdt_reset();

  float t = temperatura(); // uma leitura por ciclo, usada em todo o loop

  // ── Debug periódico
  static unsigned long t_dbg = 0;
  if (millis() - t_dbg >= 1000UL) {
    Serial.print(F("T:"));
    if (isnan(t)) Serial.print(F("--")); else Serial.print(t, 1);
    Serial.print(F("C  V:"));
    Serial.print(volume_ml(), 0);
    Serial.print(F("mL  P:"));
    Serial.print(ler_pulsos());
    Serial.print(F("  Alvo:"));
    Serial.print(VOLUME_ML[nivel_atual], 0);
    Serial.print(F("mL  E:"));
    Serial.println(estado);
    t_dbg = millis();
  }

  // ── Proteções globais (valem em qualquer estado)
  if (estado != ERRO) {
    if (isnan(t)) {
      entrar_erro("FALHA NTC");
      return;
    }
    if (t >= TEMP_SEGURANCA) {
      entrar_erro("TEMP CRITICA");
      return;
    }
  }

  // ── Máquina de estados
  switch (estado) {

    // ════════════════════════════════════════════════════════
    case IDLE: {
      leds(true, false);

      // Lê chave de nível continuamente — reflete posição atual
      uint8_t n = ler_nivel();
      if (n != NIVEL_INVALIDO && n != nivel_atual) {
        nivel_atual = n;
        print_nivel();
      }

      // Botão START — inicia aquecimento
      if (btn_pressionado(BTN_START)) {
        if (!aguarda_soltar(BTN_START)) {
          entrar_erro("BOTAO PRESO");
          break;
        }

        // Zera contador de fluxo com atomicidade
        noInterrupts();
        pulsos = 0;
        interrupts();

        Serial.print(F(">> AQUECENDO — alvo "));
        Serial.print(TEMP_ALVO, 0);
        Serial.print(F("C para "));
        Serial.print(VOLUME_ML[nivel_atual], 0);
        Serial.println(F(" mL"));

        t_pico = t;
        caldeira(true);
        leds(false, true); // vermelho = aquecendo
        mudar_estado(AQUECENDO);
      }
      break;
    }

    // ════════════════════════════════════════════════════════
    case AQUECENDO: {
      if (btn_pressionado(BTN_START)) {
        cancelar();
        break;
      }

      if (t > t_pico) t_pico = t;

      // Desliga a caldeira antes do alvo para aproveitar a inércia térmica
      if (caldeira_ligada && t >= TEMP_DESLIGA) caldeira(false);

      // Parte a bomba ao atingir o alvo, ou quando a temperatura já
      // passou do pico (a inércia não chegou ao alvo). Nesse caso o
      // preparo completa o aquecimento com a histerese.
      bool atingiu_alvo = (t >= TEMP_ALVO);
      bool passou_pico  = !caldeira_ligada && (t <= t_pico - TEMP_QUEDA_PICO);

      if (atingiu_alvo || passou_pico) {
        Serial.print(F(">> Temp OK ("));
        Serial.print(t, 1);
        Serial.print(F("C, pico "));
        Serial.print(t_pico, 1);
        Serial.print(atingiu_alvo ? F("C, alvo") : F("C, pico passou"));
        Serial.println(F(") — PREPARANDO"));

        pulsos_anterior = ler_pulsos();
        t_ultimo_pulso  = millis();
        bomba(true);
        leds(true, true);
        mudar_estado(PREPARANDO);
        break;
      }

      // Timeout de segurança no aquecimento
      if (millis() - t_estado >= T_MAX_AQUECIMENTO) {
        entrar_erro("TIMEOUT AQUECIMENTO");
      }
      break;
    }

    // ════════════════════════════════════════════════════════
    case PREPARANDO: {
      if (btn_pressionado(BTN_START)) {
        cancelar();
        break;
      }

      // Mantém temperatura durante o preparo (modo quente fixo)
      if      (t >= TEMP_ALVO)                  caldeira(false);
      else if (t <  TEMP_ALVO - TEMP_HISTERESE) caldeira(true);

      // Falta de fluxo: bomba ligada e nenhum pulso em T_SEM_FLUXO
      unsigned long p = ler_pulsos();
      if (p != pulsos_anterior) {
        pulsos_anterior = p;
        t_ultimo_pulso  = millis();
      } else if (millis() - t_ultimo_pulso >= T_SEM_FLUXO) {
        entrar_erro("SEM FLUXO");
        break;
      }

      // Volume atingido → finaliza
      float v = (p * 1000.0f) / PULSOS_POR_LITRO;
      if (v >= VOLUME_ML[nivel_atual]) {
        parar_tudo();
        Serial.print(F(">> CONCLUIDO — "));
        Serial.print(v, 0);
        Serial.println(F(" mL dispensados"));
        Serial.println(F(">> Cafe pronto!"));
        leds(true, false);
        mudar_estado(CONCLUIDO);
        break;
      }

      // Tempo máximo de preparo
      if (millis() - t_estado >= T_MAX_PREPARO) {
        entrar_erro("TIMEOUT PREPARO");
      }
      break;
    }

    // ════════════════════════════════════════════════════════
    case CONCLUIDO:
      // Pausa antes de aceitar novo preparo, sem bloquear o loop
      if (millis() - t_estado >= T_CONCLUIDO) mudar_estado(IDLE);
      break;

    // ════════════════════════════════════════════════════════
    case ERRO: {
      parar_tudo();
      // Pisca LED vermelho aguardando reset manual via START
      bool aceso = ((millis() - t_estado) / T_PISCA_ERRO) % 2 == 0;
      leds(false, aceso);

      if (btn_pressionado(BTN_START) && aguarda_soltar(BTN_START)) {
        Serial.println(F(">> Reset manual."));
        msg_erro = "";
        mudar_estado(IDLE); // se a falha persistir, a proteção global volta ao ERRO
      }
      break;
    }
  }
}
