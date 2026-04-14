/*
 * ================================================================
 *   Dolce Gusto Mini Me — Controlador Arduino Micro  v1.5
 * ================================================================
 *
 * PINAGEM:
 *  A0  — NTC 30K  (NTC entre VCC e A0; resistor 10K entre A0 e GND)
 *  D2  — Chave de nível fio 1
 *  D3  — Chave de nível fio 2
 *  D4  — Chave de nível fio 3
 *  D5  — Botão ON/OFF original
 *  D6  — LED Vermelho original
 *  D7  — Fluxômetro (INT0 — borda de subida)
 *        No Arduino Micro usar D1, D2, D3 ou D7 (pinos com interrupt)
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
 * ================================================================
 */

#include <math.h>

// ── Pinos
const uint8_t PIN_NTC            = A0;
const uint8_t PIN_1_NIVEL        = 2;
const uint8_t PIN_2_NIVEL        = 3;
const uint8_t PIN_3_NIVEL        = 4;
const uint8_t BTN_START          = 5;
const uint8_t PIN_LED_VERMELHO   = 6;
const uint8_t PIN_FLUXO          = 7;   // INT0
const uint8_t PIN_RELAY_CALDEIRA = 8;   // CH1
const uint8_t PIN_RELAY_BOMBA    = 9;   // CH2
const uint8_t PIN_LED_VERDE      = 10;

// ── Relay
const uint8_t RELAY_ON  = LOW;
const uint8_t RELAY_OFF = HIGH;

// ── NTC 100K — Steinhart-Hart (β simplificado)
const float NTC_NOMINAL = 100000.0f; // Ω a 25 °C
const float NTC_BETA    = 3950.0f;  // coeficiente B — ajuste após calibração
const float NTC_T0      = 298.15f;  // 25 °C em Kelvin
const float R_DIVISOR   = 10000.0f; // resistor série 10 KΩ

// ── Temperatura (°C)
const float TEMP_DESLIGA   = 75.0f;  // °C — desliga o elemento aqui
const float TEMP_ALVO      = 88.0f;  // °C — alvo de aquecimento
const float TEMP_HISTERESE =  5.0f;  // °C — banda liga/desliga da caldeira
const float TEMP_SEGURANCA = 110.0f; // °C — desliga por segurança
const unsigned long T_MAX_AQUECIMENTO = 120000UL; // 2 min timeout

// ── Fluxo
const float PULSOS_POR_LITRO = 3375.0f;

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
const uint8_t TOTAL_NIVEIS = 7;

// ── Estado da máquina
enum Estado : uint8_t {
  IDLE,
  AQUECENDO,
  PREPARANDO,
  CONCLUIDO,
  ERRO
};

Estado     estado       = IDLE;
uint8_t    nivel_atual  = 0; 

volatile unsigned long pulsos        = 0;
unsigned long          t_aquecimento = 0;
const char*            msg_erro      = "";

// ── ISR do fluxômetro
void ISR_fluxo() { pulsos++; }

// ── Leitura de temperatura (°C)
float temperatura() {
  int raw = analogRead(PIN_NTC);
  if (raw <= 0 || raw >= 1023) return -99.0f; // sensor desconectado
  float R   = R_DIVISOR * (1023.0f - raw) / (float)raw;
  float inv = (1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(R / NTC_NOMINAL);
  return (1.0f / inv) - 273.15f;
}

// ── Volume dispensado em mL
float volume_ml() {
  noInterrupts();
  unsigned long p = pulsos;
  interrupts();
  return (p * 1000.0f) / PULSOS_POR_LITRO;
}

// ── Define o nível de café usando 3 pinos
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
  return 0;
}

// ── Controle de saídas
inline void caldeira(bool on) { digitalWrite(PIN_RELAY_CALDEIRA, on ? RELAY_ON : RELAY_OFF); }
inline void bomba(bool on)    { digitalWrite(PIN_RELAY_BOMBA,    on ? RELAY_ON : RELAY_OFF); }

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

// ── Aguarda soltar o botão (evita múltiplos registros)
void aguarda_soltar(uint8_t pino) {
  while (digitalRead(pino) == LOW) delay(10);
  delay(20);
}

// ── Imprime nível atual no Serial
void print_nivel() {
  Serial.print(F(">> Nivel: "));
  Serial.print(nivel_atual + 1);
  Serial.print(F("/7 — "));
  Serial.print(VOLUME_ML[nivel_atual], 0);
  Serial.println(F(" mL"));
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Saídas
  pinMode(PIN_RELAY_CALDEIRA, OUTPUT);
  pinMode(PIN_RELAY_BOMBA,    OUTPUT);
  pinMode(PIN_LED_VERDE,      OUTPUT);
  pinMode(PIN_LED_VERMELHO,   OUTPUT);
  parar_tudo();
  leds(false, false);

  // Entradas com pull-up interno (botão → GND quando pressionado)
  pinMode(BTN_START,   INPUT_PULLUP);
  pinMode(PIN_1_NIVEL, INPUT_PULLUP);
  pinMode(PIN_2_NIVEL, INPUT_PULLUP);
  pinMode(PIN_3_NIVEL, INPUT_PULLUP);

  // Fluxômetro na interrupção INT0
  pinMode(PIN_FLUXO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLUXO), ISR_fluxo, RISING);

  Serial.println(F("=== Dolce Gusto Ctrl v1.3 ==="));
  print_nivel();

  leds(true, false); // verde = pronto
}

// ─────────────────────────────────────────────────────────────
//  LOOP — máquina de estados
// ─────────────────────────────────────────────────────────────
void loop() {

  // ── Debug periódico
  static unsigned long t_dbg = 0;
  if (millis() - t_dbg >= 1000UL) {
    Serial.print(F("T:"));
    Serial.print(temperatura(), 1);
    Serial.print(F("C  V:"));
    Serial.print(volume_ml(), 0);
    Serial.print(F("mL  P:"));
    Serial.print(pulsos);
    Serial.print(F("  T:"));
    Serial.print(VOLUME_ML[nivel_atual], 0);
    Serial.print(F("mL  E:"));
    Serial.println(estado);
    t_dbg = millis();
  }

  // ── Proteção global de temperatura
  if (temperatura() >= TEMP_SEGURANCA && estado != ERRO) {
    parar_tudo();
    msg_erro = "TEMP CRITICA";
    estado = ERRO;
    return;
  }

  // ── Máquina de estados
  switch (estado) {

    // ════════════════════════════════════════════════════════
    case IDLE: {
      leds(true, false);

      // Lê chave de nível continuamente — reflete posição atual
      uint8_t n = ler_nivel();
      if (n != nivel_atual) {
        nivel_atual = n;
        print_nivel();
      }

      // Botão START — inicia aquecimento
      if (btn_pressionado(BTN_START)) {
        aguarda_soltar(BTN_START);

        // Zera contador de fluxo com atomicidade
        noInterrupts();
        pulsos = 0;
        interrupts();

        Serial.print(F(">> AQUECENDO — alvo "));
        Serial.print(TEMP_ALVO, 0);
        Serial.print(F("C para "));
        Serial.print(VOLUME_ML[nivel_atual], 0);
        Serial.println(F(" mL"));

        caldeira(true);
        leds(false, true); // vermelho = aquecendo
        t_aquecimento = millis();
        estado = AQUECENDO;
      }
      break;
    }  

    // ════════════════════════════════════════════════════════
    case AQUECENDO: {

      // Cancelamento pelo operador
      if (btn_pressionado(BTN_START)) {
        aguarda_soltar(BTN_START);
        parar_tudo();
        Serial.println(F(">> Preparo cancelado."));
        estado = IDLE;
        break;
      }

      float t = temperatura();

      // Desliga a caldeira para evitar overshoot
      if (t >= TEMP_DESLIGA) caldeira(false);

      // Inicia a bomba quando atingir o alvo
      if (t >= TEMP_ALVO) {
        Serial.print(F(">> Temp OK ("));
        Serial.print(t, 1);
        Serial.println(F("C) — PREPARANDO"));
        bomba(true);
        leds(true, true);
        estado = PREPARANDO;
        break;
      }

      // Timeout de segurança no aquecimento
      if (millis() - t_aquecimento >= T_MAX_AQUECIMENTO) {
        msg_erro = "TIMEOUT AQUECIMENTO";
        parar_tudo();
        estado = ERRO;
      }
      break;
    }

    // ════════════════════════════════════════════════════════
    case PREPARANDO: {
      // Cancelamento pelo operador
      if (btn_pressionado(BTN_START)) {
        aguarda_soltar(BTN_START);
        parar_tudo();
        Serial.println(F(">> Preparo cancelado."));
        estado = IDLE;
        break;
      }

      // Mantém temperatura durante o preparo (modo quente fixo)
      float t = temperatura();
      if      (t >= TEMP_ALVO)                   caldeira(false);
      else if (t <  TEMP_ALVO - TEMP_HISTERESE)  caldeira(true);

      // Volume atingido → finaliza
      if (volume_ml() >= VOLUME_ML[nivel_atual]) {
        parar_tudo();
        Serial.print(F(">> CONCLUIDO — "));
        Serial.print(volume_ml(), 0);
        Serial.println(F(" mL dispensados"));
        estado = CONCLUIDO;
      }
      break;
    }

    // ════════════════════════════════════════════════════════
    case CONCLUIDO:
      leds(true, false);
      Serial.println(F(">> Cafe pronto!"));
      delay(4000);
      estado = IDLE;
      break;

    // ════════════════════════════════════════════════════════
    case ERRO:
      parar_tudo();
      Serial.print(F("ERRO: "));
      Serial.println(msg_erro);
      // Pisca LED vermelho aguardando reset manual via START
      leds(false, true);  delay(300);
      leds(false, false); delay(300);
      if (btn_pressionado(BTN_START)) {
        aguarda_soltar(BTN_START);
        Serial.println(F(">> Reset manual."));
        msg_erro = "";
        estado   = IDLE;
      }
      break;
  }
}