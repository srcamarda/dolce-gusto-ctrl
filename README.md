
Controladora feita para substituir a placa original da cafeteira Dolce Gusto Mini Me.

Hardware necessário:
- 1x Arduino Micro
- 1x Relé octoacoplador de 2 canais, ou 2x relé simples
- 1x Sensor Hall Effect 3144 (utilizado para substituir o sensor original do fluxômetro)
- 1x NTC 100K B3950 + 1x resistor 10K (divisor de tensão para leitura da temperatura)
- 1x Fonte 5V (alimentação do Arduino)

No meu projeto a alavanca de seleção Quente/Frio e o solenóide foram retirados. Podem ser utilizados com algumas modificações, mas é necessário uma fonte de 15V.

## Pinagem

| Pino | Função |
|------|--------|
| A0   | NTC 100K (NTC entre VCC e A0; resistor 10K entre A0 e GND) |
| D2   | Chave de nível, fio 1 (INPUT_PULLUP) |
| D3   | Chave de nível, fio 2 (INPUT_PULLUP) |
| D4   | Chave de nível, fio 3 (INPUT_PULLUP) |
| D5   | Botão ON/OFF original (INPUT_PULLUP) |
| D6   | LED vermelho original (resistor 330 Ω) |
| D7   | Fluxômetro, sensor Hall 3144 (interrupção INT6) |
| D8   | Relé CH1 → caldeira 1400 W AC |
| D9   | Relé CH2 → bomba AC |
| D10  | LED verde original (resistor 330 Ω) |

O módulo de relé é ativo em LOW.

## Proteções

- Erro e desligamento total se o NTC estiver aberto, em curto ou fora de faixa
- Desligamento acima de 110 °C
- Timeout de aquecimento (2 min) e de preparo (90 s)
- Erro se a bomba ficar 3 s ligada sem pulsos do fluxômetro (reservatório vazio, mangueira solta)
- Watchdog de 2 s

Recomenda-se manter o fusível térmico / termostato original em série com a caldeira, como proteção independente do Arduino.
