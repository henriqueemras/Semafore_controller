#include <Wire.h>

// Definição dos pinos dos LEDs
#define LED_VERMELHO1 2
#define LED_VERMELHO2 3
#define LED_VERMELHO3 4
#define LED_VERMELHO4 5

#define LED_AMARELO1 6
#define LED_AMARELO2 7
#define LED_AMARELO3 8
#define LED_AMARELO4 9

#define LED_VERDE1 10
#define LED_VERDE2 11
#define LED_VERDE3 12
#define LED_VERDE4 13

// Estados dos semáforos (usando bits para representar LEDs)
uint8_t estadosSemaforos[4] = {0x00, 0x00, 0x00, 0x00}; // Inicialmente todos apagados

void setup() {
  // Configuração dos pinos como saída
  pinMode(LED_VERMELHO1, OUTPUT);
  pinMode(LED_VERMELHO2, OUTPUT);
  pinMode(LED_VERMELHO3, OUTPUT);
  pinMode(LED_VERMELHO4, OUTPUT);

  pinMode(LED_AMARELO1, OUTPUT);
  pinMode(LED_AMARELO2, OUTPUT);
  pinMode(LED_AMARELO3, OUTPUT);
  pinMode(LED_AMARELO4, OUTPUT);

  pinMode(LED_VERDE1, OUTPUT);
  pinMode(LED_VERDE2, OUTPUT);
  pinMode(LED_VERDE3, OUTPUT);
  pinMode(LED_VERDE4, OUTPUT);

  // Inicialização do barramento I2C
  Wire.begin(0x20); // Endereço do Arduino no barramento I2C
  Wire.onReceive(receberComando); // Configura a função de recepção de dados

  Serial.begin(9600);
  Serial.println("Arduino Nano escravo inicializado.");
}

void loop() {
  // Atualiza os LEDs com base no estado atual
  atualizarLeds();
}

void receberComando(int bytesRecebidos) {
  if (Wire.available()) {
    uint8_t comando = Wire.read(); // Lê o comando recebido

    // Decodifica o semáforo e estado
    uint8_t semaforo = (comando >> 4) & 0x0F; // Bits 7 a 4
    uint8_t estado = (comando >> 1) & 0x07;          // Bits 3 a 0

    if (semaforo >= 1 && semaforo <= 4) {
      estadosSemaforos[semaforo - 1] = estado; // Atualiza o estado do semáforo
      Serial.print("Semáforo ");
      Serial.print(semaforo);
      Serial.print(" atualizado para: ");
      Serial.println(estado, BIN);
    } else {
      Serial.println("Semáforo inválido!");
    }
  }
}

void atualizarLeds() {
  // Atualiza os LEDs com base no estado armazenado em estadosSemaforos
  for (int i = 0; i < 4; i++) {
    uint8_t estado = estadosSemaforos[i];

    // Define os pinos correspondentes ao semáforo
    int vermelho = LED_VERMELHO1 + i;
    int amarelo = LED_AMARELO1 + i;
    int verde = LED_VERDE1 + i;

    // Atualiza os LEDs
    digitalWrite(vermelho, estado & 0x04 ? HIGH : LOW); // Bit 2: LED vermelho
    digitalWrite(amarelo, estado & 0x02 ? HIGH : LOW);  // Bit 1: LED amarelo
    digitalWrite(verde, estado & 0x01 ? HIGH : LOW);    // Bit 0: LED verde
  }
}
