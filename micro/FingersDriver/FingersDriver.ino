#include <Servo.h>

#define n_keys 9
#define n_bytes ((int) 1 + (n_keys - 1) / 8)
#define n_shift 8 - n_keys % 8
const uint8_t servo_pins[n_keys] = {44, 46, 47, 48, 43, 50, 51, 42, 53}; // Pines digitales
// const uint8_t keys[n_keys] = {22, 23, 24, 25, 26, 27, 28, 29, 30}; // Pines digitales

Servo servos[n_keys]; // Arreglo de servos
int angle_down[n_keys] = {120,120,120,120,120,120,120,120,120};
int angle_up[n_keys] = {130,130,130,130,130,130,130,130,130};
byte serial_msg[n_bytes]; // Bytes de lectura serial
bool pressed; // Booleano para indicar el estado de una llave


void setup() {
  for (uint8_t i = 0; i < n_keys; i++) {

    // Inicializa pines como output (para debug con LEDs)
    // pinMode(keys[i], OUTPUT);

    // Asigna pines a servos y establece min. (500us) y max (2400us) ancho de pulso
    servos[i].attach(servo_pins[i], 500, 2400);

    // LLeva servos a posición neutral
  }

  // Inicializa comunicación serial
  Serial.begin(115200);
}

void loop() {
  // Revisa si ha llegado algún dato serial
  if (Serial.available() > 1) {
    // Serial.println("Hola");
    // Guarda los 2 bytes recibidos
    Serial.readBytesUntil(-1, serial_msg, n_bytes);

    for (uint8_t i = 0; i < n_keys; i++) {

      // Ubicamos bit correspondiente
      pressed = bitRead(serial_msg[(i + n_shift) / 8], 7 - (i + n_shift) % 8);
      
      // Activa y desactiva LEDs (para debug)
      // digitalWrite(keys[i], pressed);
      
      // Acciona y desacciona los servos
      if (pressed) servos[i].write(angle_up[i]);
      else servos[i].write(angle_down[i]);
    }
  }
}
