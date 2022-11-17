#include <Servo.h>

#define n_keys 9
#define n_bytes ((int) 1 + (n_keys - 1) / 8)
#define n_shift 8 - n_keys % 8
const uint8_t servo_pins[n_keys] = {44, 46, 39, 43, 48, 50, 51, 42, 52}; // Pines digitales
// const uint8_t keys[n_keys] = {22, 23, 24, 25, 26, 27, 28, 29, 30}; // Pines digitales

Servo servos[n_keys]; // Arreglo de servos
int pos_1[2] = {100, 135}; // dedo mas cerca de boquilla
int pos_2[2] = {85, 115};
int pos_3[2] = {58, 90};
int pos_5[2] = {95, 135};
int pos_4[2] = {155, 140};
int pos_6[2] = {95, 130};
int pos_7[2] = {80, 130};
int pos_8[2] = {60, 100};
int pos_9[2] = {73, 110}; // dedo mas lejos de boquilla

//int angle_up[n_keys] = {100,85,58,155,100,95,90,65,80};
//int angle_down[n_keys] = {135,115,90,120,135,120,130,100,110};
int angle_up[n_keys] = {pos_1[0], pos_2[0], pos_3[0], pos_4[0], pos_5[0], pos_6[0], pos_7[0], pos_8[0], pos_9[0]};
int angle_down[n_keys] = {pos_1[1], pos_2[1], pos_3[1], pos_4[1], pos_5[1], pos_6[1], pos_7[1], pos_8[1], pos_9[1]};
byte serial_msg[n_bytes]; // Bytes de lectura serial
bool pressed; // Booleano para indicar el estado de una llave


void setup() {
  for (uint8_t i = 0; i < n_keys; i++) {

    // Inicializa pines como output (para debug con LEDs)
    // pinMode(keys[i], OUTPUT);

    // Asigna pines a servos y establece min. (500us) y max (2400us) ancho de pulso
    servos[i].attach(servo_pins[i], 500, 2400);

    // LLeva servos a posición neutral
    for (uint8_t i = 0; i < n_keys; i++) {
    servos[i].write(angle_up[i]);
    }
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
      if (pressed) servos[i].write(angle_down[i]);
      else servos[i].write(angle_up[i]);
    }
  }
}
