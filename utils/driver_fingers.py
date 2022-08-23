from PyQt5 import QtCore
import threading
import serial
from time import sleep


flute_dict = {'C3':  '00000 0000',
              'C#3': '00000 0000',
              'D3':  '11110 1110',
              'D#3': '11110 1111',
              'E3':  '11110 1101',
              'F3':  '11110 1001',
              'F#3': '11110 0011',
              'G3':  '11110 0001',
              'G#3': '11111 0001',
              'A3':  '11100 0001',
              'A#3': '11000 1001',
              'B3':  '11000 0001',
              'C4':  '10000 0001',
              'C#4': '00000 0001',
              'D4':  '01110 1110',
              'D#4': '10110 1111',
              'E4':  '11110 1101',
              'F4':  '11110 1001',
              'F#4': '11110 0011',
              'G4':  '11110 0001',
              'G#4': '11111 0001',
              'A4':  '11100 0001',
              'A#4': '11000 1001',
              'B4':  '11000 0001',
              'C5':  '10000 0001',
              'C#5': '00000 0001',
              'D5':  '10110 0001',
              'D#5': '11111 1111',
              'E5':  '11100 1101',
              'F5':  '11010 1001',
              'F#5': '11010 0011',
              'G5':  '01110 0001',
              'G#5': '00111 0001',
              'A5':  '10100 1001',
              'A#5': '00000 0000',
              'B5':  '00000 0000',
              'C6':  '01111 1000'}

quena_dict = {'G3':  '00 1111111',
              'G#3': '00 0000000',
              'A3':  '00 1111110',
              'A#3': '00 0000000',
              'B3':  '00 1111100',
              'C4':  '00 1111000',
              'C#4': '00 0000000',
              'D4':  '00 1110000',
              'D#4': '00 0000000',
              'E4':  '00 1100000',
              'F4':  '00 0000000',
              'F#4': '00 1000000',
              'G4':  '00 0000110',
              'G#4': '00 0000000',
              'A4':  '00 1111110',
              'A#4': '00 0000000',
              'B4':  '00 1111100',
              'C5':  '00 1111000',
              'C#5': '00 0000000',
              'D5':  '00 1110000',
              'D#5': '00 0000000',
              'E5':  '00 1100000',
              'F5':  '00 1000000',
              'F#5': '00 0000000',
              'G5':  '00 0000110',
              'G#5': '00 0111111',
              'A5':  '00 0111110',
              'A#5': '00 0000000',
              'B5':  '00 1100010',
              'C6':  '00 1101111',
              'C#6': '00 1101001',
              'D6':  '00 1010000',
              'D#6': '00 0011110',
              'E6':  '00 0000000',
              'F6':  '00 0100000',
              'F#6': '00 1111100'}

test_dict = {'1': '000000001',
             '2': '000000010',
             '3': '000000100',
             '4': '000001000',
             '5': '000010000',
             '6': '000100000',
             '7': '001000000',
             '8': '010000000',
             '9': '100000000',
             '0': '000000000'}

instrument_dicts = {'flute': flute_dict,
                    'quena': quena_dict,
                    'test':  test_dict}


class FingersDriver(QtCore.QThread):

    def __init__(self, host, running, connected=True, instrument='flute'):

        # Variables de threading
        QtCore.QThread.__init__(self)
        self.running = running
        self.connected = connected

        # Variables de músico
        self.instrument = instrument
        self.note_dict = instrument_dicts[instrument]
        self.state = '000000000'

        # Configura evento de cambio
        self.changeEvent = threading.Event()
        self.changeEvent.clear()

        # Configura comunicación serial
        if self.connected:
            self.serial_port = serial.Serial(host, 115200, timeout=1)

    def run(self):
        if self.connected:
            while self.running.is_set():
                self.changeEvent.wait(timeout=1)
                if self.changeEvent.is_set():

                    # Ejecuta acción de dedos de bajo nivel
                    self.serial_port.write(self.state)

                    # Limpia el flag de cambio
                    self.changeEvent.clear()

            # Finaliza el thread
            self.stop()
            self.serial_port.close()
            print('Fingers Driver thread killed')

    def stop(self):
        # Suelta todas las llaves
        self.serial_port.write(b'\0\0')

    def request_finger_action(self, req_note: str):
        """
        Función para llamar desde FingersController
        :param req_note: string indicando la nota que se desea.
        """

        # Modifica el estado de servos interno según un diccionario
        servo = translate_fingers_to_servo(instrument_dicts[self.instrument][req_note])
        self.state = int(servo.replace(' ', ''), 2).to_bytes(2, byteorder='big')

        # Levanta el flag para generar un cambio en el Thread principal
        self.changeEvent.set()


def translate_fingers_to_servo(note_bits):
    """
    Intercambia las llaves 4 y 5 por disposición geométrica.
    - llave nueva 4 <-- llave antigua 5.
    - llave nueva 5 <-- llave antigua 4.
    :param note_bits:
    :return:
    """
    servo_bits = list(note_bits)
    servo_bits[3] = note_bits[4]
    servo_bits[4] = note_bits[3]

    return ''.join(servo_bits)


if __name__ == "__main__":

    test_finger_event = threading.Event()
    test_finger_event.set()
    test_host = '/dev/cu.usbserial-142420'
    test_driver = FingersDriver(test_host, test_finger_event, instrument='flute')

    test_driver.start()
    sleep(1)

    for note in flute_dict.keys():

        # DEBUGGING: Muestra detalles sobre el mensaje enviado
        print(f'Nota: {note}')
        test_driver.request_finger_action(note)
        sleep(0.1)
        print(f"Binario: {int.from_bytes(test_driver.state, byteorder='big'):09b}\n")

        input()


