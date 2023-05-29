import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QHBoxLayout
import serial

class AppWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.ser = serial.Serial('COM3', 9600, timeout=1)  # Inicializa la comunicación serial
        self.initUI()

    def initUI(self):
        self.setWindowTitle('PIC16F887 Interface')

        layout = QVBoxLayout()

        servo_layout = QHBoxLayout()
        servo_layout.addWidget(QLabel("Select servo:"))

        # Crea el botón para el Servo 1
        self.servo_button1 = QPushButton('Servo 1')
        self.servo_button1.clicked.connect(lambda: self.send_servo(1))  # Conecta el botón al método send_servo con el valor 1
        servo_layout.addWidget(self.servo_button1)

        # Crea el botón para el Servo 2
        self.servo_button2 = QPushButton('Servo 2')
        self.servo_button2.clicked.connect(lambda: self.send_servo(2))  # Conecta el botón al método send_servo con el valor 2
        servo_layout.addWidget(self.servo_button2)

        # Crea el botón para el Servo 3
        self.servo_button3 = QPushButton('Servo 3')
        self.servo_button3.clicked.connect(lambda: self.send_servo(3))  # Conecta el botón al método send_servo con el valor 3
        servo_layout.addWidget(self.servo_button3)

        # Crea el botón para el Servo 4
        self.servo_button4 = QPushButton('Servo 4')
        self.servo_button4.clicked.connect(lambda: self.send_servo(4))  # Conecta el botón al método send_servo con el valor 4
        servo_layout.addWidget(self.servo_button4)

        layout.addLayout(servo_layout)

        self.value_input = QLineEdit()
        layout.addWidget(QLabel("Enter value (0-255):"))
        layout.addWidget(self.value_input)

        self.send_value_button = QPushButton('Send Value')
        self.send_value_button.clicked.connect(self.send_value)  # Conecta el botón al método send_value
        layout.addWidget(self.send_value_button)

        self.setLayout(layout)

    def send_servo(self, servo):
        servo_as_int = int(servo)  # Convierte el número de servo en entero
        servo_as_byte = servo_as_int.to_bytes(1, 'little')  # Convierte el número de servo en byte
        self.ser.write(servo_as_byte)  # Envía el número de servo como byte por el puerto serie
        print('Servo', servo_as_int, 'selected')  # Imprime el número del servo seleccionado

    def send_value(self):
        value = self.value_input.text()
        if value.isdigit() and 0 <= int(value) <= 255:
            valid_value = int(value)  # Convierte el valor en entero si es válido
            value_as_byte = valid_value.to_bytes(1, 'little')  # Convierte el valor en byte
            self.ser.write(value_as_byte)  # Envía el valor como byte por el puerto serie
            if valid_value == 1:  # Si el valor es 1, lo envía una vez más
                self.ser.write(value_as_byte)
            print('Value', valid_value, 'sent')  # Imprime el valor enviado
        else:
            QMessageBox.critical(self, "Error", f"Invalid value. Please enter a number between 0 and 255.")
            return

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = AppWindow()
    window.show()
    sys.exit(app.exec_())
