from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import serial

# Establece la comunicación con un dispositivo a través del puerto serial 'COM3' a una velocidad de 9600 baudios
ser = serial.Serial('COM3', 9600)

# Define una función que envía un valor al dispositivo a través del puerto serial
def send_value(value):
    # Convierte el valor en una cadena de caracteres, codifica esa cadena en bytes, y luego envía esos bytes al dispositivo
    ser.write(str(value).encode())

# Crea una nueva aplicación PyQt5
app = QApplication([])

# Crea un nuevo widget que actuará como la ventana principal de la aplicación
window = QWidget()

# Crea un nuevo layout vertical y se lo asigna a la ventana
layout = QVBoxLayout()

# Define los botones y sus valores asociados
buttons = [
    ("Brazo 1 Arriba", '1'),
    ("Brazo 1 Abajo", '2'),
    ("Brazo 2 Abajo", '3'),
    ("Brazo 2 Arriba", '4'),
    ("Abrir Garra", '5'),
    ("Cerrar Garra", '6'),
    ("Base Izquierda", '7'),
    ("Base Derecha", '8'),
]

# Crea un botón para cada par de valores en la lista 'buttons'
for text, value in buttons:
    # Crea un nuevo botón con el texto dado
    button = QPushButton(text)
    # Conecta la señal 'clicked' del botón a la función 'send_value'
    button.clicked.connect(lambda _, v=value: send_value(v))
    # Añade el botón al layout
    layout.addWidget(button)

# Asigna el layout a la ventana
window.setLayout(layout)
# Muestra la ventana
window.show()

# Inicia el bucle de eventos de la aplicación
app.exec_()