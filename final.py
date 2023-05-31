from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import serial

# Configurar la comunicación serial (reemplaza COM3 y 9600 con tus valores correctos)
ser = serial.Serial('COM3', 9600)

def send_value(value):
    # Convertir el valor entero a un carácter antes de enviarlo
    ser.write(str(value).encode())

app = QApplication([])

window = QWidget()
layout = QVBoxLayout()

buttons = [
    ("Brazo 1 Arriba", '1'),
    ("Brazo 1 Abajo", '2'),
    ("Brazo 2 Arriba", '3'),
    ("Brazo 2 Abajo", '4'),
    ("Abrir Garra", '5'),
    ("Cerrar Garra", '6'),
    ("Base Izquierda", '7'),
    ("Base Derecha", '8'),
]

for text, value in buttons:
    button = QPushButton(text)
    button.clicked.connect(lambda _, v=value: send_value(v))
    layout.addWidget(button)

window.setLayout(layout)
window.show()

app.exec_()
