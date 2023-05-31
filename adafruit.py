# Importa los módulos estándar de Python para el manejo del tiempo, la comunicación serie y el sistema
import time
import serial
import sys

# Importa el cliente REST de Adafruit IO.
from Adafruit_IO import Client, MQTTClient

# Mantiene la cuenta para la feed
run_count = 0

# Define tu clave de Adafruit IO.
# Recuerda, tu clave es secreta,
# ¡Así que asegúrate de no publicarla cuando publiques este código!
ADAFRUIT_IO_KEY = 'aio_vwxQ43H30XOgoLHeuPRJyltRznWl'

# Define tu nombre de usuario de Adafruit IO.
ADAFRUIT_IO_USERNAME = 'Gon21513'

# Crea una instancia del cliente REST.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Define los IDs de las feeds
FEED_ID1 = 'b1up'
FEED_ID2 = 'b1down'
FEED_ID3 = 'b2up'
FEED_ID4 = 'b2down'
FEED_ID5 = 'open'
FEED_ID6 = 'close'
FEED_ID7 = 'left'
FEED_ID8 = 'right'

# Configura la conexión serial a través del puerto COM3
ser = serial.Serial('COM3')
print(ser.name)

# Define las funciones de devolución de llamada que se llamarán cuando ocurran ciertos eventos.
def connected(client):
    # Suscríbete a los cambios en las feeds.
    client.subscribe(FEED_ID1)
    client.subscribe(FEED_ID2)
    client.subscribe(FEED_ID3)
    client.subscribe(FEED_ID4)
    client.subscribe(FEED_ID5)
    client.subscribe(FEED_ID6)
    client.subscribe(FEED_ID7)
    client.subscribe(FEED_ID8)
    print('Esperando datos de la feed...')

def disconnected(client):
    """La función disconnected se llamará cuando el cliente se desconecte."""
    sys.exit(1)

def message(client, feed_id, payload):
    """La función message se llamará cuando una feed suscrita tenga un nuevo valor.
    El parámetro feed_id identifica la feed, y el parámetro payload tiene
    el nuevo valor.
    """
    # Convierte el payload a bytes y lo escribe en la serie
    ser.write(bytes(str(payload),'utf-8')) 
    print('La feed {0} recibió un nuevo valor: {1}'.format(feed_id, payload))

# Crea una instancia del cliente MQTT.
client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Configura las funciones de devolución de llamada definidas anteriormente.
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

# Conéctate al servidor de Adafruit IO.
client.connect()

# La primera opción es ejecutar un hilo en segundo plano para que puedas continuar
# haciendo cosas en tu programa.
client.loop_blocking()

while True:
    # Lee la línea de la serie y la imprime
    run_count = ser.readline()
    print(run_count)