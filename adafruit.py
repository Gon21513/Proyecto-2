# Import standard python modules
import time
import serial
import sys

# Import Adafruit IO REST client.
from Adafruit_IO import Client, MQTTClient

# holds the count for the feed
run_count = 0

# Set to your Adafruit IO key.
# Remember, your key is a secret,
# so make sure not to publish it when you publish this code!
ADAFRUIT_IO_KEY = 'aio_vwxQ43H30XOgoLHeuPRJyltRznWl'

# Set to your Adafruit IO username.
# (go to https://accounts.adafruit.com to find your username)
ADAFRUIT_IO_USERNAME = 'Gon21513'

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Define feed ids
FEED_ID1 = 'b1up'
FEED_ID2 = 'b1down'
FEED_ID3 = 'b2up'
FEED_ID4 = 'b2down'
FEED_ID5 = 'open'
FEED_ID6 = 'close'
FEED_ID7 = 'left'
FEED_ID8 = 'right'

ser = serial.Serial('COM3')
print(ser.name)

# Define callback functions which will be called when certain events happen.
def connected(client):
    # Subscribe to changes on a feed.
    client.subscribe(FEED_ID1)
    client.subscribe(FEED_ID2)
    client.subscribe(FEED_ID3)
    client.subscribe(FEED_ID4)
    client.subscribe(FEED_ID5)
    client.subscribe(FEED_ID6)
    client.subscribe(FEED_ID7)
    client.subscribe(FEED_ID8)
    print('Waiting for feed data...')

def disconnected(client):
    """Disconnected function will be called when the client disconnects."""
    sys.exit(1)

def message(client, feed_id, payload):
    """Message function will be called when a subscribed feed has a new value.
    The feed_id parameter identifies the feed, and the payload parameter has
    the new value.
    """
    ser.write(bytes(str(payload),'utf-8')) # Convert the payload to bytes and write it to the serial
    print('Feed {0} received new value: {1}'.format(feed_id, payload))

# Create an MQTT client instance.
client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Setup the callback functions defined above.
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

# Connect to the Adafruit IO server.
client.connect()

# The first option is to run a thread in the background so you can continue
# doing things in your program.
client.loop_blocking()

while True:
    run_count = ser.readline()
    print(run_count)
