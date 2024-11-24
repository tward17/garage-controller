import network
import time
import machine
import dht
import asyncio
import secrets
import config
from umqtt.simple import MQTTClient

# WiFi connection details
SSID = secrets.SSID
PASSWORD = secrets.WIFI_PASSWORD

# MQTT Broker details
MQTT_BROKER = config.MQTT_BROKER_ADDRESS 
MQTT_PORT = config.MQTT_BROKER_PORT
MQTT_CLIENT_ID = 'garage_controller'

# MQTT Sub Topics
GARAGE_DOOR_ACTIVATE = b'garage/door/activate'
GARAGE_LIGHT_ACTIVATE = b'garage/light/activate'

#MQTT Pub Topics
GARAGE_DOOR_STATUS = b'garage/door/status'
GARAGE_LIGHT_STATUS = b'garage/light/status'
GARAGE_ENVIRONMENT_TEMPERATURE = b'garage/environment/temperature'
GARAGE_ENVIRONMENT_HUMIDITY = b'garage/environment/humidity'
GARAGE_MOTION = b'garage/motion/status'

#GPIO Pin Assignments
DOOR_BUTTON = 28
DOOR_RELAY = 18
LIGHT_RELAY = 19
DOOR_SENSOR = 10
ENVIRONMENT_SENSOR = 2
PIR_SENSOR = 6

# Setup the button GPIO pin as input with an internal pull-up resistor
button = machine.Pin(DOOR_BUTTON, machine.Pin.IN, machine.Pin.PULL_UP)

# Setup the relay GPIO pin as output (relay ON when pin is HIGH, OFF when pin is LOW)
relay = machine.Pin(DOOR_RELAY, machine.Pin.OUT)
relay.value(0)  # Set relay initially OFF (0)
relay = machine.Pin(LIGHT_RELAY, machine.Pin.OUT)
relay.value(0)  # Set relay initially OFF (0)

pir = machine.Pin(PIR_SENSOR, machine.Pin.IN)

class Environment:
    def __init__(self, mqtt_client):
        self.client = mqtt_client
        self.sensor = dht.DHT22(machine.Pin(ENVIRONMENT_SENSOR))
        self.temperature = int(0)
        self.humidity = int(0)

    async def monitor_status(self):
        while True:
            try:
                print('Checking Environment')
                self.sensor.measure()
                current_temperature = round(self.sensor.temperature())
                current_humidity = round(self.sensor.humidity())

                if self.temperature != current_temperature:
                    self.update_temperature(current_temperature)
                if self.humidity != current_humidity:
                    self.update_humidity(current_humidity)

                del current_temperature, current_humidity

                await asyncio.sleep(5)

            except OSError as e:
                print('Failed to read sensor.')

    def update_temperature(self, temperature):
        self.temperature = temperature
        print('Temperature change to: ' + str(self.temperature))
        publish_message(self.client,GARAGE_ENVIRONMENT_TEMPERATURE, str(self.temperature))

    def update_humidity(self, humidity):
        self.humidity = humidity
        print('Humidity change to: ' + str(self.humidity))
        publish_message(self.client,GARAGE_ENVIRONMENT_HUMIDITY,str(self.humidity))

class Door:
    def __init__(self, mqtt_client):
        self.client = mqtt_client
        self.state = ''
        self.sensor = machine.Pin(DOOR_SENSOR, machine.Pin.IN, machine.Pin.PULL_UP)

    async def monitor_status(self):
        while True:
            print('Checking Garage Door state')
            current_state = 'OPEN' if self.sensor.value() == 1 else 'CLOSED'

            if self.state != current_state:
                self.update_state(current_state)

            del current_state

            await asyncio.sleep(2)

    def update_state(self, state):
        self.state = state
        print('Door state changed to: ' + self.state)
        publish_message(self.client, GARAGE_DOOR_STATUS, self.state)

# Connect to Wi-Fi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print('Connecting to WiFi...')
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            time.sleep(1)
        print('WiFi connected')
        print('IP address:', wlan.ifconfig()[0])

# Callback function for handling incoming messages
def mqtt_callback(topic, msg):
    print('Received message on topic:', topic.decode())
    print('Message:', msg.decode())
    
    if topic == GARAGE_DOOR_ACTIVATE:
        relay_action(DOOR_RELAY, msg)
    elif topic == GARAGE_LIGHT_ACTIVATE:
        relay_action(LIGHT_RELAY, msg)
            
# Connect to MQTT Broker
def connect_mqtt():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
    client.set_callback(mqtt_callback)
    client.connect()
    print('Connected to MQTT broker:', MQTT_BROKER)
    return client

# Publish a message to MQTT topic
def publish_message(client, topic, message):
    print('Publishing message to topic:', topic.decode())
    client.publish(topic, message)

# Action Relay
def relay_action(relay, action):
    relay = machine.Pin(relay, machine.Pin.OUT)
    
    if action == b'ON':
        print('Turning relay ON')
        relay.value(1)  # Turn relay ON
    elif action == b'OFF':
        print('Turning relay OFF')
        relay.value(0)  # Turn relay OFF
    elif action == b'TOGGLE':
        print('Toggle relay')
        relay.toggle()
    else:
        print('Unknown command for relay:', action)


# Main logic to run the MQTT client
async def main():
    # Connect to Wi-Fi
    connect_wifi()

    # Connect to MQTT broker
    client = connect_mqtt()

    # Subscribe to necessary topics
    print('Subscribing to topics:')
    client.subscribe(GARAGE_DOOR_ACTIVATE)
    client.subscribe(GARAGE_LIGHT_ACTIVATE)

    # Publish an initial message to the MQTT topic
    #publish_message(client, MQTT_TOPIC_PUB, b'Hello from MicroPython!')

    # Initialise Environment object
    enviro = Environment(client)
    asyncio.create_task(enviro.monitor_status())

    # Initialise Door object
    door = Door(client)
    asyncio.create_task(door.monitor_status())

    try:
        while True:
            # Check for any incoming MQTT messages
            client.check_msg()

            # Check if the button is pressed (LOW signal on the GPIO pin)
            if button.value() == 0:  # Button is pressed (active low)
                print('Button pressed!')
                # Debounce the button press (wait for release)
                while button.value() == 0:
                    time.sleep(0.1)
            
            if pir.value(1):
                print('motion detected')
            else:
                print('No Motion')

            await asyncio.sleep(0.1)
            #time.sleep(0.1)  # Small delay to prevent overloading the loop

    except KeyboardInterrupt:
        print('Disconnecting...')
        client.disconnect()

if __name__ == '__main__':
    asyncio.run(main())
