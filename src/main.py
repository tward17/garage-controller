import network
import time
import machine
import dht
import asyncio
import secrets
import config
import sys
from umqtt.simple import MQTTClient

# WiFi connection details
SSID = secrets.SSID
PASSWORD = secrets.WIFI_PASSWORD

# MQTT Broker details
MQTT_BROKER = config.MQTT_BROKER_ADDRESS 
MQTT_PORT = config.MQTT_BROKER_PORT
MQTT_CLIENT_ID = 'garage_controller'

# HEARTBEAT
HEARTBEAT_MINUTES = config.MONITOR_HEARTBEAT_MINUTES

# FEATURES ENABLED
MONITOR_MOTION = config.MONITOR_MOTION_SENSOR_ENABLED
MONITOR_MOTION_POLLING_SECONDS = config.MONITOR_MOTION_POLLING_SECONDS
MONITOR_DHT22 = config.MONITOR_DHT22_ENABLED
MONITOR_DHT22_POLLING_SECONDS = config.MONITOR_DHT22_POLLING_SECONDS
MONITOR_DOOR_POSITION = config.MONITOR_GARAGE_DOOR_POSTION_SENSOR_ENABLED
MONITOR_DOOR_POLLING_SECONDS = config.MONITOR_GARAGE_DOOR_POSITION_POLLING_SECONDS
MONITOR_BUTTON_PRESS = config.BUTTON_ENABLED

# MQTT Sub Topics
GARAGE_DOOR_ACTIVATE = 'garage/door/activate'
GARAGE_LIGHT_ACTIVATE = 'garage/light/activate'

#MQTT Pub Topics
GARAGE_DOOR_STATUS = 'garage/door/status'
GARAGE_LIGHT_STATUS = 'garage/light/status'
GARAGE_DHT22_TEMPERATURE = 'garage/environment/temperature'
GARAGE_DHT22_HUMIDITY = 'garage/environment/humidity'
GARAGE_MOTION = 'garage/motion/status'

#GPIO Pin Assignments
DOOR_RELAY_PIN = 18
LIGHT_RELAY_PIN = 19

BUTTON_SENSOR_PIN = 17
DOOR_SENSOR_PIN = 6
DHT22_SENSOR_PIN = 27
PIR_SENSOR_PIN = 28

class Observer():
    _observers = []
    def __init__(self):
        self._observers.append(self)
        self._observables = {}
    def observe(self, event_name, callback):
        self._observables[event_name] = callback

class Event():
    def __init__(self, name, data, autofire = True):
        self.name = name
        self.data = data
        if autofire:
            self.fire()
    def fire(self):
        for observer in Observer._observers:
            if self.name in observer._observables:
                observer._observables[self.name](self.data)

class Relay(Observer):
    def __init__(self, mqtt_client, topic, relayPin, allowedActions = ['ON','OFF','TOGGLE']):
        Observer.__init__(self)
        self.client = mqtt_client
        self.topic = topic
        self.allowedActions = allowedActions
        self.relay = machine.Pin(relayPin, machine.Pin.OUT) # Setup the relay GPIO pin as output (relay ON when pin is HIGH, OFF when pin is LOW)
        self.relay.value(0) # Set relay initially OFF (0)
    
    def action(self, command):
        if command not in self.allowedActions:
            return

        if command == 'ON':
            self.__on()
        elif command == 'OFF':
            self.__off()
        elif command == 'TOGGLE':
            self.__toggle()
        elif command == 'PRESS':
            self.__press()
        else:
            print('Unknown command for relay:', command)

    def __on(self):
        print('Turning relay ON')
        self.relay.value(1)
    
    def __off(self):
        print('Turning relay OFF')
        self.relay.value(0)

    def __toggle(self):
        print('Toggle relay')
        self.relay.toggle()

    def __press(self):
        self.relay.value(1)
        time.sleep(0.2)
        self.relay.value(0)

class DHT22Monitor:
    def __init__(self, mqtt_client):
        self.client = mqtt_client
        self.sensor = dht.DHT22(machine.Pin(DHT22_SENSOR_PIN))
        self.temperature = int(0)
        self.humidity = int(0)
        self.pollsSinceLastTemperaturePublishCount = 0
        self.pollsSinceLastHumidityPublishCount = 0
        self.heartBeatLimit = (HEARTBEAT_MINUTES * 60) / MONITOR_DHT22_POLLING_SECONDS

    async def monitor_status(self):
        while True:
            try:
                #print('Checking DHT22')
                self.sensor.measure()
                currentTemperature = round(self.sensor.temperature(),1)
                currentHumidity = round(self.sensor.humidity(),1)

                self.pollsSinceLastTemperaturePublishCount += 1
                self.pollsSinceLastHumidityPublishCount +=1

                if self.temperature != currentTemperature or self.pollsSinceLastTemperaturePublishCount >= self.heartBeatLimit:
                    self.update_temperature(currentTemperature)

                if self.humidity != currentHumidity or self.pollsSinceLastHumidityPublishCount >= self.heartBeatLimit:
                    self.update_humidity(currentHumidity)

                del currentTemperature, currentHumidity

                await asyncio.sleep(MONITOR_DHT22_POLLING_SECONDS)

            except OSError as e:
                print('Failed to read DHT22 sensor.')

    def update_temperature(self, temperature):
        self.temperature = temperature
        print('DHT22 Temperature change to: ' + str(self.temperature))
        publish_message(self.client,GARAGE_DHT22_TEMPERATURE, str(self.temperature))
        self.pollsSinceLastTemperaturePublishCount = 0

    def update_humidity(self, humidity):
        self.humidity = humidity
        print('DHT22 Humidity change to: ' + str(self.humidity))
        publish_message(self.client, GARAGE_DHT22_HUMIDITY, str(self.humidity))
        self.pollsSinceLastHumidityPublishCount = 0

class PIRMonitor:
    def __init__(self, mqtt_client):
        self.client = mqtt_client
        self.sensor = machine.Pin(PIR_SENSOR_PIN, machine.Pin.IN)
        self.motion = ''
        self.pollsSinceLastMotionPublishCount = 0
        self.heartBeatLimit = (HEARTBEAT_MINUTES * 60) / MONITOR_MOTION_POLLING_SECONDS

    async def monitor_status(self):
        while True:
            try:
                #print('Checking Motion')
                currentMotion = 'MOTION DETECTED' if self.sensor.value() else 'NO MOTION'

                self.pollsSinceLastMotionPublishCount += 1

                if self.motion != currentMotion or self.pollsSinceLastMotionPublishCount >= self.heartBeatLimit:
                   self.update_motion(currentMotion)

                del currentMotion

                await asyncio.sleep(MONITOR_MOTION_POLLING_SECONDS)

            except:
                print('Failed to read PIR sensor.')

    def update_motion(self, motion):
        self.motion = motion
        print('Motion changed to: ' + self.motion)
        publish_message(self.client, GARAGE_MOTION, str(self.motion))
        self.pollsSinceLastMotionPublishCount = 0

class DoorPositionMonitor:
    def __init__(self, mqtt_client):
        self.client = mqtt_client
        self.sensor = machine.Pin(DOOR_SENSOR_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self.position = 'CLOSED'
        self.pollsSinceLastPositionPublishCount = 0
        self.heartBeatLimit = (HEARTBEAT_MINUTES * 60) / MONITOR_DOOR_POLLING_SECONDS

    async def monitor_status(self):
        while True:
            try:
                #print('Checking Door Position')
                currentPosition = 'OPEN' if self.sensor.value() == 1 else 'CLOSED'

                self.pollsSinceLastPositionPublishCount += 1

                if self.position != currentPosition or self.pollsSinceLastPositionPublishCount >= self.heartBeatLimit:
                   self.update_position(currentPosition)

                del currentPosition

                await asyncio.sleep(MONITOR_DOOR_POLLING_SECONDS)

            except:
                print('Failed to read Door Position sensor.')

    def update_position(self, position):
        self.position = position
        print('Position changed to: ' + self.position)
        publish_message(self.client, GARAGE_DOOR_STATUS, str(self.position))
        self.pollsSinceLastPositionPublishCount = 0

class ButtonPressMonitor:
    def __init__(self):
        self.sensor = machine.Pin(BUTTON_SENSOR_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self.pressed = False

    async def monitor_status(self):
        while True:
            try:
                # Check if the button is pressed (LOW signal on the GPIO pin)
                if self.sensor.value() == 0:  # Button is pressed (active low)
                    print('Button pressed!')
                    # Debounce the button press (wait for release)
                    while self.sensor.value() == 0:
                        await asyncio.sleep(0.1)
                    
                    Event(GARAGE_DOOR_ACTIVATE, 'PRESS')

                await asyncio.sleep(0.1)

            except:
                print('Failed to read button sensor.')

# Connect to Wi-Fi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print('Connecting to WiFi...')
        wlan.connect(SSID, PASSWORD)
        i = 0
        while not wlan.isconnected():
            time.sleep(1)
            i += 1
            print('connection wait: ' + str(i))
            if i > 10:
                print('Could not connect to Wifi.')
                sys.exit()

        print('WiFi connected')
        print('IP address:', wlan.ifconfig()[0])

# Callback function for handling incoming messages
def mqtt_callback(topic, msg):
    print('Received message on topic:', topic.decode())
    print('Message:', msg.decode())
    
    Event(topic.decode(), msg.decode())
            
# Connect to MQTT Broker
def connect_mqtt():
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
    client.set_callback(mqtt_callback)
    client.connect()
    print('Connected to MQTT broker:', MQTT_BROKER)
    return client

# Publish a message to MQTT topic
def publish_message(client, topic, message):
    print('Publishing message to topic:', topic)
    client.publish(topic, message)

# Main logic to run the MQTT client
async def main():

    # Connect to Wi-Fi
    if secrets.SSID != '' and secrets.WIFI_PASSWORD != '':
        connect_wifi()
    else:
        print('No Wifi details in secrets file. Exiting.')
        sys.exit()

    # Connect to MQTT broker
    client = connect_mqtt()

    # Subscribe to necessary topics
    print('Subscribing to topics:')
    client.subscribe(GARAGE_DOOR_ACTIVATE)
    print(GARAGE_DOOR_ACTIVATE)
    client.subscribe(GARAGE_LIGHT_ACTIVATE)
    print(GARAGE_LIGHT_ACTIVATE)

    # Initialise relays
    doorRelay = Relay(client, GARAGE_DOOR_ACTIVATE, DOOR_RELAY_PIN, ['PRESS'])
    doorRelay.observe(GARAGE_DOOR_ACTIVATE, doorRelay.action)
    lightRelay = Relay(client, GARAGE_LIGHT_ACTIVATE, LIGHT_RELAY_PIN)
    lightRelay.observe(GARAGE_LIGHT_ACTIVATE, lightRelay.action)

    if MONITOR_DHT22:
        # Initialise DHT22Monitor object
        enviro = DHT22Monitor(client)
        asyncio.create_task(enviro.monitor_status())

    if MONITOR_MOTION:
        # Initialise PIRMonitor object
        motion = PIRMonitor(client)
        asyncio.create_task(motion.monitor_status())

    if MONITOR_DOOR_POSITION:
        # Initialise DoorPisitionMonitor object
        door = DoorPositionMonitor(client)
        asyncio.create_task(door.monitor_status())

    if MONITOR_BUTTON_PRESS:
        # Initialise ButtonPressMonitor object
        button = ButtonPressMonitor()
        asyncio.create_task(button.monitor_status())
    try:
        while True:
            # Check for any incoming MQTT messages
            client.check_msg()
            
            # Small delay to prevent overloading the loop
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print('Disconnecting...')
        client.disconnect()

if __name__ == '__main__':
    asyncio.run(main())
