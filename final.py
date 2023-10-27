import time
from paho.mqtt import client as mqtt_client
import base64
from picamera import PiCamera

broker = 'broker.hivemq.com'
port = 1883
imageTopic = "chitkara-university/school-bus-monitoring-system/image"
humTopic = "chitkara-university/school-bus-monitoring-system/hum"
tempTopic = "chitkara-university/school-bus-monitoring-system/temp"
mpuTopic = "chitkara-university/school-bus-monitoring-system/mpu"
client_id = 'RaspberryPi'

camera = PiCamera()
camera.start_preview()
time.sleep(2)
camera.brightness = 60

import adafruit_dht
import board
import smbus

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)
	
def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

dht_device = adafruit_dht.DHT22(board.D17)

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Successfully connected to MQTT broker")
        else:
            print("Failed to connect, return code %d", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client
    
def publish(client):

    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
	
    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
	
    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
	
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    
    string = str(Ax) + "," + str(Ay) + "," + str(Az) + "," + str(Gx) + "," + str(Gy) + "," + str(Gz)
    
    client.publish(mpuTopic, string, 0)

    temperature = dht_device.temperature
    humidity = dht_device.humidity

    client.publish(humTopic, humidity, 0)
    client.publish(tempTopic, temperature, 0)

    camera.capture('./image.jpg')
    with open("./image.jpg",'rb') as file:
        filecontent = file.read()
        base64_bytes = base64.b64encode(filecontent)
        base64_message = base64_bytes.decode('ascii')
        result = client.publish(imageTopic,base64_message,0)
    msg_status = result[0]
    if msg_status == 0:
        print(f"message sent to topic {imageTopic}")
    else:
        print(f"Failed to send message to topic {imageTopic}")

def main():
    client = connect_mqtt()
    client.loop_start()

    while True:
        publish(client)
        time.sleep(5)
    
    client.loop_stop()


if __name__ == '__main__':
    main()
