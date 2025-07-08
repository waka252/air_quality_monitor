# air_quality_monitor
温湿度計とダストセンサーを使い、空気の状態を表示するシステム
```python
import RPi.GPIO as GPIO
import dht11
from time import sleep
from time import gmtime, strftime
import datetime
import smbus
import time

# --- 音を鳴らすための設定 ---
BUZZER_PIN = 17  # 圧電スピーカー GPIO17
DUST_SENSOR_PIN = 18  # ダストセンサー GPIO18 (BCM)

# GPIO初期化
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()

# 圧電スピーカー
GPIO.setup(BUZZER_PIN, GPIO.OUT)
# ダストセンサー
GPIO.setup(DUST_SENSOR_PIN, GPIO.IN)

# DHT11
instance = dht11.DHT11(pin=4)

# LCD設定
I2C_ADDR = 0x27
LCD_WIDTH = 16

LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100

E_PULSE = 0.0005
E_DELAY = 0.0005

bus = smbus.SMBus(1)

def lcd_init():
    lcd_byte(0x33,LCD_CMD)
    lcd_byte(0x32,LCD_CMD)
    lcd_byte(0x06,LCD_CMD)
    lcd_byte(0x0C,LCD_CMD)
    lcd_byte(0x28,LCD_CMD)
    lcd_byte(0x01,LCD_CMD)
    sleep(E_DELAY)

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    sleep(E_PULSE)
    bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
    sleep(E_DELAY)

def lcd_string(message,line):
    message = message.ljust(LCD_WIDTH," ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]),LCD_CHR)

def play_beep(duration):
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    sleep(duration)
    GPIO.output(BUZZER_PIN, GPIO.LOW)

# --- ダストセンサー読み取り関数 ---
def read_dust_sensor(duration_sec):
    """
    ダストセンサーのLowパルス時間を測定して濃度を計算
    duration_sec 秒間測定して平均を出す
    """
    start_time = time.time()
    low_pulse_occupancy = 0

    while (time.time() - start_time) < duration_sec:
        while GPIO.input(DUST_SENSOR_PIN) == 0:
            pulse_start = time.time()
            while GPIO.input(DUST_SENSOR_PIN) == 0:
                pass
            pulse_end = time.time()
            low_pulse_occupancy += (pulse_end - pulse_start)

    ratio = low_pulse_occupancy / duration_sec * 100
    concentration = 1.1 * (ratio ** 3) - 3.8 * (ratio ** 2) + 520 * ratio + 0.62
    return round(concentration, 2)  # µg/m3

# --- メイン ---
def main():
    lcd_init()
    while True:
        count = 0
        result = instance.read()
        if result.is_valid():
            play_beep(0.1)

            lcd_string(strftime("%Y.%m.%d (%a)", gmtime()), LCD_LINE_1)
            while count < 10:
                local_time = datetime.datetime.now()
                lcd_string(local_time.strftime("%H:%M:%S"), LCD_LINE_2)
                sleep(1)
                count += 1
            temp =result.temperature
            humi =result.humidity
            
            if ((27 <= temp <=28) or (60 <= humi <=70)):
                play_beep(0.1)
                sleep(0.1)
                play_beep(0.1)
            elif(temp <=12 or temp >28 or 29 > humi or humi > 71):
                for _ in range(3):
                    play_beep(0.3)
                    sleep(0.1)
            temp_str = "Temperature:" + str(result.temperature) + "C"
            humi_str= "Humidity:" + str(result.humidity) + "%"
                    
            lcd_string(temp_str, LCD_LINE_1)
            lcd_string(humi_str, LCD_LINE_2)
            dust = read_dust_sensor(15)

            lcd_string("Dust Conc.:", LCD_LINE_1)
            lcd_string(str(dust) + " ug/m3", LCD_LINE_2)
            if (3000 <= dust <=5000):
                play_beep(0.3)
                sleep(0.1)
                play_beep(0.3)
            elif(dust > 5000):
                for _ in range(10):
                    play_beep(0.2)
                    sleep(0.1)
            sleep(10)

        else:
            lcd_string("Sensor Error!", LCD_LINE_1)
            lcd_string("Retry...", LCD_LINE_2)
            sleep(1)

try:
    print('Start:' + str(datetime.datetime.now()))
    main()
except KeyboardInterrupt:
    print('End:' + str(datetime.datetime.now()))
    GPIO.cleanup()
```
dht11.py
```python
import time
import RPi.GPIO as GPIO

class DHT11Result:
 'DHT11 sensor result returned by DHT11.read() method'

 ERR_NO_ERROR = 0
 ERR_MISSING_DATA = 1
 ERR_CRC = 2

 error_code = ERR_NO_ERROR
 temperature = -1
 humidity = -1

 def __init__(self, error_code, temperature, humidity):
  self.error_code = error_code
  self.temperature = temperature
  self.humidity = humidity
  
 def is_valid(self):
  return self.error_code == DHT11Result.ERR_NO_ERROR

class DHT11:
 'DHT11 sensor reader class for Raspberry'
  
 __pin = 0

 def __init__(self, pin):
  self.__pin = pin
  
 def read(self):
  GPIO.setup(self.__pin, GPIO.OUT)

  # send initial high
  self.__send_and_sleep(GPIO.HIGH, 0.05)

  # pull down to low
  self.__send_and_sleep(GPIO.LOW, 0.02)

  # change to input using pull up
  GPIO.setup(self.__pin, GPIO.IN, GPIO.PUD_UP)
  
  # collect data into an array
  data = self.__collect_input()
  
  # parse lengths of all data pull up periods
  pull_up_lengths = self.__parse_data_pull_up_lengths(data)
  
  # if bit count mismatch, return error (4 byte data + 1 byte checksum)
  if len(pull_up_lengths) != 40:
   return DHT11Result(DHT11Result.ERR_MISSING_DATA, 0, 0)
  
  # calculate bits from lengths of the pull up periods
  bits = self.__calculate_bits(pull_up_lengths)
  
  # we have the bits, calculate bytes
  the_bytes = self.__bits_to_bytes(bits)
  
  # calculate checksum and check
  checksum = self.__calculate_checksum(the_bytes)
  if the_bytes[4] != checksum:
   return DHT11Result(DHT11Result.ERR_CRC, 0, 0)
   
  # ok, we have valid data, return it
  return DHT11Result(DHT11Result.ERR_NO_ERROR, the_bytes[2], the_bytes[0])

 def __send_and_sleep(self, output, sleep):
  GPIO.output(self.__pin, output)
  time.sleep(sleep)
  
 def __collect_input(self):

  # collect the data while unchanged found
  unchanged_count = 0
  
  # this is used to determine where is the end of the data
  max_unchanged_count = 100 

  last = -1
  data = []
  while True:
   current = GPIO.input(self.__pin)
   data.append(current)
   if last != current:
    unchanged_count = 0
    last = current
   else:
    unchanged_count += 1
    if unchanged_count > max_unchanged_count:
     break
  
  return data  

 def __parse_data_pull_up_lengths(self, data):

  STATE_INIT_PULL_DOWN = 1
  STATE_INIT_PULL_UP = 2
  STATE_DATA_FIRST_PULL_DOWN = 3
  STATE_DATA_PULL_UP = 4
  STATE_DATA_PULL_DOWN = 5

  state = STATE_INIT_PULL_DOWN

  lengths = [] # will contain the lengths of data pull up periods
  current_length = 0 # will contain the length of the previous period

  for i in range(len(data)):
   
   current = data[i]
   current_length += 1
    
   if state == STATE_INIT_PULL_DOWN:
    if current == GPIO.LOW:
     # ok, we got the initial pull down
     state = STATE_INIT_PULL_UP
     continue
    else:
     continue
   
   if state == STATE_INIT_PULL_UP:
    if current == GPIO.HIGH:
     # ok, we got the initial pull up
     state = STATE_DATA_FIRST_PULL_DOWN
     continue
    else:
     continue
   
   if state == STATE_DATA_FIRST_PULL_DOWN:
    if current == GPIO.LOW:
     # we have the initial pull down, the next will be the data pull up
     state = STATE_DATA_PULL_UP
     continue
    else:
     continue

   if state == STATE_DATA_PULL_UP:
    if current == GPIO.HIGH:
     # data pulled up, the length of this pull up will determine whether it is 0 or 1
     current_length = 0
     state = STATE_DATA_PULL_DOWN
     continue
    else:
     continue
   
   if state == STATE_DATA_PULL_DOWN:
    if current == GPIO.LOW:   
     # pulled down, we store the length of the previous pull up period
     lengths.append(current_length)
     state = STATE_DATA_PULL_UP    
     continue
    else:
     continue
     
  return lengths
  
 def __calculate_bits(self, pull_up_lengths):

  # find shortest and longest period
  shortest_pull_up = 1000
  longest_pull_up = 0
  
  for i in range(0, len(pull_up_lengths)):
    
   length = pull_up_lengths[i]
   if length < shortest_pull_up:
    shortest_pull_up = length
    
   if length > longest_pull_up:
    longest_pull_up = length
    
  # use the halfway to determine whether the period it is long or short
  halfway = shortest_pull_up + (longest_pull_up - shortest_pull_up) / 2
  
  bits = []
  
  for i in range(0, len(pull_up_lengths)):
   
   bit = False
   if pull_up_lengths[i] > halfway:
    bit = True
    
   bits.append(bit)
  
  return bits
 
 def __bits_to_bytes(self, bits):
  
  the_bytes = []
  byte = 0
  
  for i in range(0, len(bits)):
   
   byte = byte << 1
   if (bits[i]):
    byte = byte | 1
   else:
    byte = byte | 0
   
   if ((i + 1) % 8 == 0):
    the_bytes.append(byte)
    byte = 0
    
  return the_bytes
  
 def __calculate_checksum(self, the_bytes):
  return the_bytes[0] + the_bytes[1] + the_bytes[2] + the_bytes[3] & 255
```
