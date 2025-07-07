# air_quality_monitor
温湿度計とダストセンサーを使い、空気の状態を表示するシステム
\```python
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
\```
