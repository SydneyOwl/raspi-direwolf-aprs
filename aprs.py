import gpsd
import threading
import serial
import time
import math
from INA219 import INA219
import difflib
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import datetime
import RPi.GPIO as GPIO
import os


class UARTTool:
    def __init__(self, route):
        self._lockLock = threading.RLock()
        self.isOnline = True
        try:
            self.ser = serial.Serial(route, 115200, timeout=1)
        except Exception:
            self.isOnline = False
        self.common_ui()
        self.image_size = {
            "!": (2097152, 71, 57),
            "#": (2105246, 80, 76),
            "$": (2117406, 80, 76),
            "%": (2129566, 71, 57),
            "&": (2137660, 80, 76),
            "'": (2149820, 116, 95),
            "(": (2171860, 71, 57),
            "*": (2179954, 80, 76),
            "+": (2192114, 80, 76),
            ",": (2204274, 85, 76),
            "-": (2217194, 98, 76),
            # ".": (2232090,),
            # "/": (2426650,),
            "0": (2507590, 80, 76),
            "1": (2519750, 80, 76),
            "2": (2531910, 80, 76),
            "3": (2544070, 80, 76),
            "4": (2556230, 80, 76),
            "5": (2568390, 80, 76),
            "6": (2580550, 80, 76),
            "7": (2592710, 80, 76),
            "8": (2604870, 80, 76),
            "9": (2617030, 80, 76),
            ":": (2629190, 75, 57),
            ";": (2637740, 80, 76),
            "<": (2649900, 80, 76),
            "=": (2662060, 80, 76),
            ">": (2674220, 64, 57),
            "?": (2681516, 80, 76),
            "@": (2693676, 80, 76),
            "A": (2705836, 80, 76),
            "B": (2717996, 91, 76),
            "C": (2731828, 80, 76),
            "G": (2743988, 80, 76),
            "H": (2756148, 80, 76),
            "I": (2768308, 75, 57),
            "K": (2776858, 107, 95),
            "M": (2797188, 98, 76),
            "N": (2812084, 80, 76),
            "P": (2824244, 64, 57),
            "R": (2831540, 64, 57),
            "U": (2838836, 64, 57),
            "W": (2846132, 80, 76),
            "X": (2858292, 80, 76),
            "Y": (2870452, 80, 76),
            "Z": (2882612, 75, 57),
            "[": (2891162, 116, 95),
            "]": (2913202, 80, 76),
            "^": (2925362, 80, 76),
            "_": (2937522, 80, 76),
            "`": (2949682, 80, 76),
            "a": (2961842, 80, 76),
            "b": (2974002, 80, 76),
            "d": (2986162, 80, 76),
            "e": (2998322, 71, 57),
            "f": (3006416, 71, 57),
            "g": (3014510, 80, 76),
            "h": (3026670, 80, 76),
            "i": (3038830, 67, 57),
            "j": (3046468, 54, 57),
            "k": (3053764, 80, 76),
            "m": (3065924, 80, 76),
            # "n": (3078084,),
            "o": (3140670, 80, 76),
            "p": (3052830, 80, 76),
            "q": (3164990, 80, 76),
            "r": (3177150, 80, 76),
            "s": (3189310, 80, 76),
            "t": (3201470, 80, 76),
            "u": (3213630, 64, 57),
            "v": (3220926, 71, 57),
            "x": (3229020, 71, 57),
            "y": (3237114, 80, 76),
            # "z": (3250034,),
            "~": (3444594, 80, 76),
        }

    def common_ui(self):
        self.writeToSerial("CLR(15);BL(200);")
        self.writeToSerial(
            "PL(0,25,480,25,0);DCV16(5,3,'???A ???V',1);DCV16(440,3,'??%',21);"
        )

    def calcLineHeading(self, deg, speed=200):
        fixedPointX = 370
        fixedPointY = 200
        radius = 80 * (speed / 200)
        xiangxian = math.floor(deg / 90)
        leftDeg = deg % 90
        mX = 0
        mY = 0

        sinDeg = math.sin(math.radians(leftDeg))
        cosDeg = math.cos(math.radians(leftDeg))
        mX = radius * sinDeg
        mY = radius * cosDeg
        if xiangxian == 0:
            mY = -mY
        if xiangxian == 1:
            mX, mY = mY, mX
        if xiangxian == 2:
            mX = -mX
        if xiangxian == 3:
            mX = -mX
            mY = -mY
        return (fixedPointX + mX, fixedPointY + mY)

    def calcAPSDHeading(self, deg, distance=50):
        fixedPointX = 390
        fixedPointY = 220
        radius = 70 * (distance / 50)
        if distance > 70:
            radius = 70
        xiangxian = math.floor(deg / 90)
        leftDeg = deg % 90
        mX = 0
        mY = 0

        sinDeg = math.sin(math.radians(leftDeg))
        cosDeg = math.cos(math.radians(leftDeg))
        mX = radius * sinDeg
        mY = radius * cosDeg
        if xiangxian == 0:
            mY = -mY
        if xiangxian == 1:
            mX, mY = mY, mX
        if xiangxian == 2:
            mX = -mX
        if xiangxian == 3:
            mX = -mX
            mY = -mY
        return (fixedPointX + mX, fixedPointY + mY)

    def calcDistance(self, latA, lonA, latB, lonB):
        ra = 6378140  # 赤道半径
        rb = 6356755  # 极半径
        flatten = (ra - rb) / ra  # Partial rate of the earth
        # change angle to radians
        radLatA = math.radians(latA)
        radLonA = math.radians(lonA)
        radLatB = math.radians(latB)
        radLonB = math.radians(lonB)

        pA = math.atan(rb / ra * math.tan(radLatA))
        pB = math.atan(rb / ra * math.tan(radLatB))
        x = math.acos(
            math.sin(pA) * math.sin(pB)
            + math.cos(pA) * math.cos(pB) * math.cos(radLonA - radLonB)
        )
        c1 = (
            (math.sin(x) - x)
            * (math.sin(pA) + math.sin(pB)) ** 2
            / math.cos(x / 2) ** 2
        )
        c2 = (
            (math.sin(x) + x)
            * (math.sin(pA) - math.sin(pB)) ** 2
            / math.sin(x / 2) ** 2
        )
        dr = flatten / 8 * (c1 - c2)
        distance = ra * (x + dr)
        distance = round(distance / 1000, 4)
        return distance

    def calcDegree(self, latA, lonA, latB, lonB):
        radLatA = math.radians(latA)
        radLonA = math.radians(lonA)
        radLatB = math.radians(latB)
        radLonB = math.radians(lonB)
        dLon = radLonB - radLonA
        y = math.sin(dLon) * math.cos(radLatB)
        x = math.cos(radLatA) * math.sin(radLatB) - math.sin(radLatA) * math.cos(
            radLatB
        ) * math.cos(dLon)
        brng = math.degrees(math.atan2(y, x))
        brng = round((brng + 360) % 360, 4)
        return brng

    def calcDegBin(self, degrees, direction):
        degrees_abs = abs(degrees)
        minutes = int((degrees_abs % 1) * 60)
        seconds = round((minutes % 1) * 60, 2)

        # 补零处理
        if minutes < 10:
            minutes_str = f"0{minutes}"
        else:
            minutes_str = str(minutes)

        if seconds < 10:
            seconds_str = f"0{seconds}"
        else:
            seconds_str = f"{seconds}"

        return f"{int(degrees_abs)}{minutes_str}.{seconds_str}{direction}"

    def writeToSerial(self, *args):
        if self.isOnline:
            self._lockLock.acquire()
            try:
                for i in args:
                    i = i + "\r\n"
                    i = i.encode("gb2312")
                    self.ser.write(i)
            finally:
                time.sleep(0.2)
                self._lockLock.release()

    def writeToSerialAPRS(self, inst):
        # if curr_page != 0:
        #     return
        self.writeToSerial(inst)

    def writeToSerialGPS(self, inst):
        # if curr_page != 1:
        # return
        self.writeToSerial(inst)


def parseAPRS(data: str):
    # 0       1    2        3    4      5     6     7   8     9       10         11     12   13       14      15        16   17   18      19     20         21
    "chan,utime,isotime,source,heard,level,error,dti,name,symbol,latitude,longitude,speed,course,altitude,frequency,offset,tone,system,status,telemetry,comment"
    raw = data.split(",")
    recvTime = raw[1]
    timeArray = time.localtime(int(recvTime))
    otherStyleTime = time.strftime("%Y-%m-%d %H:%M:%S", timeArray)
    route = raw[4]
    soundLevel = raw[5]
    source = raw[3]
    name = raw[8]
    symbol = raw[9][0]
    if len(raw[9]) > 1:
        symbol = raw[9][1]
    lat = raw[10]
    lon = raw[11]
    speed = raw[12]
    course = raw[13]
    alt = raw[14]
    system = raw[18]
    comment = raw[21]
    return {
        "otherStyleTime": otherStyleTime,
        "route": route,
        "recvTime": otherStyleTime,
        "soundLevel": soundLevel,
        "source": source,
        "name": name,
        "symbol": symbol,
        "lat": lat,
        "lon": lon,
        "speed": speed,
        "course": course,
        "alt": alt,
        "system": system,
        "comment": comment,
    }


class MyHandler(FileSystemEventHandler):
    def __init__(self, filename):
        super().__init__()
        self.filename = filename
        self.previous_lines = self.read_file()

    def read_file(self):
        with open(self.filename, "r") as file:
            return file.readlines()

    def on_modified(self, event):
        global remain
        if event.is_directory:
            return

        current_lines = self.read_file()

        diff = difflib.unified_diff(self.previous_lines, current_lines, lineterm="")
        changes = [
            line
            for line in diff
            if (line.startswith("+") or line.startswith("-"))
            and not (line.startswith("+++") or line.startswith("---"))
        ]
        GPIO.output(beeper, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(beeper, GPIO.LOW)
        time.sleep(0.2)
        GPIO.output(beeper, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(beeper, GPIO.LOW)
        aprsdata = parseAPRS(changes[0].strip().lstrip("+"))
        if curr_page == 0:
            updateAPRS(aprsdata)
        else:
            remain = aprsdata

        self.previous_lines = current_lines


def getBatt():
    bus_voltage = ina219.getBusVoltage_V()  # voltage on V- (load side)
    current = ina219.getCurrent_mA()  # current in mA
    power = ina219.getPower_W()  # power in W
    p = (bus_voltage - 3) / 1.2 * 100
    if p > 100:
        p = 100
    if p < 0:
        p = 0
    # A V B W
    return (
        "{:6.3f}".format(current / 1000),
        "{:6.3f}".format(bus_voltage),
        # "{:6.3f}".format(power),
        "{:3.1f}".format(p),
    )


def updateGPS():
    while True:
        if curr_page == 1:
            try:
                lock.acquire()
                packet = gpsd.get_current()
                mode = packet.mode
                sats = packet.sats
                lat = "未知"
                lon = "未知"
                precision = (0, 0, -1)
                speed = 0
                alt = 0
                vet = 0
                deg = 0
                time_local = "未知"
                if mode == 2 or mode == 3:
                    precision = packet.position_precision()
                    speed = packet.speed() * 3.6
                    lat = packet.lat
                    lon = packet.lon
                    time_local = packet.time
                if mode == 3:
                    alt = packet.alt
                    vet = packet.speed_vertical() * 3.6
                    deg = packet.track

                gps_stat = ""
                if mode == 0 or mode == 1:
                    gps_stat = "未定位"
                if mode == 2:
                    gps_stat = "2D定位"
                if mode == 3:
                    gps_stat = "3D定位"
                uu.writeToSerialGPS("DCV16(200,3,'位置信息',0);")
                uu.writeToSerialGPS(
                    "DCV16(5,30,'GPS: {}',0);DCV16(5,50,'卫星数:{}',0);".format(
                        gps_stat, sats
                    )
                )
                uu.writeToSerialGPS("DCV16(5,220,'UTC:{}')".format(time_local))
                uu.writeToSerialGPS(
                    "DCV24(5,70,'经度:{}[误差:{}m]',0);DCV24(5,100,'纬度:{}[误差:{}m]',0);DCV24(5,130,'速度:{}km/h',0);".format(
                        lon, precision[0], lat, precision[1], speed
                    )
                )
                uu.writeToSerialGPS(
                    "DCV24(5,160,'海拔:{}m',0);DCV24(5,190,'垂直速度:{}km/h',0);".format(
                        alt, vet
                    )
                )
                uu.writeToSerialGPS(
                    "CIR(370,200,80,1);CIR(370,200,16,1);CIR(370,200,32,1);CIR(370,200,48,1);CIR(370,200,64,1);PL(370,120,370,280,0);PL(290,200,450,200,0);DCV16(366,100,'N',0);DCV16(280,190,'W',0);DCV16(366,287,'S',0);DCV16(454,190,'E',0);"
                )

                x, y = uu.calcLineHeading(deg, float(speed))
                uu.writeToSerialGPS("PL(370,200,{},{})".format(x, y))
                uu.writeToSerialGPS("CIRF({},{},3,0);".format(x, y))
            except Exception:
                pass
            finally:
                lock.release()
        time.sleep(1)


def checkEmpty(chk: str):
    return "未知" if chk == "" else chk


def updateAPRS(aprs_frame: dict):
    if curr_page != 0:
        return
    lock.acquire(True)
    try:
        uu.writeToSerialAPRS("CLR(15);PL(0,25,480,25,0);DCV16(200,3,'信标信息',0);")
        if aprs_frame == {}:
            uu.writeToSerialAPRS("DCV32(200,200,'等待信标',0);")
            return
        packet = gpsd.get_current()
        mode = packet.mode
        mylat = 0
        mylon = 0
        if mode >= 2:
            mylat = packet.lat
            mylon = packet.lon
            dist = uu.calcDistance(
                float(mylat),
                float(mylon),
                float(aprs_frame["lat"]),
                float(aprs_frame["lon"]),
            )
            deg = uu.calcDegree(
                float(mylat),
                float(mylon),
                float(aprs_frame["lat"]),
                float(aprs_frame["lon"]),
            )
            x, y = uu.calcAPSDHeading(deg, dist)
            # draw a circle
            uu.writeToSerialAPRS(
                "CIR(390,220,70,0);CIR(390,220,14,0);CIR(390,220,28,0);CIR(390,220,42,0);CIR(390,220,56,0);DCV16(390,140,'N',0);DCV16(290,200,'50km',0);"
            )
            uu.writeToSerialAPRS("PL(390,220,{},{})".format(x, y))
            uu.writeToSerialAPRS("CIRF({},{},3,0);".format(x, y))

        # --------------------
        # mylat = 24.91
        # mylon = 118.192
        # dist = uu.calcDistance(float(mylat),float(mylon),float(aprs_frame["lat"]),float(aprs_frame["lon"]))
        # deg = uu.calcDegree(float(mylat),float(mylon),float(aprs_frame["lat"]),float(aprs_frame["lon"]))
        # x,y = uu.calcAPSDHeading(deg,dist)
        # # draw a circle
        # uu.writeToSerial(
        #     "CIR(390,220,70,0);CIR(390,220,14,0);CIR(390,220,28,0);CIR(390,220,42,0);CIR(390,220,56,0);DCV16(390,140,'N',0);DCV16(290,200,'50km',0);"
        # )
        # uu.writeToSerial("PL(390,220,{},{})".format(x, y))
        # uu.writeToSerial("CIRF({},{},3,0);".format(x, y))
        ###########

        if aprs_frame["symbol"] in uu.image_size:
            addr = uu.image_size[aprs_frame["symbol"]][0]
            wid = uu.image_size[aprs_frame["symbol"]][1]
            height = uu.image_size[aprs_frame["symbol"]][2]
            uu.writeToSerialAPRS("FSIMG({},80,50,{},{},0);".format(addr, wid, height))
        uu.writeToSerialAPRS(
            "DCV32(180,60,'{}',0);DCV16(340,45,'VL:{}',0);DCV16(340,65,'name:{}',0);DCV16(340,85,'track:{}',0);".format(
                checkEmpty(aprs_frame["source"]),
                checkEmpty(aprs_frame["soundLevel"]),
                checkEmpty(aprs_frame["name"]),
                checkEmpty(aprs_frame["route"]),
            )
        )
        uu.writeToSerialAPRS(
            "DCV24(40,120,'接收时间：{}',0);".format(aprs_frame["otherStyleTime"])
        )
        uu.writeToSerialAPRS(
            "DCV24(40,150,'经度：{}',0);".format(checkEmpty(aprs_frame["lon"]))
        )
        uu.writeToSerialAPRS(
            "DCV24(40,180,'纬度：{}',0);".format(checkEmpty(aprs_frame["lat"]))
        )
        uu.writeToSerialAPRS(
            "DCV24(40,210,'速度：{}',0);".format(checkEmpty(aprs_frame["speed"]))
        )
        uu.writeToSerialAPRS(
            "DCV24(40,240,'方位角：{}',0);".format(checkEmpty(aprs_frame["course"]))
        )
        uu.writeToSerialAPRS(
            "DCV24(40,270,'海拔：{}',0);".format(checkEmpty(aprs_frame["alt"]))
        )
        uu.writeToSerialAPRS(
            "DCV16(0,300,'注释<cmt>:{}',0);".format(aprs_frame["comment"])
        )
    except Exception as e:
        print(repr(e))
    finally:
        lock.release()


def updateBattery():
    while True:
        try:
            a, v, b = getBatt()
            uu.writeToSerial(
                "PL(0,25,480,25,0);DCV16(5,3,'{}A {}V',1);DCV16(440,3,'{}%',21);".format(
                    a, v, b
                )
            )
            if int(b) < 8:
                pwrCallback("")
        except Exception:
            pass
        time.sleep(5)


def pgSwitchCallback(key):
    global curr_page, remain
    uu.writeToSerial("CLR(15);")
    if curr_page == 0:
        curr_page = 1
    else:
        curr_page = 0
        updateAPRS(remain)
        remain = {}


def aprsTxCallback(key):
    packet = gpsd.get_current()
    mode = packet.mode
    mylat = 0
    mylon = 0
    if mode >= 2:
        uu.writeToSerial("DCV16(380,2,'tx>>>',1);")
        mylat = packet.lat
        mylon = packet.lon
        lat_direction = "N" if mylat >= 0 else "S"
        lon_direction = "E" if mylon >= 0 else "W"
        latitude_dms = uu.calcDegBin(mylat, lat_direction)
        longitude_dms = uu.calcDegBin(mylon, lon_direction)
        with open("/root/kissdata/tx/tmp", "w", encoding="utf-8") as f:
            sender = "BG5VLI-7>APDW17:!{}/{}>360/000144.640MHz /A=000233[DIREWOLF](SHX8800+PI0 manual trigger)".format(
                latitude_dms, longitude_dms
            )
            print(sender)
            f.write(sender)
            f.close()
        time.sleep(2)
        uu.writeToSerial("DCV16(380,2,'ok<><',1);")
    else:
        uu.writeToSerial("DCV16(380,2,'noloc',1);")
        time.sleep(2)
        uu.writeToSerial("DCV16(380,2,'ok<><',1);")


def pwrCallback(key):
    GPIO.output(beeper, GPIO.HIGH)
    start_time = time.time()
    counter = 0
    while GPIO.input(pwr) == 0: # Wait for the button up
        counter+=1
        time.sleep(0.05)
        if counter>40:
            GPIO.output(beeper, GPIO.LOW)
    GPIO.output(beeper, GPIO.LOW)
    buttonTime = time.time() - start_time    # How long was the button down?   # 1= brief push
    if buttonTime >= 4:
        lock.acquire(True)
        uu.writeToSerial("DC48(30,46,'关机',1,0);")
        time.sleep(3)
        os.popen("poweroff")


lock = threading.Lock()

ina219 = INA219(addr=0x43)
uu = UARTTool("/dev/ttyUSB0")
gpsd.connect()

GPIO.setmode(GPIO.BOARD)
beeper = 15
pgSwitch = 33
aprsTx = 35
pwr = 37

path = (
    "/root/singleLog"
)  # Change this to your specific file name
event_handler = MyHandler(path)
observer = Observer()
observer.schedule(event_handler, path, recursive=False)
observer.start()

remain = {}

GPIO.setup(beeper, GPIO.OUT)
GPIO.output(beeper, GPIO.LOW)

GPIO.setup(pgSwitch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(pgSwitch, GPIO.FALLING, callback=pgSwitchCallback, bouncetime=200)

GPIO.setup(aprsTx, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(aprsTx, GPIO.FALLING, callback=aprsTxCallback, bouncetime=200)

GPIO.setup(pwr, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(pwr, GPIO.FALLING, callback=pwrCallback, bouncetime=30)

# 0->aprs 1->gps
curr_page = 0

# start batt mon
updateAPRS({})
threading.Thread(target=updateBattery).start()
threading.Thread(target=updateGPS).start()

try:
    while True:
        time.sleep(0.05)
except Exception:
    observer.stop()
finally:
    GPIO.cleanup()
observer.join()
