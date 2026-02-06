import subprocess
import re
import datetime
import time
import math
import serial
import yaml




class StepperController:
    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2) 

    def _wait_ok(self, timeout: float = 1.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line == 'ok':
                return True
        return False

    def move_motors(self, pos1: int, pos2: int, speed: float):
        cmd = f"{pos1} {pos2} {speed}\n"
        self.ser.write(cmd.encode())
        self.ser.flush()  

        if self._wait_ok():
            print(f" {cmd.strip()}")
        else:
            print("time out")

    def close(self):
        self.ser.close()

with open('cfg.yaml') as f:
    PORT = yaml.safe_load(f)['com_num']
motor_ctrl = StepperController(PORT)   


def parse_trial_data(data):
    pattern = (
        r"trial=(\d+)\s+"
        r"target=(-?\d+\.\d+)\s+"
        r"x=(-?\d+\.\d+)\s+"
        r"y=(-?\d+\.\d+)\s+"
        r"z=(-?\d+\.\d+)\s+"
        r"yaw=(-?\d+\.\d+)\s+"
        r"unix=(-?\d+\.\d+)"
    )
    match = re.search(pattern, data)
    
    if match:
        return int(match.group(1)),float(match.group(2)),float(match.group(3)),float(match.group(4)), float(match.group(5)), float(match.group(6)), float(match.group(7))
    else:
        return None



def parse_trial_end(data):
    pattern = (
        r"targetS=(-?\d+\.\d+)\s+"
        r"lastS=(-?\d+\.\d+)\s+"
        r"errS=(-?\d+\.\d+)\s+"
        r"rt=(-?\d+\.\d+)"
    )
    match = re.search(pattern, data)
    
    if match:
        return float(match.group(1)), float(match.group(2)), float(match.group(3)), float(match.group(4))
    else:
        return None
    


def save_data_to_file(data, filename):
    with open(filename, 'w') as f:
        f.write("tgt,chosen,err,rt\n")
        for row in data:
            f.write(f"{row[0]},{row[1]},{row[2]},{row[3]}\n")

def prop2motor(prop):
    if prop <= 0.01:
        return 0
    elif 0.01 < prop < 0.926291403611155:
        return int(733.71 * prop**4 - 592.29 * prop ** 3 + 101.01 * prop ** 2 + 370.68 * prop + 50.024)
    elif prop < 1:
        return int(154437 * prop ** 3 - 418628 * prop **2 +  379046 * prop - 114111)
    else:
        return int(750)

import math
from typing import Tuple

Vector2 = Tuple[float, float]

def _dot(u: Vector2, v: Vector2) -> float:
    return u[0] * v[0] + u[1] * v[1]

def _cross(u: Vector2, v: Vector2) -> float:
    return u[0] * v[1] - u[1] * v[0]

def _norm(u: Vector2) -> float:
    return math.hypot(u[0], u[1]) 


def signed_angle(u: Vector2, v: Vector2) -> float:
    nu, nv = _norm(u), _norm(v)
    if nu == 0 or nv == 0:
        raise ValueError("error")
    return math.atan2(_cross(u, v), _dot(u, v))


def lrcheck(aa):

    if aa >= 0:
        nl = 1 - aa/math.pi
        nr = 1 - (2*aa)/math.pi
        if nr < 0:
            nr = 0
    elif aa < 0:
        aa = -aa
        nr = 1 - aa/math.pi
        nl = 1 - (2*aa)/math.pi
        if nl < 0:
            nl = 0
    return nl, nr


adb_cmd = "adb logcat -v raw"

process = subprocess.Popen(
    adb_cmd.split(),
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    encoding='utf-8', 
    bufsize=1 
)

test_data = []
yaw = 0
target = 0
n = 0


try:
    for line in process.stdout:
        cleaned_line = line.strip()
        if cleaned_line:

            if "[Trial]" in cleaned_line and "yaw=" in cleaned_line:
                trial, target, x, y, z, yaw, ts = parse_trial_data(cleaned_line)
                yaw1 = math.radians(yaw)
                yaw2 = math.radians(target)
                relative_yaw = signed_angle((math.sin(yaw1), math.cos(yaw1)),(math.sin(yaw2), math.cos(yaw2)))
                ll, rr = lrcheck(relative_yaw)
                if n > 10:
                    motor_ctrl.move_motors(prop2motor(ll), prop2motor(rr), speed=800)

            if "[DistanceTrial] end" in cleaned_line:
                tgt, chosen, err, rt = parse_trial_end(cleaned_line)
                if tgt is not None:
                    test_data.append([tgt, chosen, err, rt])
                    print(f"target={tgt}, chosen={chosen}, error={err}, rt={rt}")
                    motor_ctrl.move_motors(0, 0, speed=800)
                    time.sleep(10)
                    
            if "[DistanceTrial] block finished" in cleaned_line:
                experiment_finished = True
                break

            n = n+1
            
except KeyboardInterrupt:
    print("\n end")
finally:
    process.terminate()
    motor_ctrl.move_motors(0, 0, speed=800)
    if test_data:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"angle_grad_{timestamp}.csv"
        save_data_to_file(test_data, filename)
    time.sleep(3)
    motor_ctrl.close()




