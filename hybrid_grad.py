import subprocess
import re
import os
import datetime
import time
import serial
import math
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
            print(f"{cmd.strip()}")
            
        else:
            print("timeout")

    def close(self):
        self.ser.close()



with open('cfg.yaml') as f:
    PORT = yaml.safe_load(f)['com_num']
motor_ctrl = StepperController(PORT)   



def parse_target_data(data):

    pattern = (
        r"trial=(\d+)\s+"
        r"index=(\d+)\s+"
        r"target_x=(-?\d+\.\d+)\s+"
        r"target_y=(-?\d+\.\d+)\s+"
        r"target_z=(-?\d+\.\d+)\s+"
        r"unix=(-?\d+\.\d+)"
    )
    match = re.search(pattern, data)
    
    if match:
        return (
            float(match.group(3)),  # target_x
            float(match.group(4)),  # target_y
            float(match.group(5)),  # target_z
        )
    else:
        print("未找到匹配的Target数据")
        return None
    

POSE_RE = re.compile(
    r"""\[pose\]\s+                # [Pose] / [pose]
        trial=(\d+)\s+
        x=([+-]?\d+(?:\.\d+)?)\s+
        y=([+-]?\d+(?:\.\d+)?)\s+
        z=([+-]?\d+(?:\.\d+)?)\s+
        yaw=([+-]?\d+(?:\.\d+)?)\s+
        target_index=([+-]?\d+)\s+
        target_x=([+-]?\d+(?:\.\d+)?)\s+
        target_y=([+-]?\d+(?:\.\d+)?)\s+
        target_z=([+-]?\d+(?:\.\d+)?)\s+
        unix=([+-]?\d+(?:\.\d+)?)
    """,
    re.IGNORECASE | re.VERBOSE
)

def parse_pose_data(line: str):
    m = POSE_RE.search(line)
    if not m:
        return None  
    trial = int(m.group(1))
    x = float(m.group(2)); y = float(m.group(3)); z = float(m.group(4))
    yaw = float(m.group(5))
    tind = int(m.group(6))
    tx = float(m.group(7)); ty = float(m.group(8)); tz = float(m.group(9))
    ts = float(m.group(10))
    return trial, x, y, z, yaw, tind, tx, ty, tz, ts

def parse_open_data(data):

    pattern = r'trial=(\d+)\s+chest_x=([\d.-]+)\s+chest_y=([\d.-]+)\s+chest_z=([\d.-]+)\s+is_target=(\w+)\s+target_index=(\d+)\s+target_x=([\d.-]+)\s+target_y=([\d.-]+)\s+target_z=([\d.-]+)\s+unix=([\d.]+)'
    match = re.search(pattern, data)
    
    if match:
        return {
            'trial': int(match.group(1)),
            'chest_x': float(match.group(2)),
            'chest_y': float(match.group(3)),
            'chest_z': float(match.group(4)),
            'is_target': match.group(5).lower() == 'true',
            'target_index': int(match.group(6)),
            'target_x': float(match.group(7)),
            'target_y': float(match.group(8)),
            'target_z': float(match.group(9)),
            'unix': float(match.group(10))
        }
    else:
        return None

def parse_roundend_data(data):

    pattern = r'trial=(\d+)\s+target_index=(\d+)\s+target_x=([\d.-]+)\s+target_y=([\d.-]+)\s+target_z=([\d.-]+)\s+rt=([\d.]+)s\s+unix=([\d.]+)'
    match = re.search(pattern, data)
    
    if match:
        return {
            'trial': int(match.group(1)),
            'chest_x': None,
            'chest_y': None,
            'chest_z': None,
            'is_target': None,
            'target_index': int(match.group(2)),
            'target_x': float(match.group(3)),
            'target_y': float(match.group(4)),
            'target_z': float(match.group(5)),
            'rt': float(match.group(6)),
            'unix': float(match.group(7))
        }
    else:
        return None

CSV_FIELDS = [
    "trial",
    "chest_x", "chest_y", "chest_z",
    "is_target",
    'target_index',
    'target_x', 'target_y', 'target_z',
    "rt",
    "unix"
]

def save_data_to_file(rows, filename):
    with open(filename, "w", encoding="utf-8") as f:
        f.write(",".join(CSV_FIELDS) + "\n")
        for row in rows:
            line = ",".join("" if row.get(k) is None else str(row.get(k))
                             for k in CSV_FIELDS)
            f.write(line + "\n")
    print(f" {filename}")


def prop2motor(prop):
    if prop <= 0.01:
        return 0
    elif 0.01 < prop < 0.926291403611155:
        return int(733.71 * prop**4 - 592.29 * prop ** 3 + 101.01 * prop ** 2 + 370.68 * prop + 50.024)
    elif prop < 1:
        return int(154437 * prop ** 3 - 418628 * prop **2 +  379046 * prop - 114111)
    else:
        return int(750)
    


def discheck(aa):
    nd = (1/(1+aa))**0.4

    return nd



def lrcheck(con, diff_yaw):

    if diff_yaw >= 0:
        left = con
        if abs(diff_yaw) < math.pi/2:
            right = con*(1 - (2*abs(diff_yaw))/math.pi)
        else:
            right = 0
    elif diff_yaw < 0:
        right = con
        if abs(diff_yaw) < math.pi/2:
            left = con*(1 - (2*abs(diff_yaw))/math.pi)
        else:
            left = 0

    return left, right


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


adb_cmd = "adb logcat -v raw"


process = subprocess.Popen(
    adb_cmd.split(),
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    encoding='utf-8', 
    bufsize=1
)


experiment_data = []
current_round_data = None
yaw = 0
n = 0
experiment_complete = False
shift = [0.570, -1.421]


try:
    for line in process.stdout:
        cleaned_line = line.strip()
        if not cleaned_line:
            continue
        
        
        # 实时更新状态
        if "[Pose]" in cleaned_line:
            trial, x, y, z, yaw, tind,tx,ty,tz,ts = parse_pose_data(cleaned_line)
            yaw = math.radians(yaw)
            x = x - shift[0]
            z = z - shift[1]
            relative_yaw = signed_angle((math.sin(yaw), math.cos(yaw)),(tx-x, tz-z))
            relative_dis = math.sqrt((tx-x)**2 + (tz-z)**2)
            nn = discheck(relative_dis)
            if relative_dis < 2:
                motor_ctrl.move_motors(prop2motor(nn), prop2motor(nn), speed=800)
            else:
                ll, rr = lrcheck(nn, relative_yaw)
                motor_ctrl.move_motors(prop2motor(ll), prop2motor(rr), speed=800)


        elif "[Open]" in cleaned_line:
            current_round_data = parse_open_data(cleaned_line)
            if current_round_data:
                experiment_data.append(current_round_data)
                print(f" {current_round_data}")
                
        
        elif "[RoundEnd]" in cleaned_line:
            current_round_data = parse_roundend_data(cleaned_line)
            if current_round_data:
                experiment_data.append(current_round_data)
                print(f"this round: {current_round_data}")

                motor_ctrl.move_motors(0, 0, speed=800)
                time.sleep(10)

        elif "[GameEnd]" in cleaned_line:
            experiment_complete = True
            break
            
except KeyboardInterrupt:
    print("\n end")
finally:
    process.terminate()
    motor_ctrl.move_motors(0, 0, speed=800)
    # 保存数据
    if experiment_data:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"test3_con_grad_{timestamp}.csv"
        save_data_to_file(experiment_data, filename)
    time.sleep(3)
    motor_ctrl.close()






            