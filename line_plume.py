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
            print(f" {cmd.strip()}")
        else:
            print("timeout")

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
        r"sCam=(-?\d+\.\d+)\s+"
        r"errS=(-?\d+\.\d+)\s+"
        r"unix=(-?\d+\.\d+)"
    )
    match = re.search(pattern, data)
    
    if match:
        return int(match.group(1)),float(match.group(2)),float(match.group(3)),float(match.group(4)), float(match.group(5)), float(match.group(6)), float(match.group(7)), float(match.group(8)), float(match.group(9))
    else:
        return None

def parse_round_result(data):

    pattern = r'targetS=([\d.]+)\s+lastS=([\d.]+)\s+errS=([\d.]+)\s+rt=([\d.]+)'
    match = re.search(pattern, data)
    
    if match:
        return {
            'targetS': float(match.group(1)),
            'lastS': float(match.group(2)),
            'errS': float(match.group(3)),
            'rt': float(match.group(4))
        }
    else:
        return None

def save_data_to_file(data, filename):
    with open(filename, 'w') as f:
        f.write("targetS,lastS,errS,rt\n")
        for row in data:
            f.write(f"{row['targetS']},{row['lastS']},{row['errS']},{row['rt']}\n")
    print(f"{filename}")

def prop2motor(prop):
    if prop <= 0.01:
        return 0
    elif 0.01 < prop < 0.926291403611155:
        return int(733.71 * prop**4 - 592.29 * prop ** 3 + 101.01 * prop ** 2 + 370.68 * prop + 50.024)
    elif prop < 1:
        return int(154437 * prop ** 3 - 418628 * prop **2 +  379046 * prop - 114111)
    else:
        return int(750)
    


pluse_mode = False
last_pluse_mode = False
last_switch_time = time.time()
current_dis = 0
last_dis = 0
conc = 0

def pluse_intens(dis):
    if abs(dis) <= 2:
        return 0.5*math.exp(-abs(dis))
    else:
        return 0.4
    
def pluse_freq(dis):
    if abs(dis) > 6:
        return 0
    else:
        return 1.5/(2+3*abs(dis))

def pluse_dur(dis):
    if abs(dis) <= 6:
        return max((-0.25)*abs(dis) + 2.5, 0.0)
    else:
        return 0

def pluse_base(dis):
    return math.exp(-0.4*abs(dis))

def toggle_state():
    global pluse_mode, last_pluse_mode, last_switch_time

    current_time = time.time()

    if last_pluse_mode != pluse_mode:
        last_switch_time = current_time
        last_pluse_mode = pluse_mode

    time_since_last_switch = current_time - last_switch_time
    return time_since_last_switch


def gen_plume(current_dis):
    global pluse_mode, conc, last_dis

    intens = pluse_intens(current_dis)
    freq = pluse_freq(current_dis)
    dur = pluse_dur(current_dis)
    base = pluse_base(current_dis)

    last_time = toggle_state()

    if freq > 0:
        if pluse_mode == False and  last_time >= ((1/freq) - dur):
            conc = intens + base
            pluse_mode = True
            toggle_state()
            last_dis = current_dis
        
        elif pluse_mode == True:
            if last_time >= dur:
                conc = base
                pluse_mode = False
                toggle_state()
            elif abs(current_dis - last_dis) > 2: 
                conc = base
                pluse_mode = False
                toggle_state()
    else:
        conc = base
        pluse_mode = False
        toggle_state()



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
target = 0
n = 0
experiment_complete = False

try:

    for line in process.stdout:
        cleaned_line = line.strip()
        if not cleaned_line:
            continue

        if "[Trial]" in cleaned_line and "yaw=" in cleaned_line:
            trial, target, x, y, z, yaw, sCam, errS, ts = parse_trial_data(cleaned_line)
            relative_dis = abs(sCam - target)
            gen_plume(relative_dis)
            
            if n > 10:
                motor_ctrl.move_motors(prop2motor(conc), prop2motor(conc), speed=800)
                
            n += 1
            
        # 每轮结束时的结果
        elif "[DistanceTrial] end" in cleaned_line:
            current_round_data = parse_round_result(cleaned_line)
            if current_round_data:
                experiment_data.append(current_round_data)
                print(f" {current_round_data}")

                motor_ctrl.move_motors(0, 0, speed=800)
                time.sleep(10)

        elif "[DistanceTrial] block finished" in cleaned_line:
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
        filename = f"test2_plume_{timestamp}.csv"
        save_data_to_file(experiment_data, filename)


