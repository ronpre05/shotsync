import time
import sensors
import math
import json
import os
import grovelcd

# config
SAMPLE_DT = 0.02  # ~50Hz


sensors.set_pins({"light": 0, "accel": 1, "gyro": 1, "button": 4})

# Grip thresholds
CLOSE_T = 100   
OPEN_T  = 550   
GRIP_N  = 3     # debounce samples

# Motion detection
BASELINE_SECONDS = 3.0
MIN_DUR_S = 0.08
MOTION_N = max(1, int(MIN_DUR_S / SAMPLE_DT))

# Shot validation threshold

MIN_SHAPE_SIM = 0.65 # bad shots should still pass

# Reference file
REF_FILE = "reference.json"

# Keep the last shot's raw samples so button-press can save them later
last_shot_features = None
last_shot_samples = None  # list of tuples (t, acc_mag, gx, gy, gz)

# LCD helpers
def lcd_ready():
    grovelcd.setRGB(0, 0, 255)   
    grovelcd.setText("Ready")

def lcd_ref_saved():
    grovelcd.setRGB(0, 255, 0)   
    grovelcd.setText("REF SAVED")

def lcd_no_last_shot():
    grovelcd.setRGB(255, 0, 0)   
    grovelcd.setText("No last shot")

def lcd_non_shot():
    grovelcd.setRGB(255, 0, 0)   
    grovelcd.setText("Not a shot")

def lcd_score(score):
    if score < 25:
        grovelcd.setRGB(0, 255, 0)        
    elif score < 50:
        grovelcd.setRGB(255, 165, 0)      
    else:
        grovelcd.setRGB(255, 0, 0)        
    grovelcd.setText(f"Score:\n{score:.2f}")

# Helpers 
def load_reference():
    if os.path.exists(REF_FILE):
        try:
            with open(REF_FILE, "r") as f:
                return json.load(f)
        except Exception:
            return None
    return None

def save_reference(ref):
    with open(REF_FILE, "w") as f:
        json.dump(ref, f, indent=2)

def compute_features(samples):
    """
    samples: list of tuples (t, acc_mag, gx, gy, gz)
    Uses gyro magnitude internally for feature extraction.
    """
    t0 = samples[0][0]
    ts = [s[0] - t0 for s in samples]
    acc = [s[1] for s in samples]
    gx = [s[2] for s in samples]
    gy = [s[3] for s in samples]
    gz = [s[4] for s in samples]

    gyro_mag = [math.sqrt(x*x + y*y + z*z) for x, y, z in zip(gx, gy, gz)]

    # smooth gyro magnitude (3-sample moving average)
    smooth = []
    for i in range(len(gyro_mag)):
        window = gyro_mag[max(0, i-1):min(len(gyro_mag), i+2)]
        smooth.append(sum(window) / len(window))

    peak_gyro = max(smooth)
    peak_i = smooth.index(peak_gyro)
    t_to_peak = ts[peak_i]

    feats = {
        "shot_duration": float(ts[-1]),
        "peak_gyro": float(peak_gyro),
        "t_to_peak_gyro": float(t_to_peak),
        "peak_acc": float(max(acc)),
    }
    return feats

def deviation_score(feats, ref):
    """
    Feature-based score (0 - 100). Lower is better.
    """
    eps = 1e-6

    def rel(a, b):
        return abs(a - b) / (abs(b) + eps)

    w_peak_gyro = 1.0
    w_t_peak    = 1.0
    w_peak_acc  = 0.5

    score = (
        w_peak_gyro * rel(feats["peak_gyro"], ref["peak_gyro"]) +
        w_t_peak    * rel(feats["t_to_peak_gyro"], ref["t_to_peak_gyro"]) +
        w_peak_acc  * rel(feats["peak_acc"], ref["peak_acc"])
    )
    return float(score * 100.0)

def corr01(a, b):
    """
    Signed correlation mapped to 0 to 1.
    1 = identical shape, 0.5 = unrelated, 0 = opposite.
    """
    n = min(len(a), len(b))
    if n < 3:
        return 0.0

    a = a[:n]
    b = b[:n]

    am = sum(a) / n
    bm = sum(b) / n

    a = [x - am for x in a]
    b = [y - bm for y in b]

    num = sum(x * y for x, y in zip(a, b))
    den = math.sqrt(sum(x * x for x in a)) * math.sqrt(sum(y * y for y in b))

    if den == 0:
        return 0.0

    c = num / den  # -1..1
    return (c + 1) / 2  # 0..1

def gyro_axes_similarity(samples, ref_axes):
    """
    Directional similarity based on signed gx/gy/gz curve shapes.
    Returns 0 to 1 (1 = very similar).
    """
    gx = [s[2] for s in samples]
    gy = [s[3] for s in samples]
    gz = [s[4] for s in samples]

    rgx = ref_axes["gx"]
    rgy = ref_axes["gy"]
    rgz = ref_axes["gz"]

    return (corr01(gx, rgx) + corr01(gy, rgy) + corr01(gz, rgz)) / 3.0

def dominant_axis(samples):
    """
    Returns dominant gyro axis by energy:
    0 = x, 1 = y, 2 = z
    """
    gx = [s[2] for s in samples]
    gy = [s[3] for s in samples]
    gz = [s[4] for s in samples]

    ex = sum(v * v for v in gx)
    ey = sum(v * v for v in gy)
    ez = sum(v * v for v in gz)

    energies = [ex, ey, ez]
    return max(range(3), key=lambda i: energies[i])


lcd_ready()

# Calibrate acceleration threshold 
print("Calibrating acc baseline for 3s... (stay still)")
acc_vals = []
t0 = time.time()
while time.time() - t0 < BASELINE_SECONDS:
    ax, ay, az = sensors.accel.get_xyz()
    acc_mag = math.sqrt(ax * ax + ay * ay + az * az)
    acc_vals.append(acc_mag)
    time.sleep(SAMPLE_DT)

mu = sum(acc_vals) / len(acc_vals)
sd = (sum((x - mu) ** 2 for x in acc_vals) / len(acc_vals)) ** 0.5
ACC_TH = mu + 3 * sd
print(f"Baseline mean={mu:.4f}, std={sd:.4f}, ACC_TH={ACC_TH:.4f}")

# Reference handling 
reference = load_reference()
if reference:
    print("Loaded reference from reference.json:", reference)
else:
    print("No reference loaded. Do a good shot, then press the button to save it as reference.")

#  State machines 
grip_state = 0  # 0 = open, 1 = closed
close_count = 0
open_count = 0

shot_state = 0  # 0 = idle, 1 = in_grip_wait_motion, 2 = in_shot_motion
motion_count = 0


prev_btn = 0

# Current shot buffer: list of tuples (t, acc_mag, gx, gy, gz)
shot_samples = []

print("time,light,grip_closed,acc_mag,gyro_mag,shot_state,event,score")

while True:
    t = time.time()

    
    light = sensors.light.get_level()
    ax, ay, az = sensors.accel.get_xyz()
    gx, gy, gz = sensors.gyro.get_xyz()
    btn = sensors.button.get_level()

    acc_mag = math.sqrt(ax * ax + ay * ay + az * az)
    gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)

    # Grip hysteresis + debounce 
    if grip_state == 0:
        if light < CLOSE_T:
            close_count += 1
            if close_count >= GRIP_N:
                grip_state = 1
                close_count = 0
        else:
            close_count = 0
    else:
        if light > OPEN_T:
            open_count += 1
            if open_count >= GRIP_N:
                grip_state = 0
                open_count = 0
        else:
            open_count = 0

    event = ""
    score_str = ""

    # Button save reference on press 
    if btn == 0 and prev_btn == 1:
        if last_shot_features is not None and last_shot_samples is not None:
            reference = dict(last_shot_features)  

            # Save directional reference curves (gx/gy/gz) from the LAST shot
            reference["gyro_axes"] = {
                "gx": [s[2] for s in last_shot_samples],
                "gy": [s[3] for s in last_shot_samples],
                "gz": [s[4] for s in last_shot_samples],
            }

            # Save dominant axis too
            reference["dom_axis"] = dominant_axis(last_shot_samples)

            save_reference(reference)
            event = "REFERENCE_SAVED"

            lcd_ref_saved()
            time.sleep(1)
            lcd_ready()
        else:
            event = "NO_LAST_SHOT_TO_SAVE"
            lcd_no_last_shot()
            time.sleep(1)
            lcd_ready()
    prev_btn = btn

    # Shot state machine
    if shot_state == 0:
        if grip_state == 1:
            shot_state = 1
            motion_count = 0
            if not event:
                event = "GRIP_START"

    elif shot_state == 1:
        if grip_state == 0:
            shot_state = 0
            if not event:
                event = "GRIP_CANCEL"
        else:
            if acc_mag > ACC_TH:
                motion_count += 1
                if motion_count >= MOTION_N:
                    shot_state = 2
                    shot_samples = []  
                    if not event:
                        event = "SHOT_START"
            else:
                motion_count = 0

    elif shot_state == 2:
        # buffer samples while in shot
        shot_samples.append((t, acc_mag, gx, gy, gz))

        # end when grip opens (release)
        if grip_state == 0:
            shot_state = 0
            if not event:
                event = "SHOT_END_RELEASE"

            if len(shot_samples) >= 5:
                last_shot_features = compute_features(shot_samples)
                last_shot_samples = list(shot_samples)

                if reference is not None and isinstance(reference, dict) and "gyro_axes" in reference:
                    score_feat = deviation_score(last_shot_features, reference)
                    shape_sim = gyro_axes_similarity(shot_samples, reference["gyro_axes"])
                    shape_score = (1 - shape_sim) * 100.0

                    # non-shot validation 
                    dom_axis = dominant_axis(shot_samples)
                    ref_dom_axis = reference.get("dom_axis", dom_axis)

                    # Reject only if:
                    # the directional shape is poor, OR
                    # the dominant axis is different
                    if shape_sim < MIN_SHAPE_SIM:
                        event = "NON_SHOT"
                        score_str = ""

                        lcd_non_shot()
                        time.sleep(1)
                        lcd_ready()
                    else:
                        score = 0.7 * score_feat + 0.3 * shape_score
                        score_str = f"{score:.4f}"

                        lcd_score(score)
                        time.sleep(2)
                        lcd_ready()

                elif reference is not None:
                    # old reference.json without directional curves
                    score = deviation_score(last_shot_features, reference)
                    score_str = f"{score:.4f}"

                    lcd_score(score)
                    time.sleep(2)
                    lcd_ready()
            else:
                last_shot_features = None
                last_shot_samples = None

            # clear buffer
            shot_samples = []

    print(f"{t},{light},{grip_state},{acc_mag:.6f},{gyro_mag:.6f},{shot_state},{event},{score_str}")
    time.sleep(SAMPLE_DT)