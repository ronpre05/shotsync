# ShotSync

A glove-mounted wearable prototype that gives a basketball player instant feedback on their shooting form. It detects each shot from sensor data, compares it against the player's own saved reference shot, and shows a similarity score on an LCD.

Rather than judging against a universal "perfect" shot, ShotSync is personalised: the player saves a shot they are happy with, and later attempts are scored on how closely they match it.

## Hardware

Runs in Python on a Raspberry Pi with a GrovePi board.

| Component | Port | Role |
|---|---|---|
| Light sensor (palm) | 0 | Infers grip: covered when holding the ball, exposed on release |
| 6-axis IMU (accelerometer + gyroscope) | 1 | Captures the shooting motion |
| Push button | 4 | Saves the last shot as the reference |
| Grove RGB LCD | I2C | Shows status and score, colour-coded |

## How it works

1. **Calibration.** On start-up the player stays still for three seconds. The motion threshold is set to the mean resting acceleration plus three standard deviations.
2. **Grip detection.** The light level is turned into an open/closed grip state using hysteresis (separate open and close thresholds) and a three-sample debounce.
3. **Shot segmentation.** A state machine moves from idle, to grip detected, to in-shot once sustained motion exceeds the threshold. Reopening the grip marks the release and ends the shot.
4. **Feature extraction.** Each shot yields its duration, peak acceleration, peak gyroscope magnitude (smoothed) and time to that peak, plus the raw per-axis gyroscope curves.
5. **Validation.** The per-axis gyroscope curves are correlated with the reference. If the similarity is below 0.65 the motion is rejected as "Not a shot". This keeps shot *detection* separate from shot *quality*: a poor shot is still scored, while a pass or dribble is rejected.
6. **Scoring.** The score blends relative feature deviation (70%) and curve-shape difference (30%). Lower is better: under 25 shows green, 25 to 50 amber, above 50 red.

The recognition logic uses only the Python standard library. External modules are used solely for reading the sensors and driving the LCD.

## Results

Trials recorded against a single fixed reference shot (logs in `data/`):

| Dataset | Motion | Attempts | Scored | Rejected as non-shot | Cancelled | Mean score |
|---|---|---|---|---|---|---|
| `good_form` | Good-form shots | 10 | 10 | 0 | 0 | 24.76 |
| `good_form_orientation` | Good-form shots from varied court positions | 10 | 10 | 0 | 0 | 19.71 |
| `bad_form` | Deliberately poor shots | 11 | 9 | 1 | 1 | 55.00 |
| `non_shot` | Passes, pump fakes, dribbles | 7 | 1 | 6 | 0 | 79.12 |
| `idle_other_motions` | Claps, hands on hips, fist bumps | 15 | 1 | 0 | 14 | 148.73 |

Every genuine shot was detected, good form scored clearly better than bad form, and most unrelated motions were rejected or cancelled before being scored.

## Running it

On the Pi, with the sensors connected as above and the `sensors` and `grovelcd` helper modules for the GrovePi kit on the Python path:

```bash
python shotsync.py
```

Stay still during calibration until the LCD shows `Ready`. Take a shot, then press the button to save it as the reference (`reference.json`). Later shots are scored against it. The script also prints a CSV log row per sample (time, light, grip state, acceleration, gyroscope, state, event, score) for offline analysis.

## Limitations

- Scoring depends on a single reference shot, so a poor reference gives poor feedback. Storing several references would make it more robust.
- The grip thresholds are fixed and may need adjusting under different lighting.
- Curves are compared sample by sample without time alignment.
- Testing was small-scale with one user, so the results show feasibility rather than general accuracy.

## Author

Ron Prekopuca
