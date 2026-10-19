# -*- coding: utf-8 -*-
"""
Convert an OpenRocket simulation export into simulated flight computer sensor data.

The output is a CSV sampled at a fixed rate, with what HPR's sensors would read
(accelerometer, gyro, barometer, GPS) plus the true values from OpenRocket for
reference. Use it to replay a flight through the event logic (C++ or Rust).

Usage:
    python3 openrocket_to_test.py <openrocket.csv> [-o out.csv] [--rate 1000]
                                  [--pad-seconds 2] [--ground-seconds 10]
                                  [--no-noise] [--seed 42]

OpenRocket export requirements:
    Export all variables (or at least the ones in REQUIRED_COLUMNS), comma
    separated, with "include flight events" and "include field descriptions"
    turned on.

Output units match what HPR works in after applying sensor gains:
    accelerometer in m/s^2 (reads +9.8 at rest), gyro in deg/s,
    altitude in m above the launch site, pressure in mbar, time in microseconds.
"""

import argparse
import csv
import math
import os
import random
import re

REQUIRED_COLUMNS = {
    "time": "Time (s)",
    "alt": "Altitude (m)",
    "alt_asl": "Altitude above sea level (m)",
    "vvel": "Vertical velocity (m/s)",
    "vacc": "Vertical acceleration (m/s²)",
    "east": "Position East of launch (m)",
    "north": "Position North of launch (m)",
    "roll": "Roll rate (°/s)",
    "pitch": "Pitch rate (°/s)",
    "yaw": "Yaw rate (°/s)",
    "zenith": "Vertical orientation (zenith) (°)",
    "mass": "Mass (g)",
    "grav": "Gravitational acceleration (m/s²)",
    "thrust": "Thrust (N)",
    "drag": "Drag force (N)",
    "temp": "Air temperature (°C)",
    "press": "Air pressure (mbar)",
    "mach": "Mach number (​)",
}

# standard deviation of the noise added to each simulated sensor
NOISE = {
    "accel": 0.1,   # m/s^2
    "gyro": 0.1,    # deg/s
    "alt": 0.5,     # m
    "press": 0.06,  # mbar (about 0.5 m)
}

EVENT_PATTERN = re.compile(r"#\s*Event\s+(\w+)\s+occurred at t=([\d.eE+-]+)")


def read_openrocket(path):
    """Return (column header list, data rows as float lists, [(time, event name)])."""
    header = None
    rows = []
    events = []
    with open(path, newline="", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                match = EVENT_PATTERN.match(line)
                if match:
                    events.append((float(match.group(2)), match.group(1)))
                elif header is None and "Time (s)" in line:
                    header = [h.strip() for h in line.lstrip("#").split(",")]
                continue
            rows.append([float(v) if v != "NaN" else math.nan for v in line.split(",")])
    if header is None:
        raise SystemExit("No column header found. Export with 'include field descriptions' on.")
    return header, rows, events


def column_indices(header):
    missing = [name for name in REQUIRED_COLUMNS.values() if name not in header]
    if missing:
        raise SystemExit("OpenRocket export is missing columns:\n  " + "\n  ".join(missing))
    return {key: header.index(name) for key, name in REQUIRED_COLUMNS.items()}


def fill_nans(rows, idx):
    """Replace NaN with the last valid value in that column (0 if none yet)."""
    last = {key: 0.0 for key in idx}
    for row in rows:
        for key, i in idx.items():
            if math.isnan(row[i]):
                row[i] = last[key]
            else:
                last[key] = row[i]


def interpolate(rows, idx, t):
    """Linearly interpolate every column at time t. Assumes rows are sorted by time."""
    times = interpolate.times
    lo, hi = 0, len(times) - 1
    if t <= times[0]:
        return {key: rows[0][i] for key, i in idx.items()}
    if t >= times[-1]:
        return {key: rows[-1][i] for key, i in idx.items()}
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    t0, t1 = times[lo], times[hi]
    frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return {key: rows[lo][i] + frac * (rows[hi][i] - rows[lo][i]) for key, i in idx.items()}


def accelerometer_reading(s, liftoff_time, apogee_time, t):
    """
    What an axial accelerometer reads, in m/s^2.

    An accelerometer measures every force except gravity, divided by mass. On the
    pad the pad pushes up on the rocket, so it reads +1 g. On the way up the forces
    along the body axis are thrust and drag, so it reads (thrust - drag) / mass:
    positive while the motor burns, negative while coasting. After apogee
    OpenRocket's drag no longer acts along the body axis, so use the vertical
    force instead (vertical acceleration + gravity), which reads about +1 g
    hanging under a parachute.
    """
    if t < liftoff_time:
        return s["grav"] * math.sin(math.radians(s["zenith"]))
    if t >= apogee_time:
        return s["vacc"] + s["grav"]
    mass_kg = s["mass"] / 1000.0
    return (s["thrust"] - s["drag"]) / mass_kg


def convert(in_path, out_path, rate, pad_seconds, ground_seconds, noise, seed):
    header, rows, events = read_openrocket(in_path)
    idx = column_indices(header)
    rows.sort(key=lambda r: r[idx["time"]])
    fill_nans(rows, idx)
    interpolate.times = [r[idx["time"]] for r in rows]

    liftoff_time = next((t for t, name in events if name == "LIFTOFF"), 0.0)
    apogee_time = next((t for t, name in events if name == "APOGEE"), math.inf)
    end_time = interpolate.times[-1]
    rng = random.Random(seed)

    def noisy(value, kind):
        return value + rng.gauss(0.0, NOISE[kind]) if noise else value

    # map each OpenRocket event to the first output sample at or after it
    pending_events = sorted(events)

    step = 1.0 / rate
    n_samples = int(round((pad_seconds + end_time + ground_seconds) / step)) + 1
    columns = [
        "time_us", "time_s",
        "accel_z_mps2", "gyro_roll_dps", "gyro_pitch_dps", "gyro_yaw_dps",
        "baro_alt_m", "baro_pressure_mbar", "baro_temp_c",
        "gps_east_m", "gps_north_m", "gps_alt_asl_m",
        "off_vert_deg",
        "true_alt_m", "true_vert_vel_mps", "true_vert_accel_mps2", "mach",
        "openrocket_event",
    ]

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        for n in range(n_samples):
            out_t = n * step
            # simulated time: negative while sitting on the pad, 0 = OpenRocket ignition
            sim_t = out_t - pad_seconds
            s = interpolate(rows, idx, max(sim_t, 0.0))
            accel = accelerometer_reading(s, liftoff_time, apogee_time, sim_t)

            # after OpenRocket's last sample the rocket lies still on the ground
            if sim_t > end_time:
                s["vvel"] = s["vacc"] = 0.0
                accel = s["grav"]

            event_names = []
            while pending_events and pending_events[0][0] <= sim_t + 1e-9:
                event_names.append(pending_events.pop(0)[1])

            writer.writerow([
                int(round(out_t * 1e6)), f"{out_t:.6f}",
                f"{noisy(accel, 'accel'):.4f}",
                f"{noisy(s['roll'], 'gyro'):.4f}",
                f"{noisy(s['pitch'], 'gyro'):.4f}",
                f"{noisy(s['yaw'], 'gyro'):.4f}",
                f"{noisy(s['alt'], 'alt'):.3f}",
                f"{noisy(s['press'], 'press'):.3f}",
                f"{s['temp']:.2f}",
                f"{s['east']:.3f}", f"{s['north']:.3f}", f"{s['alt_asl']:.3f}",
                f"{90.0 - s['zenith']:.3f}",
                f"{s['alt']:.3f}", f"{s['vvel']:.4f}", f"{s['vacc']:.4f}", f"{s['mach']:.4f}",
                "|".join(event_names),
            ])

    return n_samples, events


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="OpenRocket CSV export")
    parser.add_argument("-o", "--output", help="output CSV (default: <input>_sensors.csv next to the input)")
    parser.add_argument("--rate", type=float, default=1000.0, help="output sample rate in Hz (default 1000)")
    parser.add_argument("--pad-seconds", type=float, default=2.0,
                        help="seconds of rocket sitting on the pad before ignition (default 2)")
    parser.add_argument("--ground-seconds", type=float, default=10.0,
                        help="seconds of rocket lying on the ground after landing (default 10)")
    parser.add_argument("--no-noise", action="store_true", help="output clean values with no sensor noise")
    parser.add_argument("--seed", type=int, default=42, help="random seed for the noise (default 42)")
    args = parser.parse_args()

    out_path = args.output or os.path.splitext(args.input)[0] + "_sensors.csv"
    n_samples, events = convert(args.input, out_path, args.rate, args.pad_seconds,
                                args.ground_seconds, not args.no_noise, args.seed)

    print(f"Wrote {n_samples} samples at {args.rate:g} Hz to {out_path}")
    print(f"Pad time before ignition: {args.pad_seconds:g} s (OpenRocket times are shifted by this)")
    print("OpenRocket events:")
    for t, name in events:
        print(f"  {t + args.pad_seconds:8.3f} s  {name}")


if __name__ == "__main__":
    main()
