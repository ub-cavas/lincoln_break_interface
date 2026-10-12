#!/usr/bin/env python3
import csv, os, sys
from collections import Counter

def load(D, n):
    with open(os.path.join(D, n)) as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k, v in r.items():
            try: r[k] = float(v)
            except (ValueError, TypeError): pass
    return rows

for D in sys.argv[1:]:
    u = load(D, 'vehicle_ulc_report.csv')
    v = load(D, 'vehicle_status_velocity_status.csv')
    print(f"--- {D} ---")

    a = sorted({r['accel_meas'] for r in u})
    steps = Counter(round(y - x, 6) for x, y in zip(a, a[1:]))
    print(f"  accel_meas distinct values: {len(a)}")
    print(f"  smallest gaps between distinct values: {sorted(steps)[:5]}")

    vv = sorted({r['longitudinal_velocity'] for r in v})
    vsteps = Counter(round(y - x, 6) for x, y in zip(vv, vv[1:]))
    print(f"  velocity distinct values: {len(vv)}")
    print(f"  smallest gaps: {sorted(vsteps)[:5]}")

    dts = [q['t'] - p['t'] for p, q in zip(u, u[1:]) if q['t'] > p['t']]
    dt = sorted(dts)[len(dts)//2]
    print(f"  median sample dt: {dt*1000:.1f} ms  ({1/dt:.0f} Hz)")
    q = sorted(steps)[0] if steps else 0
    print(f"  ==> ONE quantization step = {q/dt:.2f} m/s^3 of apparent jerk")
