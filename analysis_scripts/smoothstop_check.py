#!/usr/bin/env python3
import csv, os, sys
import numpy as np

def load(D, n):
    with open(os.path.join(D, n)) as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k, v in r.items():
            if v == 'True':  r[k] = True;  continue
            if v == 'False': r[k] = False; continue
            try: r[k] = float(v)
            except (ValueError, TypeError): pass
    return rows

def deriv(t, v, half=0.30):
    t = np.asarray(t); v = np.asarray(v)
    a = np.full(t.shape, np.nan)
    for i in range(len(t)):
        m = (t >= t[i]-half) & (t <= t[i]+half)
        if m.sum() >= 8:
            a[i] = np.polyfit(t[m]-t[i], v[m], 3)[2]
    return a

D = sys.argv[1]
vel = sorted(load(D, 'vehicle_status_velocity_status.csv'), key=lambda r: r['t'])
cmd = sorted(load(D, 'control_command_control_cmd.csv'), key=lambda r: r['t'])
t0 = vel[0]['t']
for rows in (vel, cmd):
    for r in rows: r['t'] -= t0

tv = np.array([r['t'] for r in vel])
vv = np.array([r['longitudinal_velocity'] for r in vel])
av = deriv(tv, vv)

tc = np.array([r['t'] for r in cmd])
ac = np.array([r['longitudinal.acceleration'] for r in cmd])
vc = np.array([r['longitudinal.velocity'] for r in cmd])

stops = []
for i in range(1, len(vv)):
    if vv[i] < 0.15 <= vv[i-1] and vv[max(0, i-150):i].max() > 3.0:
        stops.append(tv[i])

print(f"found {len(stops)} stops: {[f'{s:.1f}' for s in stops]}\n")

for ts in stops:
    print("=" * 70)
    print(f"STOP at t={ts:.2f}s   (last 4 s of motion)")
    print("=" * 70)
    print(f"  {'t-tstop':>8} {'v_meas':>7} {'v_cmd':>7} "
          f"{'AW acc_cmd':>11} {'actual acc':>11}")
    for dt in np.arange(-4.0, 1.01, 0.25):
        t = ts + dt
        i = int(np.argmin(np.abs(tv - t)))
        j = int(np.argmin(np.abs(tc - t)))
        if abs(tv[i]-t) > 0.2 or abs(tc[j]-t) > 0.2:
            continue
        print(f"  {dt:+8.2f} {vv[i]:7.2f} {vc[j]:7.2f} "
              f"{ac[j]:11.2f} {av[i]:11.2f}")

    w = (tc >= ts-2.0) & (tc <= ts)
    wm = (tv >= ts-2.0) & (tv <= ts)
    if w.any() and wm.any():
        cmd_peak, cmd_final = ac[w].min(), ac[w][-1]
        act_peak, act_final = np.nanmin(av[wm]), av[wm][-1]
        print(f"\n  in the last 2 s before standstill:")
        print(f"    AW commanded accel : peak {cmd_peak:6.2f} -> final {cmd_final:6.2f} "
              f"m/s^2  ({'TAPERS OFF' if cmd_final > cmd_peak + 0.15 else 'no taper'})")
        print(f"    vehicle actual     : peak {act_peak:6.2f} -> final {act_final:6.2f} "
              f"m/s^2  ({'tapers off' if act_final > act_peak + 0.15 else 'NO TAPER'})")
