#!/usr/bin/env python3
import csv, os, sys
from statistics import mean

D = sys.argv[1] if len(sys.argv) > 1 else '.'

def load(name):
    with open(os.path.join(D, name)) as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except (ValueError, TypeError):
                pass
    return rows

cmd = load('control_command_control_cmd.csv')
ulc_c = load('vehicle_ulc_cmd.csv')
ulc_r = load('vehicle_ulc_report.csv')
brk = load('vehicle_brake_report.csv')
inf = load('vehicle_brake_info.csv')
vel = load('vehicle_status_velocity_status.csv')

t0 = min(r['t'] for r in cmd + ulc_r + brk + vel)
for rows in (cmd, ulc_c, ulc_r, brk, inf, vel):
    for r in rows:
        r['t'] -= t0

print("=" * 78)
print("1. ULC COMMAND LIMITS ACTUALLY SENT")
print("=" * 78)
for f in ('limit_accel', 'limit_decel', 'limit_jerk_throttle', 'limit_jerk_brake', 'cmd_type', 'enable'):
    vals = sorted({r[f] for r in ulc_c})
    print(f"  {f:22s} = {vals}")

print()
print("=" * 78)
print("2. BRAKE REPORT: what mode / who is limiting")
print("=" * 78)
for f in ('cmd_type', 'external_control', 'ready', 'enabled', 'override_active',
          'limiting_value', 'limiting_rate'):
    vals = sorted({r[f] for r in brk})
    print(f"  {f:22s} = {vals[:8]}")

print()
print("=" * 78)
print("3. DECELERATION EVENTS (from velocity_status)")
print("=" * 78)
vel.sort(key=lambda r: r['t'])
v = [(r['t'], r['longitudinal_velocity']) for r in vel]

events = []
i = 0
while i < len(v) - 1:
    if v[i][1] > 1.0:
        j = i
        while j < len(v) - 1 and v[j + 1][1] <= v[j][1] + 0.05:
            j += 1
        if v[i][1] - v[j][1] > 2.0 and v[j][0] - v[i][0] > 1.0:
            events.append((i, j))
            i = j
    i += 1

for (a, b) in events:
    dt = v[b][0] - v[a][0]
    dv = v[b][1] - v[a][1]
    print(f"  t={v[a][0]:6.2f}->{v[b][0]:6.2f}s  ({dt:5.2f}s)  "
          f"v {v[a][1]:5.2f}->{v[b][1]:5.2f} m/s   mean decel {dv/dt:6.3f} m/s^2")

if not events:
    print("  (none found)")
    sys.exit(0)

a, b = max(events, key=lambda e: v[e[0]][1] - v[e[1]][1])
ta, tb = v[a][0] - 2.0, v[b][0] + 3.0
print(f"\n  --> analyzing window {ta:.2f} .. {tb:.2f} s")

def win(rows):
    return [r for r in rows if ta <= r['t'] <= tb]

wr = win(ulc_r)
print()
print("=" * 78)
print("4. MEASURED ACCEL / JERK DURING THE STOP  (ULC report accel_meas)")
print("=" * 78)
jerks = []
for p, q in zip(wr, wr[1:]):
    dt = q['t'] - p['t']
    if dt > 1e-4:
        jerks.append((q['t'], (q['accel_meas'] - p['accel_meas']) / dt))
if jerks:
    amax = min(r['accel_meas'] for r in wr)
    jpk = max(jerks, key=lambda x: abs(x[1]))
    big = [j for j in jerks if abs(j[1]) > 2.0]
    print(f"  peak decel        : {amax:7.3f} m/s^2")
    print(f"  peak jerk         : {jpk[1]:7.3f} m/s^3  at t={jpk[0]:.2f}s")
    print(f"  |jerk| > 2 m/s^3  : {len(big)} of {len(jerks)} samples "
          f"({100*len(big)/len(jerks):.1f}%)")
    print(f"  RMS jerk          : {(mean(j[1]**2 for j in jerks))**0.5:7.3f} m/s^3")

print()
print("=" * 78)
print("5. TRACE  (cmd velocity | ULC ref/meas | measured accel | brake torque)")
print("=" * 78)
print(f"  {'t':>6} {'aw_vel':>7} {'aw_acc':>7} {'ulc_ref':>8} {'ulc_meas':>8} "
      f"{'a_meas':>7} {'jerk':>7} {'trq_req':>8} {'trq_act':>8}")

def near(rows, t, f):
    r = min(rows, key=lambda r: abs(r['t'] - t))
    return r[f] if abs(r['t'] - t) < 0.3 else float('nan')

tt = ta
prev_a = None
while tt <= tb:
    am = near(ulc_r, tt, 'accel_meas')
    jk = float('nan')
    if prev_a is not None and am == am:
        jk = (am - prev_a) / 0.2
    prev_a = am
    print(f"  {tt:6.2f} {near(cmd,tt,'longitudinal.velocity'):7.2f} "
          f"{near(cmd,tt,'longitudinal.acceleration'):7.2f} "
          f"{near(ulc_r,tt,'vel_ref'):8.2f} {near(ulc_r,tt,'vel_meas'):8.2f} "
          f"{am:7.2f} {jk:7.2f} "
          f"{near(inf,tt,'brake_torque_request'):8.1f} "
          f"{near(inf,tt,'brake_torque_actual'):8.1f}")
    tt += 0.2

print()
print("=" * 78)
print("6. IS THE AUTOWARE VELOCITY COMMAND SMOOTH?")
print("=" * 78)
wc = win(cmd)
dv = []
for p, q in zip(wc, wc[1:]):
    dt = q['t'] - p['t']
    if dt > 1e-4:
        dv.append((q['t'], (q['longitudinal.velocity'] - p['longitudinal.velocity']) / dt))
if dv:
    print(f"  implied accel from d(vel_cmd)/dt: min {min(d[1] for d in dv):7.3f}  "
          f"max {max(d[1] for d in dv):7.3f} m/s^2")
    steps = [d for d in dv if abs(d[1]) > 2.0]
    print(f"  steps steeper than 2 m/s^2: {len(steps)} of {len(dv)}")
    print(f"  commanded longitudinal.acceleration range: "
          f"{min(r['longitudinal.acceleration'] for r in wc):7.3f} .. "
          f"{max(r['longitudinal.acceleration'] for r in wc):7.3f} m/s^2")
