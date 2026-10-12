#!/usr/bin/env python3
import csv, os, sys
import numpy as np

HALF = 0.30

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

def deriv(t, v, half=HALF):
    t = np.asarray(t); v = np.asarray(v)
    a = np.full(t.shape, np.nan); j = np.full(t.shape, np.nan)
    for i in range(len(t)):
        m = (t >= t[i] - half) & (t <= t[i] + half)
        if m.sum() < 8:
            continue
        tt = t[m] - t[i]
        c = np.polyfit(tt, v[m], 3)
        a[i] = c[2]
        j[i] = 2 * c[1]
    return a, j

def run(D, label):
    vel = sorted(load(D, 'vehicle_status_velocity_status.csv'), key=lambda r: r['t'])
    inf = sorted(load(D, 'vehicle_brake_info.csv'), key=lambda r: r['t'])
    brk = sorted(load(D, 'vehicle_brake_report.csv'), key=lambda r: r['t'])
    ulc = load(D, 'vehicle_ulc_cmd.csv')
    t0 = vel[0]['t']
    for rows in (vel, inf, brk, ulc):
        for r in rows: r['t'] -= t0

    print("=" * 76)
    print(f"RUN: {label}")
    lim = {f: sorted({r[f] for r in ulc if r['cmd_type'] == 1})
           for f in ('limit_accel','limit_decel','limit_jerk_throttle','limit_jerk_brake')}
    print("  " + "  ".join(f"{k.replace('limit_','')}={v}" for k, v in lim.items()))
    print("=" * 76)

    tv = [r['t'] for r in vel]
    vv = [r['longitudinal_velocity'] for r in vel]
    acc, jrk = deriv(tv, vv)
    tv = np.array(tv); vv = np.array(vv)

    def engaged(t):
        r = min(brk, key=lambda r: abs(r['t'] - t))
        return r['external_control'] and not r['override_active']

    trq = [(r['t'], r['brake_torque_request']) for r in inf]
    eps, i = [], 0
    while i < len(trq):
        if trq[i][1] > 50 and engaged(trq[i][0]):
            j0, gap = i, 0
            while j0 < len(trq) - 1:
                j0 += 1
                gap = 0 if trq[j0][1] > 50 else gap + 1
                if gap > 25: break
            ta, tb = trq[i][0], trq[j0][0]
            seg = (tv >= ta) & (tv <= tb)
            if seg.sum() > 20 and vv[seg].max() > 2.5 and vv[seg].min() < 0.3:
                eps.append((ta, tb))
            i = j0
        i += 1

    out = []
    for (ta, tb) in eps:
        seg = (tv >= ta) & (tv <= tb)
        ts, vs, as_, js = tv[seg], vv[seg], acc[seg], jrk[seg]
        ok = ~np.isnan(as_)
        ts, vs, as_, js = ts[ok], vs[ok], as_[ok], js[ok]
        if len(ts) < 20: continue

        below = np.where(vs < 0.15)[0]
        i_stop = below[0] if len(below) else len(vs) - 1
        t_stop = ts[i_stop]

        w = (ts >= t_stop - 1.0) & (ts <= t_stop)
        end_dec = as_[w].min() if w.any() else float('nan')
        w2 = (ts >= t_stop - 0.3) & (ts <= t_stop + 0.7)
        rel_jerk = np.abs(js[w2]).max() if w2.any() else float('nan')

        moving = ts <= t_stop
        e = dict(
            v0=vs[0], dur=t_stop - ts[0],
            peak_dec=as_[moving].min(),
            end_dec=end_dec,
            jerk_rms=float(np.sqrt(np.mean(js[moving]**2))),
            jerk_pk=float(np.abs(js[moving]).max()),
            rel_jerk=float(rel_jerk),
            bad=100.0*np.mean(np.abs(js[moving]) > 1.0),
        )
        WI = [r for r in inf if ta <= r['t'] <= t_stop]
        tr = [r['brake_torque_request'] for r in WI]
        if len(tr) > 2:
            d = [abs(y-x) for x, y in zip(tr, tr[1:])]
            rev, prev = 0, 0
            for x, y in zip(tr, tr[1:]):
                s = (y > x) - (y < x)
                if s and prev and s != prev: rev += 1
                if s: prev = s
            dur = WI[-1]['t'] - WI[0]['t']
            e.update(trq_max=max(tr), trq_step=float(np.mean(d)),
                     chatter=rev/dur/2 if dur else 0)
        out.append(e)

        print(f"\n  stop from {e['v0']:.2f} m/s over {e['dur']:.1f}s  (t={ta:.1f}s)")
        print(f"    peak decel                {e['peak_dec']:7.3f} m/s^2")
        print(f"    decel in last 1s of motion{e['end_dec']:7.3f} m/s^2   <- head-nod")
        print(f"    RMS jerk (while moving)   {e['jerk_rms']:7.3f} m/s^3")
        print(f"    peak jerk                 {e['jerk_pk']:7.3f} m/s^3")
        print(f"    jerk at brake release     {e['rel_jerk']:7.3f} m/s^3")
        print(f"    |jerk| > 1                {e['bad']:6.1f} %")
        print(f"    brake torque max          {e.get('trq_max',0):7.0f} Nm")
        print(f"    mean torque step/tick     {e.get('trq_step',0):7.1f} Nm")
        print(f"    chatter                   {e.get('chatter',0):7.1f} Hz")
    return out

A = run(sys.argv[1], "BASELINE   3 / 3 / 0 / 0")
print()
B = run(sys.argv[2], "STAGE 1    1.5 / 2.0 / 1.0 / 1.0")

print()
print("=" * 76)
print("MATCHED COMPARISON  (episodes averaged, |v0| within 1 m/s of 5.6)")
print("=" * 76)
sel = lambda L: [e for e in L if abs(e['v0'] - 5.6) < 1.0] or L
a, b = sel(A), sel(B)
def avg(L, k): return float(np.mean([e[k] for e in L]))
print(f"  baseline episodes: {len(a)}   stage1 episodes: {len(b)}")
print(f"  {'metric':30s} {'baseline':>10s} {'stage1':>10s} {'change':>10s}")
for k, name, want in [
        ('v0',       'approach speed   m/s',   None),
        ('dur',      'stop duration      s',   None),
        ('peak_dec', 'peak decel     m/s^2',   None),
        ('end_dec',  'last-1s decel  m/s^2',   None),
        ('jerk_rms', 'RMS jerk       m/s^3',   'lower'),
        ('jerk_pk',  'peak jerk      m/s^3',   'lower'),
        ('rel_jerk', 'release jerk   m/s^3',   'lower'),
        ('bad',      '|jerk|>1           %',   'lower'),
        ('trq_max',  'brake torque max  Nm',   'lower'),
        ('trq_step', 'torque step/tick  Nm',   'lower'),
        ('chatter',  'chatter           Hz',   'lower')]:
    x, y = avg(a, k), avg(b, k)
    if abs(x) > 1e-9:
        pct = 100 * (abs(y) - abs(x)) / abs(x)
        mark = ""
        if want == 'lower':
            mark = "  better" if pct < -5 else ("  worse" if pct > 5 else "  ~same")
        print(f"  {name:30s} {x:10.3f} {y:10.3f} {pct:+9.1f}%{mark}")
    else:
        print(f"  {name:30s} {x:10.3f} {y:10.3f}")
