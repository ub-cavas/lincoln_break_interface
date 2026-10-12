#!/usr/bin/env python3
import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = 'figures'
os.makedirs(OUT, exist_ok=True)

S1, S2, S3 = '#2a78d6', '#eb6834', '#1baf7a'
S8 = '#e34948'
INK, INK2, MUTED = '#0b0b0b', '#52514e', '#8a8a85'
SURF = '#fcfcfb'
GRID = '#e4e4e0'

plt.rcParams.update({
    'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'text.color': INK,
    'xtick.color': INK2, 'ytick.color': INK2, 'font.size': 9,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False,
    'axes.axisbelow': True,
})

HALF = 0.30


def load(D, n):
    p = os.path.join(D, n)
    if not os.path.exists(p):
        return None
    rows = list(csv.DictReader(open(p)))
    for r in rows:
        for k, v in r.items():
            if v == 'True':
                r[k] = True
            elif v == 'False':
                r[k] = False
            else:
                try:
                    r[k] = float(v)
                except (ValueError, TypeError):
                    pass
    return rows


def rebase(D):
    vel = sorted(load(D, 'vehicle_status_velocity_status.csv'), key=lambda r: r['t'])
    t0 = vel[0]['t']
    out = {'vel': vel}
    for key, fn in [('inf', 'vehicle_brake_info.csv'), ('ulc', 'vehicle_ulc_report.csv'),
                    ('cmd', 'control_command_control_cmd.csv'), ('brk', 'vehicle_brake_report.csv')]:
        rows = load(D, fn)
        if rows:
            out[key] = sorted(rows, key=lambda r: r['t'])
    for rows in out.values():
        for r in rows:
            r['t'] -= t0
    return out


def deriv(t, v, half=HALF):
    t = np.asarray(t); v = np.asarray(v)
    a = np.full(t.shape, np.nan); j = np.full(t.shape, np.nan)
    for i in range(len(t)):
        m = (t >= t[i] - half) & (t <= t[i] + half)
        if m.sum() < 8:
            continue
        c = np.polyfit(t[m] - t[i], v[m], 3)
        a[i] = c[2]; j[i] = 2 * c[1]
    return a, j


def near(rows, field, t):
    return min(rows, key=lambda r: abs(r['t'] - t))[field]


BASE_A = 'extracted_csv/csv_baseline_0818a'
STG1_B = 'extracted_csv/csv_stage1_0818b'
COAST_C = 'extracted_csv/csv_coast_0818c'
COAST_D = 'extracted_csv/csv_coast_0818d'


def fig1_command_chain():
    D = rebase(STG1_B)
    ta, tb = 113.2, 119.4
    tv = np.array([r['t'] for r in D['vel']])
    vv = np.array([r['longitudinal_velocity'] for r in D['vel']])
    acc, _ = deriv(tv, vv)
    s = (tv >= ta) & (tv <= tb)
    ts = tv[s] - ta

    grid = np.arange(0, tb - ta, 0.03)
    aw_v = [near(D['cmd'], 'longitudinal.velocity', ta + g) for g in grid]
    ref = [near(D['ulc'], 'accel_ref', ta + g) for g in grid]
    req = [near(D['inf'], 'brake_torque_request', ta + g) for g in grid]
    act = [near(D['inf'], 'brake_torque_actual', ta + g) for g in grid]

    fig, ax = plt.subplots(3, 1, figsize=(9.5, 8.2), sharex=True)

    ax[0].plot(grid, aw_v, color=S1, lw=2, label='Autoware velocity command')
    ax[0].plot(ts, vv[s], color=S3, lw=2, label='measured velocity')
    ax[0].set_ylabel('velocity  [m/s]')
    ax[0].set_title('Autoware commands a smooth ramp — zero direction reversals',
                    color=INK, fontsize=10, loc='left', pad=8)
    ax[0].legend(loc='upper right')

    ax[1].plot(grid, ref, color=S1, lw=2, label="ULC internal accel_ref")
    ax[1].plot(ts, acc[s], color=S3, lw=2, label='measured acceleration')
    ax[1].set_ylabel('acceleration  [m/s$^2$]')
    ax[1].set_title("The ULC's own acceleration reference is clean too (±1 quantisation step)",
                    color=INK, fontsize=10, loc='left', pad=8)
    ax[1].legend(loc='lower left')

    ax[2].plot(grid, req, color=S2, lw=2, label='brake torque REQUESTED')
    ax[2].plot(grid, act, color=S3, lw=2, label='brake torque DELIVERED')
    ax[2].set_ylabel('brake torque  [Nm]')
    ax[2].set_xlabel('time from brake onset  [s]')
    ax[2].set_title('…but the torque the ULC derives from it oscillates, and the brake ignores it',
                    color=INK, fontsize=10, loc='left', pad=8)
    ax[2].legend(loc='upper left')
    ax[2].annotate('requested torque swings 160↔600 Nm…',
                   xy=(1.55, 520), xytext=(1.15, 1080), color=S8, fontsize=9,
                   arrowprops=dict(arrowstyle='->', color=S8, lw=1.4))
    ax[2].annotate('…while the brake delivers essentially zero',
                   xy=(2.9, 25), xytext=(2.55, 330), color=S8, fontsize=9,
                   arrowprops=dict(arrowstyle='->', color=S8, lw=1.4))

    fig.suptitle('Where the wobble is generated — one Stage 1 stop from 5.6 m/s',
                 color=INK, fontsize=12, x=0.005, ha='left', y=0.995)
    fig.text(0.005, 0.005,
             'Every signal Autoware and the bridge produce is monotonic. The oscillation first appears '
             'inside the ULC, converting accel_ref into brake torque.',
             color=INK2, fontsize=8.5, ha='left')
    fig.tight_layout(rect=[0, 0.022, 1, 0.975])
    fig.savefig(f'{OUT}/fig1_command_chain.png', dpi=150)
    plt.close(fig)
    print('fig1_command_chain.png')


def fig2_head_nod():
    D = rebase(STG1_B)
    tv = np.array([r['t'] for r in D['vel']])
    vv = np.array([r['longitudinal_velocity'] for r in D['vel']])
    acc, _ = deriv(tv, vv)
    seg = (tv >= 113.2) & (tv <= 120.5)
    below = np.where(vv[seg] < 0.15)[0]
    t_stop = tv[seg][below[0]]

    grid = np.arange(-2.5, 0.85, 0.03)
    aw_a = [near(D['cmd'], 'longitudinal.acceleration', t_stop + g) for g in grid]
    real = np.interp(t_stop + grid, tv, acc)

    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    ax.axvline(0, color=MUTED, lw=1.2, ls='--')
    ax.plot(grid, aw_a, color=S1, lw=2, label='Autoware commanded acceleration')
    ax.plot(grid, real, color=S2, lw=2, label='actual vehicle acceleration')
    ax.set_xlabel('time relative to standstill  [s]')
    ax.set_ylabel('acceleration  [m/s$^2$]')
    ax.set_title('The head-nod: Autoware eases off, the car does the opposite',
                 color=INK, fontsize=12, loc='left', pad=10)
    ax.legend(loc='lower left')
    ax.text(0.04, 0.06, 'standstill', transform=ax.get_xaxis_transform(),
            color=MUTED, fontsize=8.5, rotation=90, va='bottom')
    ax.annotate('Autoware tapers off:  −2.1 → −0.5\n(this is the channel the bridge discards)',
                xy=(-0.34, -1.24), xytext=(-2.45, -0.30), color=S1, fontsize=9,
                arrowprops=dict(arrowstyle='->', color=S1, lw=1.4))
    ax.annotate('vehicle brakes hardest\nat the moment it stops',
                xy=(-0.09, -2.03), xytext=(-1.70, -2.85), color=S2, fontsize=9,
                arrowprops=dict(arrowstyle='->', color=S2, lw=1.4))
    ax.annotate('stopped_acc = −3.4 applied\nonce standstill is detected',
                xy=(0.42, -3.38), xytext=(0.02, -3.06), color=MUTED, fontsize=8.5,
                arrowprops=dict(arrowstyle='->', color=MUTED, lw=1.2))
    fig.text(0.005, 0.008,
             'Present in 17 of 17 braking episodes across all six runs, in every configuration tested.',
             color=INK2, fontsize=8.5, ha='left')
    fig.tight_layout(rect=[0, 0.045, 1, 1])
    fig.savefig(f'{OUT}/fig2_head_nod.png', dpi=150)
    plt.close(fig)
    print('fig2_head_nod.png')


def episodes(D):
    R = rebase(D)
    tv = np.array([r['t'] for r in R['vel']])
    vv = np.array([r['longitudinal_velocity'] for r in R['vel']])
    acc, jrk = deriv(tv, vv)

    def eng(t):
        r = min(R['brk'], key=lambda r: abs(r['t'] - t))
        return r['external_control'] and not r['override_active']

    trq = [(r['t'], r['brake_torque_request']) for r in R['inf']]
    eps, i = [], 0
    while i < len(trq):
        if trq[i][1] > 50 and eng(trq[i][0]):
            j0, gap = i, 0
            while j0 < len(trq) - 1:
                j0 += 1
                gap = 0 if trq[j0][1] > 50 else gap + 1
                if gap > 25:
                    break
            ta, tb = trq[i][0], trq[j0][0]
            s = (tv >= ta) & (tv <= tb)
            if s.sum() > 20 and vv[s].max() > 2.5 and vv[s].min() < 0.3:
                eps.append((ta, tb))
            i = j0
        i += 1

    out = []
    for ta, tb in eps:
        s = (tv >= ta) & (tv <= tb)
        ts, vs, as_, js = tv[s], vv[s], acc[s], jrk[s]
        ok = ~np.isnan(as_)
        ts, vs, as_, js = ts[ok], vs[ok], as_[ok], js[ok]
        if len(ts) < 20:
            continue
        below = np.where(vs < 0.15)[0]
        t_stop = ts[below[0]] if len(below) else ts[-1]
        mv = ts <= t_stop
        w2 = (ts >= t_stop - 0.3) & (ts <= t_stop + 0.7)
        W = [r for r in R['inf'] if ta <= r['t'] <= t_stop]
        tr = np.array([r['brake_torque_request'] for r in W])
        ac = np.array([r['brake_torque_actual'] for r in W])
        m = tr > 50
        out.append(dict(v0=vs[0], peak=as_[mv].min(),
                        jpk=float(np.abs(js[mv]).max()),
                        rel=float(np.abs(js[w2]).max()), trq=tr.max(),
                        dead=100 * np.mean(ac[m] < 20)))
    return out


def fig3_stage1():
    A, B = episodes(BASE_A), episodes(STG1_B)
    specs = [('peak', 'peak deceleration', 'm/s$^2$', True),
             ('jpk', 'peak jerk', 'm/s$^3$', True),
             ('rel', 'jerk at brake release', 'm/s$^3$', True),
             ('trq', 'peak brake torque', 'Nm', False)]
    fig, axes = plt.subplots(1, 4, figsize=(11, 4.3))
    for ax, (k, name, unit, flip) in zip(axes, specs):
        a = [abs(e[k]) for e in A]
        b = [abs(e[k]) for e in B]
        ax.scatter([0] * len(a), a, s=90, color=S2, zorder=3,
                   edgecolors=SURF, linewidths=2)
        ax.scatter([1] * len(b), b, s=90, color=S1, zorder=3,
                   edgecolors=SURF, linewidths=2)
        ax.plot([0, 1], [np.mean(a), np.mean(b)], color=MUTED, lw=1.4, ls='--', zorder=2)
        ax.set_xlim(-0.45, 1.45)
        ax.margins(y=0.22)
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['baseline', 'Stage 1'], fontsize=9)
        ax.set_title(f'{name}\n[{unit}]', color=INK, fontsize=9.5, pad=8)
        pct = 100 * (np.mean(b) - np.mean(a)) / np.mean(a)
        ax.text(0.5, 0.02, f'{pct:+.0f}%', transform=ax.transAxes,
                ha='center', color=INK2, fontsize=10)
        ax.grid(axis='x', visible=False)
    fig.suptitle('Stage 1 vs baseline — matched route, matched approach speeds, same session',
                 color=INK, fontsize=12, x=0.005, ha='left', y=0.99)
    fig.text(0.005, 0.005,
             'Each dot is one stop (n=3 each). On all four metrics every Stage 1 stop beats every baseline '
             'stop — the ranges do not overlap. Magnitudes shown; lower is better throughout.',
             color=INK2, fontsize=8.5, ha='left')
    fig.tight_layout(rect=[0, 0.055, 1, 0.955])
    fig.savefig(f'{OUT}/fig3_stage1_vs_baseline.png', dpi=150)
    plt.close(fig)
    print('fig3_stage1_vs_baseline.png')


def fig4_deadband():
    R = rebase(STG1_B)
    eps = [(61.4, 67.3), (113.2, 119.4), (149.0, 156.7)]
    req, act = [], []
    for ta, tb in eps:
        for r in R['inf']:
            if ta <= r['t'] <= tb:
                req.append(r['brake_torque_request'])
                act.append(r['brake_torque_actual'])
    req, act = np.array(req), np.array(act)
    bins = [(1, 200), (200, 400), (400, 600), (600, 700), (700, 900), (900, 1400), (1400, 3000)]
    labels, delivered, ideal = [], [], []
    for lo, hi in bins:
        m = (req >= lo) & (req < hi)
        if m.sum() == 0:
            continue
        labels.append(f'{lo}–{hi}')
        delivered.append(act[m].mean())
        ideal.append(req[m].mean())

    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    ax.bar(x - 0.19, ideal, 0.36, color=S2, label='requested (mean)',
           edgecolor=SURF, linewidth=2)
    ax.bar(x + 0.19, delivered, 0.36, color=S3, label='actually delivered (mean)',
           edgecolor=SURF, linewidth=2)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel('brake torque request band  [Nm]')
    ax.set_ylabel('brake torque  [Nm]')
    ax.set_title('The friction brake ignores anything under ~900 Nm',
                 color=INK, fontsize=12, loc='left', pad=10)
    ax.legend(loc='upper left')
    ax.grid(axis='x', visible=False)
    fig.text(0.005, 0.012,
             'Pooled over three Stage 1 stops. Roughly half the braking time is spent requesting torque that never\n'
             'arrives — the ULC hunts here, and each time it crosses far enough to bite, you feel a jolt.',
             color=INK2, fontsize=8.5, ha='left')
    fig.tight_layout(rect=[0, 0.085, 1, 1])
    fig.savefig(f'{OUT}/fig4_brake_deadband.png', dpi=150)
    plt.close(fig)
    print('fig4_brake_deadband.png')


def fig5_coast():
    C, Dd = episodes(COAST_C), episodes(COAST_D)
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    for i, (a, b) in enumerate(zip(C, Dd)):
        ax.plot([0, 1], [a['dead'], b['dead']], color=MUTED, lw=1.3, zorder=2)
        ax.text(1.06, b['dead'], f"{b['v0']:.1f} m/s approach",
                color=INK2, fontsize=8.5, va='center')
    c = [e['dead'] for e in C]
    d = [e['dead'] for e in Dd]
    ax.scatter([0] * len(c), c, s=110, color=S1, zorder=3, edgecolors=SURF,
               linewidths=2, label='coast_decel = false')
    ax.scatter([1] * len(d), d, s=110, color=S2, zorder=3, edgecolors=SURF,
               linewidths=2, label='coast_decel = true')
    ax.set_xlim(-0.35, 1.62)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(['false  (run c)', 'true  (run d)'])
    ax.set_ylabel('% of braking time requesting >50 Nm\nwhile delivering <20 Nm')
    ax.set_title('coast_decel does not reduce the hunting — all three matched pairs got worse',
                 color=INK, fontsize=11.5, loc='left', pad=10)
    ax.legend(loc='upper left')
    ax.grid(axis='x', visible=False)
    fig.text(0.005, 0.012,
             'SOC matched to within 0.5 points per episode (35.0/34.5/39.0 vs 34.5/34.5/39.5), approach speeds\n'
             'to 0.0%, same route 10 minutes apart — the best-controlled comparison in this investigation.',
             color=INK2, fontsize=8.5, ha='left')
    fig.tight_layout(rect=[0, 0.10, 1, 1])
    fig.savefig(f'{OUT}/fig5_coast_decel.png', dpi=150)
    plt.close(fig)
    print('fig5_coast_decel.png')


STAGE2_E = 'extracted_csv/csv_stage2_0818e'


def _taper(D, eps):
    R = rebase(D)
    tv = np.array([r['t'] for r in R['vel']])
    vv = np.array([r['longitudinal_velocity'] for r in R['vel']])
    a, _ = deriv(tv, vv)
    out = []
    for ta, tb in eps:
        s = (tv >= ta) & (tv <= tb)
        below = np.where(vv[s] < 0.15)[0]
        if not len(below):
            continue
        tstop = tv[s][below[0]]
        mv2 = (tv >= ta) & (tv <= tstop - 0.25)
        w = (tv >= tstop - 1.0) & (tv <= tstop - 0.25)
        peak2 = np.nanmin(a[mv2])
        tap = np.nanmean(a[w])
        out.append(dict(v0=vv[s].max(), peak=peak2, tap=tap, ratio=tap / peak2,
                        tstop=tstop, ta=ta))
    return out


def fig6_stage2():
    CFG = [('baseline\nCMD_VELOCITY', BASE_A,
            [(55.7, 61.5), (107.5, 113.6), (143.6, 151.2)], S2),
           ('Stage 1\nCMD_VELOCITY', COAST_C,
            [(57.2, 64.9), (108.8, 116.8), (145.3, 157.6)], S1),
           ('Stage 2\nCMD_ACCEL', STAGE2_E,
            [(62.9, 73.1), (174.6, 190.4)], S3)]

    fig, ax = plt.subplots(1, 2, figsize=(11.5, 4.8),
                           gridspec_kw={'width_ratios': [1, 1.25]})

    for i, (lbl, D, eps, col) in enumerate(CFG):
        T = _taper(D, eps)
        for t in T:
            ax[0].scatter([i], [t['ratio']], s=95, color=col, zorder=3,
                          edgecolors=SURF, linewidths=2)
            if t['ratio'] < 0.5:
                ax[0].annotate(f"{t['v0']:.1f} m/s", xy=(i, t['ratio']),
                               xytext=(i - 0.42, t['ratio'] - 0.09),
                               color=INK2, fontsize=8.5)
    ax[0].axhline(1.0, color=MUTED, lw=1.1, ls='--')
    ax[0].text(2.42, 1.02, 'no easing off', color=MUTED, fontsize=8.5, ha='right')
    ax[0].set_xlim(-0.55, 2.55)
    ax[0].set_xticks(range(len(CFG)))
    ax[0].set_xticklabels([c[0] for c in CFG], fontsize=9)
    ax[0].set_ylabel('taper ratio\n(last second of motion ÷ approach peak)')
    ax[0].set_ylim(0.15, 1.15)
    ax[0].set_title('Does the car ease off before stopping?',
                    color=INK, fontsize=11, loc='left', pad=10)
    ax[0].grid(axis='x', visible=False)

    R = rebase(STAGE2_E)
    tv = np.array([r['t'] for r in R['vel']])
    vv = np.array([r['longitudinal_velocity'] for r in R['vel']])
    a, _ = deriv(tv, vv)
    T = _taper(STAGE2_E, [(62.9, 73.1)])[0]
    tstop = T['tstop']
    grid = np.arange(-2.0, 0.30, 0.03)
    aw = [near(R['cmd'], 'longitudinal.acceleration', tstop + g) for g in grid]
    ax[1].plot(grid, aw, color=S1, lw=2, label='Autoware commanded')
    ax[1].plot(grid, np.interp(tstop + grid, tv, np.nan_to_num(a)), color=S2,
               lw=2, label='actual vehicle')
    ax[1].axvline(0, color=MUTED, lw=1.2, ls='--')
    ax[1].set_xlabel('time relative to standstill  [s]')
    ax[1].set_ylabel('acceleration  [m/s$^2$]')
    ax[1].set_title('Stage 2, 5.8 m/s — actual follows the taper until the last 0.25 s',
                    color=INK, fontsize=11, loc='left', pad=10)
    ax[1].annotate('residual settle at standstill\n(present in every config;\nexcluded from the ratio)',
                   xy=(-0.08, -1.42), xytext=(-1.95, -2.35), color=INK2, fontsize=8.5,
                   arrowprops=dict(arrowstyle='->', color=MUTED, lw=1.2))
    ax[1].legend(loc='lower left')

    fig.suptitle('Stage 2 (CMD_ACCEL) — head-nod at low approach speed',
                 color=INK, fontsize=12.5, x=0.005, ha='left', y=0.995)
    fig.text(0.005, 0.008,
             'n=2 for Stage 2 (one episode lost to a driver override). The 5.8 m/s stop drops to ratio 0.32; the '
             '8.5 m/s stop stays at 0.96 —\nthe taper lands at low speed and not yet at high speed. Needs more stops.',
             color=INK2, fontsize=8.5, ha='left')
    fig.tight_layout(rect=[0, 0.075, 1, 0.955])
    fig.savefig(f'{OUT}/fig6_stage2_taper.png', dpi=150)
    plt.close(fig)
    print('fig6_stage2_taper.png')


if __name__ == '__main__':
    fig1_command_chain()
    fig2_head_nod()
    fig3_stage1()
    fig4_deadband()
    fig5_coast()
    fig6_stage2()
    print(f'\nwrote 6 figures to {OUT}/')
