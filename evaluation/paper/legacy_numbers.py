"""Hardware numbers of the paper recomputed from the existing (pre-campaign) logs.

Covers everything that needs no forward kinematics:
  - Table V (tab:hw): all tracking runs, grouped by agent, script version and path duration
  - Sec. VI-B text: step counts, factors, completion, V2.3 statistics, loop times
  - Table VIII (tab:p2p_hw), Fig. 8 and Sec. VI-C text: set-point runs 068-073, command factors
    and joint-limit findings (A43)
The FK-based numbers (playback, Sec. V-B, and the Kortex tool offset, A42) come from
legacy_fk_numbers.m. Fig. 3 numbers come from fig/src/plot_smoothing_run074.py in the paper repo.

Definitions follow the scripts the paper numbers were first computed with:
  - effective command factor: mean over J2, J4 of max|dq_cmd| / max|dq_filt| (dq_cmd in deg/s,
    dq_filt in rad/s), as in the table of data/hardware/laptop_backup/README.md
  - completed: last wall time >= path duration - 0.6 s (logs without wall time: not completed)
  - RMS and max of ||e_p|| over the logged steps up to kEnd, a trailing zero entry dropped
  - loop time: mean of the positive wall-time differences
  - set-point KPIs as in hardware/analysis/evaluate_p2p_hardware.m

Usage (from the repository root):
    python evaluation/paper/legacy_numbers.py
Writes evaluation/paper/legacy_numbers.csv (key, value, unit, source files, note).
"""
import csv
import glob
import os
import statistics as st

import numpy as np
import scipy.io as sio
from scipy import stats

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HW = os.path.join(ROOT, 'data', 'hardware')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'legacy_numbers.csv')
TRACKING_GLOBS = ['deploy_logs/run_0*.mat', 'laptop_backup/runs/run_0*_agent_train_*.mat',
                  'runs/run_07*_agent_train_*.mat']
SETPOINT_RUNS = [68, 69, 70, 71, 72, 73]
SCRIPT_VERSION = {'deploy_agent_kinova_improved': 'V2.1', 'deploy_agent_kinova_robust_timing': 'V2.2',
                  'deploy_agent_kinova_robust_timing2': 'V2.3'}
rows = []


def put(key, value, unit='', files=(), note=''):
    rows.append({'key': key, 'value': value, 'unit': unit, 'files': ';'.join(files), 'note': note})
    v = f'{value:.4g}' if isinstance(value, float) else value
    print(f'  {key:<42} {v} {unit}  {note}')


def s(meta, name, default=''):
    return str(getattr(meta, name)) if hasattr(meta, name) else default


# ----------------------------------------------------------------------------------------------
# Tracking runs
# ----------------------------------------------------------------------------------------------
def tracking_run(path):
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    d, meta = m['data'], m['meta']
    n = int(meta.kEnd)
    cmd = np.abs(np.asarray(d.dq_cmd, float).reshape(-1, 7)[:n, [1, 3]])
    flt = np.abs(np.asarray(d.dq_filt, float).reshape(-1, 7)[:n, [1, 3]])
    eff = float(np.mean(cmd.max(0) / flt.max(0)) / (180 / np.pi))
    ep = np.asarray(d.ep_norm, float).ravel()[:n]
    tw = np.asarray(d.t_wall, float).ravel()[:n] if hasattr(d, 't_wall') else np.zeros(0)
    dur = float(meta.maxDuration)
    done = tw.size > 0 and tw[-1] > 0 and tw[-1] >= dur - 0.6
    if ep[-1] == 0:
        ep = ep[:-1]
    loop = float(np.mean(np.diff(tw[tw > 0]))) * 1e3 if tw.size and np.any(tw > 0) else np.nan
    agent = os.path.splitext(os.path.basename(s(meta, 'agentFile').replace('\\', '/')))[0]
    return {'file': os.path.relpath(path, HW).replace('\\', '/'), 'agent': agent,
            'version': SCRIPT_VERSION[s(meta, 'scriptCaller')], 'rateHz': float(meta.rateHz), 'dur': dur,
            'eff': eff, 'steps': n, 'done': done, 'rms': float(np.sqrt(np.mean(ep ** 2))),
            'max': float(ep.max()), 'loop_ms': loop,
            'phase_end': float(tw[-1] / dur) if tw.size and tw[-1] > 0 else np.nan}


def group_of(r):
    if r['version'] == 'V2.1':
        return 'G1_40Hz_V21'
    if r['agent'] == 'CDR2-4':
        return 'G2_CDR_V22'
    if r['version'] == 'V2.2':
        return 'G3_10Hz_V22_8p5s' if r['dur'] == 8.5 else 'G4_10Hz_V22_other'
    return 'G5_10Hz_V23_17s' if r['dur'] == 17 else 'G6_10Hz_V23_16s'


def logged_steps(path):
    return int(sio.loadmat(path, squeeze_me=True, struct_as_record=False)['meta'].kEnd)


# Runs 063-066 (27.05., copied V2.3 script) stopped at the first step and are skipped (kEnd < 2)
files = sorted(p for g in TRACKING_GLOBS for p in glob.glob(os.path.join(HW, g)) if logged_steps(p) >= 2)
runs = [tracking_run(p) for p in files]
groups = {}
for r in runs:
    groups.setdefault(group_of(r), []).append(r)

print(f'Tracking: {len(runs)} runs')
for g in sorted(groups):
    rs = groups[g]
    fs = [r['file'] for r in rs]
    done = [r for r in rs if r['done']]
    print(f'{g}: {len(rs)} runs')
    put(f'{g}.runs', len(rs), '', fs)
    put(f'{g}.completed', len(done), '', fs)
    put(f'{g}.eff_min', min(r['eff'] for r in rs), '', fs)
    put(f'{g}.eff_max', max(r['eff'] for r in rs), '', fs)
    put(f'{g}.steps_min', min(r['steps'] for r in rs), '', fs)
    put(f'{g}.steps_max', max(r['steps'] for r in rs), '', fs)
    base = done if done else rs
    tag = 'completed runs' if done else 'all runs until the stop'
    put(f'{g}.rms_min', min(r['rms'] for r in base), 'm', fs, tag)
    put(f'{g}.rms_max', max(r['rms'] for r in base), 'm', fs, tag)
    put(f'{g}.max_min', min(r['max'] for r in base), 'm', fs, tag)
    put(f'{g}.max_max', max(r['max'] for r in base), 'm', fs, tag)
    if len(base) > 1:
        put(f'{g}.rms_mean', st.mean(r['rms'] for r in base), 'm', fs, tag)
        put(f'{g}.rms_sd', st.stdev(r['rms'] for r in base), 'm', fs, tag)
        put(f'{g}.max_mean', st.mean(r['max'] for r in base), 'm', fs, tag)
        put(f'{g}.max_sd', st.stdev(r['max'] for r in base), 'm', fs, tag)
    loops = [r['loop_ms'] for r in rs if np.isfinite(r['loop_ms'])]
    if loops:
        put(f'{g}.loop_ms_min', min(loops), 'ms', fs, 'mean loop time per run')
        put(f'{g}.loop_ms_max', max(loops), 'ms', fs, 'mean loop time per run')
    if done:
        put(f'{g}.eff_completed_min', min(r['eff'] for r in done), '', fs)
        put(f'{g}.eff_completed_max', max(r['eff'] for r in done), '', fs)

# Sec. VI-B details
g3 = groups['G3_10Hz_V22_8p5s']
early = [r for r in g3 if not r['done']]
put('G3.stopped_eff_le_0.45', ', '.join(f'{r["eff"]:.2f}' for r in early if r['eff'] <= 0.451), '',
    [r['file'] for r in early], 'factors of stopped runs with factor <= 0.45')
put('G3.stopped_eff_1.0_steps', ', '.join(str(r['steps']) for r in early if r['eff'] > 0.99), 'steps',
    [r['file'] for r in early if r['eff'] > 0.99])
stopped = [r for r in runs if not r['done']]
put('stopped.runs', len(stopped), '', [r['file'] for r in stopped], 'all runs that did not complete')
put('stopped.max_min', min(r['max'] for r in stopped), 'm', [r['file'] for r in stopped],
    'largest logged error per stopped run, OOD limit 0.4 m, stop step not logged')
put('stopped.max_max', max(r['max'] for r in stopped), 'm', [r['file'] for r in stopped])
g6 = groups['G6_10Hz_V23_16s']
put('G6.steps', g6[0]['steps'], 'steps', [g6[0]['file']], 'run with factor 0.85')
v22 = [r for r in runs if r['version'] == 'V2.2' and r['agent'] != 'CDR2-4']
loops = sorted(r['loop_ms'] for r in v22)
put('V22.loop_ms_sorted', ', '.join(f'{x:.0f}' for x in loops), 'ms', [r['file'] for r in v22],
    'mean loop time per V2.2 run of the 10 Hz agent')
g5 = groups['G5_10Hz_V23_17s']
rms = [r['rms'] for r in g5]
ci = stats.t.ppf(0.975, len(rms) - 1) * st.stdev(rms) / np.sqrt(len(rms))
put('G5.rms_ci95_halfwidth', float(ci), 'm', [r['file'] for r in g5], 't distribution, 4 dof')
put('G5.phase_end_min', min(r['phase_end'] for r in g5), '', [r['file'] for r in g5],
    'last wall time / path duration')
lm = st.mean(r['loop_ms'] for r in g5)
put('G5.loop_ms_mean', lm, 'ms', [r['file'] for r in g5])
put('G5.loop_rate_hz', 1e3 / lm, 'Hz', [r['file'] for r in g5])

# ----------------------------------------------------------------------------------------------
# Set-point runs 068-073
# ----------------------------------------------------------------------------------------------
print('\nSet-point runs 068-073')
TOL, HOLD = 0.05, 0.5
sp = []
for run in SETPOINT_RUNS:
    path = glob.glob(os.path.join(HW, 'runs', f'run_{run:03d}_*.mat'))[0]
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    d, meta = m['data'], m['meta']
    t = np.asarray(d.t, float).ravel()
    ee = np.asarray(d.ee_measured, float).reshape(-1, 3)
    target = np.asarray(d.ee_ref, float).reshape(-1, 3)[-1]
    dist = np.linalg.norm(ee - target, axis=1)
    k = {'run': run, 'file': os.path.relpath(path, HW).replace('\\', '/'), 'scale': float(meta.speedScale),
         'd0': float(np.linalg.norm(target - ee[0])), 'final': float(dist[-1]), 'closest': float(dist.min()),
         'success': bool(dist[-1] < TOL), 'duration': float(t[-1] - t[0]), 'settle': np.nan}
    idx = np.flatnonzero(dist < TOL)
    for i0 in idx:
        msk = (t >= t[i0]) & (t <= min(t[i0] + HOLD, t[-1]))
        if np.all(dist[msk] < TOL) and (t[-1] - t[i0]) >= HOLD:
            k['settle'] = float(t[i0] - t[0])
            break
    path_len = float(np.sum(np.linalg.norm(np.diff(ee, axis=0), axis=1)))
    k['pathEff'] = min(1.0, k['d0'] / path_len) if path_len > 1e-6 else 0.0
    k['overshoot'] = max(0.0, float(dist[idx[0]:].max()) - TOL) if idx.size and idx[0] < dist.size - 1 else 0.0
    q = (np.asarray(d.q_measured, float).reshape(-1, 7) + 180.0) % 360.0 - 180.0
    k['j6_frac_zone'] = float(np.mean(np.abs(q[:, 5]) > 115.17 - 10.0))
    k['j6_max'] = float(np.abs(q[:, 5]).max())
    k['j4_min'] = float(q[:, 3].min())
    k['j4_end'] = float(q[-1, 3])
    sp.append(k)
sp.sort(key=lambda k: k['d0'])
for k in sp:
    f = [k['file']]
    r = f'{k["run"]:03d}'
    for name, unit, scale in [('d0', 'mm', 1e3), ('final', 'mm', 1e3), ('closest', 'mm', 1e3), ('settle', 's', 1),
                              ('pathEff', '', 1), ('overshoot', 'mm', 1e3), ('duration', 's', 1),
                              ('scale', '', 1), ('j6_frac_zone', '', 1), ('j6_max', 'deg', 1),
                              ('j4_min', 'deg', 1), ('j4_end', 'deg', 1)]:
        put(f'SP{r}.{name}', k[name] * scale, unit, f)
    put(f'SP{r}.success', k['success'], '', f)
allf = [k['file'] for k in sp]
ok = [k for k in sp if k['success']]
put('SP.mean_final', 1e3 * st.mean(k['final'] for k in sp), 'mm', allf)
put('SP.mean_closest', 1e3 * st.mean(k['closest'] for k in sp), 'mm', allf)
put('SP.successes', f'{len(ok)}/{len(sp)}', '', allf)
put('SP.mean_final_successes', 1e3 * st.mean(k['final'] for k in ok), 'mm', allf)
put('SP.final_successes_range', f'{1e3 * min(k["final"] for k in ok):.1f}-{1e3 * max(k["final"] for k in ok):.1f}',
    'mm', allf)
put('SP.settle_successes_range',
    f'{np.nanmin([k["settle"] for k in ok]):.1f}-{np.nanmax([k["settle"] for k in ok]):.1f}', 's', allf,
    'run 071 has no settling time (4 steps)')
put('SP.mean_pathEff', st.mean(k['pathEff'] for k in sp), '', allf)
put('SP.mean_overshoot', 1e3 * st.mean(k['overshoot'] for k in sp), 'mm', allf)
put('SP.d0_max_success', 1e3 * max(k['d0'] for k in ok), 'mm', allf)

with open(OUT, 'w', newline='', encoding='utf8') as fh:
    w = csv.DictWriter(fh, fieldnames=['key', 'value', 'unit', 'files', 'note'])
    w.writeheader()
    for row in rows:
        v = row['value']
        row = dict(row, value=f'{v:.6g}' if isinstance(v, float) else v)
        w.writerow(row)
print(f'\n{len(rows)} Werte nach {os.path.relpath(OUT, ROOT)}')
