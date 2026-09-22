"""Auswertung der Messkampagne: Table III, Table IV, Fig. 7 und die Set-Point-Reihe direkt aus den Logs.

Liest alle Kampagnen-Logs aus data/hardware/campaign/<condId>/*.mat
(geschrieben von hardware/campaign/deploy_tracking_v24.m, measure_loop_timing.m und
deploy_setpoint_v24.m) und rechnet alle Kennzahlen aus den Rohdaten neu. Die
Metadaten dienen nur zur Gruppierung und zur Kontrolle.

Ausgabe (Standard: data/hardware/campaign/results/):
    runs_tracking.csv       eine Zeile pro Tracking-Lauf
    runs_timing.csv         eine Zeile pro Timing-Messung
    runs_setpoint.csv       eine Zeile pro Set-Point-Lauf
    table3_looprate.tex     Tabellenzeilen fuer Table III (Loop-Zeiten)
    table3_breakdown.csv    Median jeder Teilzeit je Konfiguration
    table4_tracking.tex     Tabellenzeilen fuer Table IV (Tracking-Laeufe)
    table_setpoint_hw.tex   Tabellenzeilen fuer die Set-Point-Tabelle (Erfolg ueber d0)
    fig7_loop_histogram.pdf Verteilung der Schleifenzeiten (Fig. 7), Vektor-PDF
    fig_setpoint_d0.pdf     Endfehler ueber dem Startabstand d0, Vektor-PDF
    paper_numbers.csv       jede Zahl der Tabellen mit Quelle (Bedingung, Laeufe)

Aufruf (aus dem Repo-Wurzelordner):
    python evaluation/campaign/analyze_campaign.py
    python evaluation/campaign/analyze_campaign.py --include-dry --out <Ordner>

Trockenlaeufe (_dryrun) werden nur mit --include-dry gelesen und dann in allen
Ausgaben als DRY markiert.
"""
import argparse
import csv
import glob
import os
import statistics as st

import numpy as np
import scipy.io as sio

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
CAMPAIGN = os.path.join(ROOT, 'data', 'hardware', 'campaign')
FIRST_WINDOW_S = 3.4                   # Zeitfenster fuer den Vergleich mit abgebrochenen Laeufen
ACTIVE_JOINTS = [1, 3]                 # J2, J4 (0-basiert)
TIMING_ORDER = ['M_send', 'M_fb', 'M_fk', 'M_full']
SP_TOL = 0.05                          # Erfolgstoleranz der Set-Point-Reihe [m], wie Table VIII
SP_HOLD = 0.5                          # Haltezeit fuer die Setzzeit [s]
# Gelenkgrenzen des Gen3 (Datenblatt) [deg]. Training und Deploy nutzen fuer J2, J4 weitere
# Grenzen (138.1, 152.4 deg), J6 enger (115.2 deg). Siehe Befund A43.
HW_LIMIT_DEG = np.array([np.inf, 128.9, np.inf, 147.8, np.inf, 120.3, np.inf])
TIMING_LABEL = {'send_only': 'Send only (\\texttt{SendJointSpeedCommand})',
                'send_feedback': 'Send + feedback',
                'send_feedback_fk': 'Send + feedback + FK',
                'closed_loop_zero': 'Closed loop (+ \\texttt{getAction}, pipeline, logging)'}


def load_run(path):
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    run = m['run']
    return run.schema, run.meta, run.cfg, run.log


def arr(x, ncol=None):
    a = np.asarray(x, dtype=float)
    if ncol is not None and a.ndim == 1:
        a = a.reshape(-1, ncol) if a.size % ncol == 0 and ncol > 1 else a.reshape(-1, 1)
    return a


def fmt(x, nd=4):
    return 'n/a' if x is None or (isinstance(x, float) and np.isnan(x)) else f'{x:.{nd}f}'


def mean_sd(vals, nd=4):
    vals = [v for v in vals if v is not None and np.isfinite(v)]
    if not vals:
        return '--', None, None
    if len(vals) == 1:
        return f'{vals[0]:.{nd}f}', vals[0], None
    m, s = st.mean(vals), st.stdev(vals)
    return f'{m:.{nd}f} $\\pm$ {s:.{nd}f}', m, s


def tracking_metrics(path, meta, cfg, L):
    ep = arr(L.ep_norm).ravel()
    tw = arr(L.t_wall).ravel()
    dt = arr(L.dt_loop).ravel()[1:]
    dt = dt[np.isfinite(dt)]
    ok = np.isfinite(ep)
    w = ok & (tw <= FIRST_WINDOW_S)
    sent = np.abs(arr(L.dq_sent_deg, 7)[:, ACTIVE_JOINTS])
    safe = np.degrees(np.abs(arr(L.dq_safe, 7)[:, ACTIVE_JOINTS]))
    msk = safe > 1e-3
    eff = float(np.median(sent[msk] / safe[msk])) if msk.any() else float('nan')
    durs = {}
    for k in ('dur_feedback', 'dur_fk', 'dur_obs', 'dur_agent', 'dur_pipeline', 'dur_send', 'dur_log', 'dur_wait'):
        v = arr(getattr(L, k)).ravel()
        v = v[np.isfinite(v)]
        durs[k] = float(np.median(v)) * 1e3 if v.size else float('nan')
    stop = str(meta.stopReason)
    return {
        'file': os.path.relpath(path, CAMPAIGN).replace('\\', '/'),
        'condId': str(meta.condId), 'repetition': int(meta.repetition), 'dry': bool(meta.dryRun),
        'agentLabel': str(meta.agentLabel), 'agentSampleTime': float(meta.agentSampleTime),
        'rateHz': float(meta.rateHz), 'pathDuration': float(meta.pathDuration),
        'cmdScale': float(meta.cmdScale), 'effFactor': eff, 'referenceTiming': str(meta.referenceTiming),
        'scriptVersion': str(meta.scriptVersion), 'gitHash': str(meta.gitHash), 'gitDirty': bool(meta.gitDirty),
        'stopReason': stop, 'completed': stop == 'completed', 'nSteps': int(len(tw)),
        'tEnd': float(tw[-1]), 'rms': float(np.sqrt(np.mean(ep[ok] ** 2))) if ok.any() else float('nan'),
        'rms_first': float(np.sqrt(np.mean(ep[w] ** 2))) if w.any() else float('nan'),
        'max': float(np.max(ep[ok])) if ok.any() else float('nan'),
        'loopMean_ms': float(np.mean(dt)) * 1e3 if dt.size else float('nan'),
        'loopMedian_ms': float(np.median(dt)) * 1e3 if dt.size else float('nan'),
        'anyCap': bool(np.any(arr(L.cap_active, 7))), 'anySoftLimit': bool(np.any(arr(L.softlimit_active, 7))),
        **{f'{k}_ms': v for k, v in durs.items()},
        '_dt': dt,
    }


def timing_metrics(path, meta, cfg, L):
    dt = arr(L.dt_loop).ravel()
    dt = dt[np.isfinite(dt)]
    row = {'file': os.path.relpath(path, CAMPAIGN).replace('\\', '/'), 'condId': str(meta.condId),
           'repetition': int(meta.repetition), 'mode': str(meta.mode), 'dry': bool(meta.dryRun),
           'nCycles': int(dt.size + 1), 'median_ms': float(np.median(dt)) * 1e3,
           'mean_ms': float(np.mean(dt)) * 1e3, 'p95_ms': float(np.percentile(dt, 95)) * 1e3,
           'gitHash': str(meta.gitHash), '_dt': dt}
    for k in ('dur_send', 'dur_feedback', 'dur_fk', 'dur_obs', 'dur_agent', 'dur_pipeline', 'dur_log'):
        v = arr(getattr(L, k)).ravel()
        v = v[np.isfinite(v)]
        row[f'{k}_ms'] = float(np.median(v)) * 1e3 if v.size else float('nan')
    return row


def setpoint_kpis(t, d, P, tol=SP_TOL, hold=SP_HOLD):
    """Definitionen wie hardware/analysis/evaluate_p2p_hardware.m (Table VIII)."""
    k = {'d0': float(d[0]), 'final': float(d[-1]), 'closest': float(np.min(d)), 'success': bool(d[-1] < tol)}
    k['settle'] = float('nan')
    idx = np.flatnonzero(d < tol)
    for i0 in idx:
        msk = (t >= t[i0]) & (t <= min(t[i0] + hold, t[-1]))
        if np.all(d[msk] < tol) and (t[-1] - t[i0]) >= hold:
            k['settle'] = float(t[i0] - t[0])
            break
    k['pathLen'] = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))) if len(P) > 1 else 0.0
    k['pathEff'] = min(1.0, k['d0'] / k['pathLen']) if k['pathLen'] > 1e-6 else 0.0
    k['overshoot'] = max(0.0, float(np.max(d[idx[0]:])) - tol) if idx.size else 0.0
    return k


def setpoint_metrics(path, meta, cfg, L):
    d = arr(L.ep_norm).ravel()
    t = arr(L.t_wall).ravel()
    P = arr(L.ee_pos, 3)
    ok = np.isfinite(d)
    k = setpoint_kpis(t[ok], d[ok], P[ok])
    dt = arr(L.dt_loop).ravel()[1:]
    dt = dt[np.isfinite(dt)]
    sent = np.abs(arr(L.dq_sent_deg, 7))
    safe = np.degrees(np.abs(arr(L.dq_safe, 7)))
    msk = safe > 1e-3
    res = np.linalg.norm(arr(L.ee_kortex_urdf, 3) - arr(L.ee_fk, 3), axis=1)
    res = res[np.isfinite(res)]
    soft = np.asarray(arr(L.softlimit_active, 7), bool)
    qdeg = (np.degrees(arr(L.q, 7)) + 180.0) % 360.0 - 180.0
    at_hw = np.nanmax(np.abs(qdeg), axis=0) >= HW_LIMIT_DEG - 0.5
    stop = str(meta.stopReason)
    return {
        'file': os.path.relpath(path, CAMPAIGN).replace('\\', '/'),
        'condId': str(meta.condId), 'startId': str(meta.startId), 'repetition': int(meta.repetition),
        'dry': bool(meta.dryRun), 'agentLabel': str(meta.agentLabel), 'cmdScale': float(meta.cmdScale),
        'eeSource': str(meta.eeSource), 'maxDuration': float(meta.maxDuration),
        'scriptVersion': str(meta.scriptVersion), 'gitHash': str(meta.gitHash), 'gitDirty': bool(meta.gitDirty),
        'stopReason': stop, 'converged': stop == 'converged', 'nSteps': int(len(t)), 'tEnd': float(t[-1]),
        'd0List_m': float(meta.d0List_m), 'd0_m': k['d0'], 'final_m': k['final'], 'closest_m': k['closest'],
        'success50': k['success'], 'settle50_s': k['settle'], 'pathLen_m': k['pathLen'], 'pathEff': k['pathEff'],
        'overshoot_m': k['overshoot'],
        'effFactor': float(np.median(sent[msk] / safe[msk])) if msk.any() else float('nan'),
        'loopMedian_ms': float(np.median(dt)) * 1e3 if dt.size else float('nan'),
        'kortexResidualRms_m': float(np.sqrt(np.mean(res ** 2))) if res.size else float('nan'),
        'minHeight_m': float(np.nanmin(arr(L.min_height))),
        'anyCap': bool(np.any(arr(L.cap_active, 7))), 'anySoftLimit': bool(soft.any()),
        'softLimitFrac': float(soft.any(axis=1).mean()),
        'softLimitJoints': ' '.join(f'J{j + 1}' for j in np.flatnonzero(soft.any(axis=0))),
        'hwLimitJoints': ' '.join(f'J{j + 1}' for j in np.flatnonzero(at_hw)),
    }


def collect(include_dry):
    pattern = [os.path.join(CAMPAIGN, '*', '*.mat')]
    if include_dry:
        pattern.append(os.path.join(CAMPAIGN, '_dryrun', '*', '*.mat'))
    tracking, timing, setpoint = [], [], []
    for pat in pattern:
        for p in sorted(glob.glob(pat)):
            if os.sep + 'results' + os.sep in p:
                continue
            schema, meta, cfg, L = load_run(p)
            if schema == 'sk_campaign_v1':
                tracking.append(tracking_metrics(p, meta, cfg, L))
            elif schema == 'sk_campaign_timing_v1':
                timing.append(timing_metrics(p, meta, cfg, L))
            elif schema == 'sk_campaign_setpoint_v1':
                setpoint.append(setpoint_metrics(p, meta, cfg, L))
    return tracking, timing, setpoint


def write_csv(rows, path):
    if not rows:
        return
    keys = [k for k in rows[0] if not k.startswith('_')]
    with open(path, 'w', newline='', encoding='utf8') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)


def group(rows, key='condId'):
    g = {}
    for r in rows:
        g.setdefault(r[key], []).append(r)
    return g


def table4(tracking, out, numbers):
    lines = ['% Automatisch erzeugt von evaluation/campaign/analyze_campaign.py. Nicht von Hand aendern.',
             '% Spalten: Agent (Trainingsrate) & Schleife & Bahn & Laeufe & Faktor & Abgeschlossen &',
             '%          RMS [m] (abgeschlossen) & RMS erste 3.4 s [m] (alle) & Max [m] (abgeschlossen) & Loop-Zeit']
    for cid, rs in sorted(group(tracking).items()):
        done = [r for r in rs if r['completed']]
        rms_s, rms_m, rms_sd = mean_sd([r['rms'] for r in done])
        first_s, first_m, _ = mean_sd([r['rms_first'] for r in rs])
        max_s, max_m, _ = mean_sd([r['max'] for r in done])
        loop = st.median([r['loopMean_ms'] for r in rs])
        eff = sorted({round(r['effFactor'], 2) for r in rs})
        eff_s = f'{eff[0]:.2f}' if len(eff) == 1 else f'{eff[0]:.2f}--{eff[-1]:.2f}'
        stops = {}
        for r in rs:
            stops[r['stopReason']] = stops.get(r['stopReason'], 0) + 1
        r0 = rs[0]
        rate_train = 1.0 / r0['agentSampleTime']
        dry = ' (DRY)' if any(r['dry'] for r in rs) else ''
        lines.append(f'% {cid}{dry}: Stoppgruende {stops}')
        lines.append(f'{r0["agentLabel"]}{dry} ({rate_train:.0f}~Hz) & {r0["rateHz"]:.0f}~Hz & '
                     f'\\SI{{{r0["pathDuration"]:g}}}{{\\second}} & {len(rs)} & {eff_s} & {len(done)}/{len(rs)} & '
                     f'{rms_s} & {first_s} & {max_s} & \\SI{{{loop:.0f}}}{{\\milli\\second}} \\\\')
        files = ';'.join(r['file'] for r in rs)
        for key, val in [('runs', len(rs)), ('completed', len(done)), ('rms_mean', rms_m), ('rms_sd', rms_sd),
                         ('rms_first_mean', first_m), ('max_mean', max_m), ('loop_mean_ms_median', loop)]:
            numbers.append({'table': 'IV', 'condId': cid, 'quantity': key, 'value': val, 'source': files})
    with open(os.path.join(out, 'table4_tracking.tex'), 'w', encoding='utf8') as f:
        f.write('\n'.join(lines) + '\n')


def table3(timing, tracking, out, numbers):
    lines = ['% Automatisch erzeugt von evaluation/campaign/analyze_campaign.py. Nicht von Hand aendern.',
             '% Spalten: Konfiguration & Median dt & effektive Rate (Median ueber alle Zyklen aller Wiederholungen)']
    breakdown = []
    g = group(timing)
    for cid in TIMING_ORDER + sorted(set(g) - set(TIMING_ORDER)):
        if cid not in g:
            continue
        rs = g[cid]
        dt = np.concatenate([r['_dt'] for r in rs])
        med = float(np.median(dt)) * 1e3
        dry = ' (DRY)' if any(r['dry'] for r in rs) else ''
        lines.append(f'{TIMING_LABEL.get(rs[0]["mode"], rs[0]["mode"])}{dry} & \\SI{{{med:.1f}}}{{\\milli\\second}} & '
                     f'\\SI{{{1e3 / med:.1f}}}{{\\hertz}} \\\\')
        numbers.append({'table': 'III', 'condId': cid, 'quantity': 'median_ms', 'value': med,
                        'source': ';'.join(r['file'] for r in rs)})
        breakdown.append({'condId': cid, 'mode': rs[0]['mode'], 'repetitions': len(rs), 'median_dt_ms': med,
                          **{k: st.median([r[k] for r in rs]) for k in rs[0] if k.startswith('dur_')}})
    # Geschlossene Schleife aus echten Tracking-Laeufen, je Bedingung
    for cid, rs in sorted(group(tracking).items()):
        dt = np.concatenate([r['_dt'] for r in rs])
        med = float(np.median(dt)) * 1e3
        dry = ' (DRY)' if any(r['dry'] for r in rs) else ''
        lines.append(f'% Tracking {cid}{dry}: Median dt {med:.1f} ms ({1e3 / med:.1f} Hz) ueber {len(rs)} Laeufe')
        numbers.append({'table': 'III', 'condId': cid, 'quantity': 'tracking_median_ms', 'value': med,
                        'source': ';'.join(r['file'] for r in rs)})
        breakdown.append({'condId': cid, 'mode': 'tracking', 'repetitions': len(rs), 'median_dt_ms': med,
                          **{k: st.median([r[k] for r in rs]) for k in rs[0] if k.startswith('dur_')}})
    with open(os.path.join(out, 'table3_looprate.tex'), 'w', encoding='utf8') as f:
        f.write('\n'.join(lines) + '\n')
    write_csv(breakdown, os.path.join(out, 'table3_breakdown.csv'))


COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']   # Referenzpalette, validiert
INK = '#52514e'


def pyplot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'pdf.fonttype': 42, 'ps.fonttype': 42, 'font.family': 'Arial', 'font.size': 8,
                         'mathtext.fontset': 'custom', 'mathtext.rm': 'Arial', 'mathtext.it': 'Arial:italic',
                         'axes.linewidth': 0.6, 'axes.edgecolor': INK, 'xtick.color': INK,
                         'ytick.color': INK})
    return plt


def fig7(tracking, out):
    plt = pyplot()
    colors = COLORS
    fig, ax = plt.subplots(figsize=(3.45, 2.1))
    g = group(tracking)
    all_dt = np.concatenate([r['_dt'] for rs in g.values() for r in rs]) * 1e3 if g else np.array([0.0])
    bins = np.arange(0, max(200.0, float(np.max(all_dt)) + 5), 2.5)
    for i, (cid, rs) in enumerate(sorted(g.items())):
        dt = np.concatenate([r['_dt'] for r in rs]) * 1e3
        label = f'{rs[0]["agentLabel"]}, {len(rs)} runs' + (' (DRY)' if any(r['dry'] for r in rs) else '')
        ax.hist(dt, bins=bins, histtype='step', lw=1.4, color=colors[i % len(colors)], label=label)
    for x, txt in [(25, '25 ms (40 Hz)'), (100, '100 ms (10 Hz)')]:
        ax.axvline(x, color='#52514e', lw=0.6, ls=':')
        ax.text(x + 1.5, ax.get_ylim()[1] * 0.95, txt, fontsize=7, va='top', color='#52514e')
    ax.set_xlabel('Loop time $\\Delta t$ [ms]')
    ax.set_ylabel('Steps')
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.legend(frameon=False, fontsize=7, loc='upper right')
    fig.tight_layout(pad=0.3)
    fig.savefig(os.path.join(out, 'fig7_loop_histogram.pdf'))
    plt.close(fig)


def table_setpoint(setpoint, out, numbers):
    lines = ['% Automatisch erzeugt von evaluation/campaign/analyze_campaign.py. Nicht von Hand aendern.',
             '% Spalten: Start & d0 [mm] & Laeufe & Erfolg (Endfehler < 50 mm) & Endfehler [mm] &',
             '%          naechster Abstand [mm] & Setzzeit [s] (erfolgreiche Laeufe) & Pfadeffizienz']
    for cid, rs in sorted(group(setpoint).items()):
        dry = ' (DRY)' if any(r['dry'] for r in rs) else ''
        ok = [r for r in rs if r['success50']]
        lines.append(f'% {cid}{dry}: {len(ok)}/{len(rs)} Laeufe unter 50 mm, Faktor '
                     f'{sorted({r["cmdScale"] for r in rs})}, Regelpunkt {sorted({r["eeSource"] for r in rs})}')
        by_start = group(rs, 'startId')
        for sid in sorted(by_start, key=lambda s: st.mean(r['d0_m'] for r in by_start[s])):
            g = by_start[sid]
            succ = [r for r in g if r['success50']]
            d0 = st.mean(r['d0_m'] for r in g) * 1e3
            fin = st.mean(r['final_m'] for r in g) * 1e3
            clo = st.mean(r['closest_m'] for r in g) * 1e3
            settle = [r['settle50_s'] for r in succ if np.isfinite(r['settle50_s'])]
            settle_s = f'{st.mean(settle):.1f}' if settle else '--'
            eff = st.mean(r['pathEff'] for r in g)
            lines.append(f'{sid}{dry} & {d0:.0f} & {len(g)} & {len(succ)}/{len(g)} & {fin:.1f} & {clo:.1f} & '
                         f'{settle_s} & {eff:.2f} \\\\')
            files = ';'.join(r['file'] for r in g)
            for key, val in [('d0_mm', d0), ('final_mm_mean', fin), ('closest_mm_mean', clo),
                             ('successes', len(succ)), ('runs', len(g))]:
                numbers.append({'table': 'setpoint_hw', 'condId': f'{cid}/{sid}', 'quantity': key, 'value': val,
                                'source': files})
        numbers.append({'table': 'setpoint_hw', 'condId': cid, 'quantity': 'success_rate_50mm',
                        'value': len(ok) / len(rs), 'source': ';'.join(r['file'] for r in rs)})
    with open(os.path.join(out, 'table_setpoint_hw.tex'), 'w', encoding='utf8') as f:
        f.write('\n'.join(lines) + '\n')


def fig_setpoint(setpoint, out):
    plt = pyplot()
    fig, ax = plt.subplots(figsize=(3.45, 2.3))
    for i, (cid, rs) in enumerate(sorted(group(setpoint).items())):
        c = COLORS[i % len(COLORS)]
        dry = ' (DRY)' if any(r['dry'] for r in rs) else ''
        d0 = np.array([r['d0_m'] for r in rs])
        fin = np.array([r['final_m'] for r in rs]) * 1e3
        suc = np.array([r['success50'] for r in rs])
        ax.scatter(d0[suc], fin[suc], s=22, color=c, edgecolors='white', linewidths=0.8, zorder=3,
                   label=f'{cid}{dry}, {int(suc.sum())}/{len(rs)} < 50 mm')
        if (~suc).any():
            ax.scatter(d0[~suc], fin[~suc], s=22, facecolors='white', edgecolors=c, linewidths=1.2, zorder=3)
    ax.axhline(SP_TOL * 1e3, color=INK, lw=0.6, ls=':')
    ax.text(0.99, SP_TOL * 1e3 * 1.08, 'tolerance 50 mm', fontsize=7, color=INK, ha='right',
            transform=ax.get_yaxis_transform())
    fin_all = np.array([r['final_m'] for r in setpoint]) * 1e3
    ax.set_yscale('log')
    ax.set_ylim(min(1.0, 0.7 * fin_all.min()), max(200.0, 1.5 * fin_all.max()))
    ticks = [t for t in (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000) if ax.get_ylim()[0] <= t <= ax.get_ylim()[1]]
    ax.set_yticks(ticks)
    ax.set_yticklabels([f'{t:g}' for t in ticks])
    ax.minorticks_off()
    ax.set_xlabel('Start distance $d_0$ [m]')
    ax.set_ylabel('Final error [mm]')
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.legend(frameon=False, fontsize=7, loc='upper left')
    fig.tight_layout(pad=0.3)
    fig.savefig(os.path.join(out, 'fig_setpoint_d0.pdf'))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--include-dry', action='store_true', help='Trockenlaeufe mit auswerten (nur zum Testen)')
    ap.add_argument('--out', default=os.path.join(CAMPAIGN, 'results'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    tracking, timing, setpoint = collect(a.include_dry)
    print(f'{len(tracking)} Tracking-Laeufe, {len(timing)} Timing-Messungen, {len(setpoint)} Set-Point-Laeufe gefunden')
    for r in tracking + setpoint:
        if r['gitDirty']:
            print(f'  Hinweis: {r["file"]} lief mit uncommittetem Code ({r["gitHash"]})')
    numbers = []
    write_csv(tracking, os.path.join(a.out, 'runs_tracking.csv'))
    write_csv(timing, os.path.join(a.out, 'runs_timing.csv'))
    write_csv(setpoint, os.path.join(a.out, 'runs_setpoint.csv'))
    if tracking:
        table4(tracking, a.out, numbers)
        fig7(tracking, a.out)
    if timing or tracking:
        table3(timing, tracking, a.out, numbers)
    if setpoint:
        table_setpoint(setpoint, a.out, numbers)
        fig_setpoint(setpoint, a.out)
    write_csv(numbers, os.path.join(a.out, 'paper_numbers.csv'))
    print(f'Ergebnisse in {a.out}')


if __name__ == '__main__':
    main()
