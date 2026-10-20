"""Select per-cell settings by the declared rule and draw the paper 2x2 figures.

Rule (declared before selection): for every (time mode, solver, arm, N), among
the identical-size candidate grid of that solver,
  tier 1: accepted by the common audit and normal native termination
          (DDP Optimal Solution or Outer Max/Min; SCP converged),
  tier 2: accepted by the common audit,
  tier 3: no accepted run -> the cell is reported as a failure;
within a tier take the lowest objective, ties (|dJ| <= 1e-6) broken by wall time.
Accepted means: certified input/speed/time-bound feasibility and terminal error
<= 1e-6 m on the exact replay of the returned controls from x0; CTCS arms must in
addition certify the physical path and every interval budget I_k <= B.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D

from point_mass_2x2_paper import theorem14_tightening

HERE = Path(__file__).resolve().parent
ARMS = (('ddp', 'node'), ('ddp', 'ctcs'), ('scp', 'node'), ('scp', 'ctcs'))
LABEL = {('ddp', 'node'): 'DDP node-only', ('ddp', 'ctcs'): 'DDP CTCS',
         ('scp', 'node'): 'CT-SCvx node-only', ('scp', 'ctcs'): 'CT-SCvx CTCS'}

# Original 2x2 style (plot_point_mass_ctcs_compare_fixed.py).
mpl.rc('font', family='serif', serif=['Computer Modern Roman', 'DejaVu Serif'])
mpl.rcParams.update({'font.size': 14, 'mathtext.fontset': 'cm', 'axes.linewidth': .8,
                     'lines.solid_capstyle': 'round', 'lines.dash_capstyle': 'round'})
C_SURF, C_OBS = '#6f42c1', '#7a7a7a'
COLORS = {('ddp', 'node'): '#eb6834', ('ddp', 'ctcs'): '#2a78d6', ('scp', 'node'): '#b44d22', ('scp', 'ctcs'): '#168aa0'}
CMAP = LinearSegmentedColormap.from_list('pen', ['#e8f1fd', '#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0b3568'])
CMAP_IN = LinearSegmentedColormap.from_list('inside', ['#fdeceb', '#f8cfcc', '#eda49f', '#df766f', '#c9443c', '#a4243b', '#7d1b2c', '#551220'])


def inside_runs(mask):
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(edges[::2], edges[1::2]-1))


def normal_termination(policy):
    # Normal native termination. Per ALIPDDP CLAUDE.md, "Outer Max/Min" is expected
    # solver output (all schedules reached their limits), not an error.
    return policy['native_status'] in ('Optimal Solution', 'Outer Max/Min', 'converged')


def load(grid):
    cells = {}
    for prof in sorted(p for p in grid.iterdir() if p.is_dir()):
        P = json.loads((prof/'protocol.json').read_text())
        for run in prof.iterdir():
            if not run.is_dir() or '.stale' in run.name or not (run/'audit.json').exists(): continue
            mode, solver, arm, n = run.name.split('_'); N = int(n[1:])
            if N not in (6, 10, 20, 40): continue  # N=100 dropped
            a = json.loads((run/'audit.json').read_text()); p = json.loads((run/'policy.json').read_text())
            if not a['status'].get('valid'): continue
            cells.setdefault((mode, solver, arm, N), []).append(dict(profile=prof.name, run=str(run), audit=a, policy=p, protocol=P))
    return cells


def select(cands):
    def tier(c):
        acc = c['audit']['status']['accepted']
        return 1 if acc and normal_termination(c['policy']) else 2 if acc else 3
    best = min(tier(c) for c in cands)
    pool = [c for c in cands if tier(c) == best]
    if best == 3:  # failure: show the least-violating representative, flagged
        return dict(min(pool, key=lambda c: (c['audit']['terminal_error_inf'] > 1e-6, c['audit']['max_integral_over_B'])), tier=3)
    jmin = min(c['audit']['objective'] for c in pool)
    near = [c for c in pool if c['audit']['objective'] <= jmin+1e-6]
    return dict(min(near, key=lambda c: c['policy']['wall_seconds']), tier=best)


def replay(P, pol, step=1e-3):
    x = np.array(P['initial_state'], float); U = np.asarray(pol['U'], float); dt = np.asarray(pol['durations_s'], float)
    o = np.asarray(P['obstacle']['center']); R = P['obstacle']['radius']
    out = []; nodes = [np.r_[0., x]]; t0 = 0.
    for u, d in zip(U, dt):
        a = u-np.array([0., 0., 9.81]); m = max(2, int(np.ceil(d/step)))
        for s in np.linspace(0., d, m, endpoint=False):
            p = x[:3]+s*x[3:]+.5*s*s*a; out.append(np.r_[t0+s, p])
        x = np.r_[x[:3]+d*x[3:]+.5*d*d*a, x[3:]+d*a]; t0 += d; nodes.append(np.r_[t0, x])
    out.append(np.r_[t0, x[:3]]); D = np.array(out); Nd = np.array(nodes)
    def clear(p): return np.linalg.norm(p-o, axis=1)-R
    def glide(p): return np.sqrt(p[:, 0]**2+p[:, 1]**2+1e-12)-P['glide_tan']*p[:, 2]
    return D, clear(D[:, 1:4]), glide(D[:, 1:4]), Nd, clear(Nd[:, 1:4]), glide(Nd[:, 1:4])


def figure(sel, mode, mesh, out, timings):
    meshes = sorted({k[3] for k in sel if k[0] == mode})
    P = next(v for k, v in sel.items() if k[0] == mode)['protocol']; o = P['obstacle']['center']; R = P['obstacle']['radius']
    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.08, 1.], height_ratios=[1.65, 1.], wspace=.27, hspace=.28, left=.095, right=.97, top=.85, bottom=.10)
    ax3 = fig.add_subplot(gs[0, 0], projection='3d'); axmesh = fig.add_subplot(gs[1, 0])
    tg = gs[0, 1].subgridspec(2, 1, height_ratios=[1.85, 1.], hspace=.15)
    axcl = fig.add_subplot(tg[0]); axgl = fig.add_subplot(tg[1], sharex=axcl); axt = fig.add_subplot(gs[1, 1]); axt.axis('off')
    uu, vv = np.meshgrid(np.linspace(0, 2*np.pi, 34), np.linspace(0, np.pi, 17))
    ax3.plot_wireframe(o[0]+R*np.cos(uu)*np.sin(vv), o[1]+R*np.sin(uu)*np.sin(vv), o[2]+R*np.cos(vv), color=C_OBS, lw=.4, alpha=.5)
    th, zz = np.meshgrid(np.linspace(0, 2*np.pi, 34), np.linspace(0, 10.5, 25))
    ax3.plot_wireframe(zz*np.cos(th), zz*np.sin(th), zz, color=C_SURF, ls=':', lw=.5, alpha=.3, rstride=4, cstride=4)
    handles, rows, curves = [], [], []
    for arm in ARMS:
        c = sel.get((mode, *arm, mesh))
        if c is None: continue
        D, cl, gl, Nd, ncl, ngl = replay(P, c['policy']); color = COLORS[arm]
        ls = '-' if arm[0] == 'ddp' else '--'; mk = 'o' if arm[1] == 'node' else 's'
        lab = LABEL[arm]+('' if c['tier'] < 3 else ' (failed)')
        handles.append(Line2D([], [], color=color, lw=2.4, ls=ls, marker=mk, mec='black', label=lab))
        ax3.plot(*D[:, 1:4].T, color=color, lw=2.5, ls=ls)
        for a, b in inside_runs(cl < 0.): ax3.plot(*D[a:b+1, 1:4].T, color=color, lw=6)
        axcl.plot(D[:, 0], cl*1e3, color=color, lw=2., ls=ls); axgl.plot(D[:, 0], gl*1e3, color=color, lw=1.8, ls=ls)
        axcl.scatter(Nd[:, 0], ncl*1e3, s=35, marker=mk, facecolor=color, edgecolor='black', lw=.6, zorder=5)
        axgl.scatter(Nd[:, 0], ngl*1e3, s=28, marker=mk, facecolor=color, edgecolor='black', lw=.6, zorder=5)
        curves.append((D[:, 0], cl*1e3, Nd[:, 0], ncl*1e3, color, mk, ls))
        a = c['audit']; budget = '—' if arm[1] == 'node' else f"{a['max_integral_over_B']:.2f}B" + ('' if a['status'].get('integral_theorem14') else ' FAIL')
        tm = timings.get(f'{mode}_{arm[0]}_{arm[1]}_N{mesh}')
        rows.append([lab, f"{cl.min()*1e3:+.2f}", budget, f"{a['objective']:.6f}",
                     f"{a['final_time_s']:.3f}", '—' if tm is None else f'{tm*1e3:.1f}'])
    p0, pf = P['initial_state'][:3], P['terminal_state'][:3]
    ax3.scatter([p0[0], pf[0]], [p0[1], pf[1]], [p0[2], pf[2]], s=100, marker='*', color='black')
    ax3.set(xlim=(-2.75, 7.75), ylim=(-2.75, 7.75), zlim=(0, 10.5), xlabel='x [m]', ylabel='y [m]', zlabel='z [m]')
    ax3.set_box_aspect((1, 1, 1)); ax3.view_init(elev=9, azim=-58)
    ax3.set_title(f'(a) exact open-loop replay, N={mesh}', loc='left', fontsize=15)
    t_end = max(cv[0][-1] for cv in curves); hi = max(cv[1].max() for cv in curves)*1.05
    lo = min(-45., min(cv[1].min() for cv in curves)*1.2)
    axcl.axhspan(lo, 0., color='.90'); axcl.axhline(0., color=C_SURF, lw=1.2, ls='--')
    axcl.set(xlim=(0, t_end), ylim=(lo, hi), ylabel='obstacle clearance [mm]'); axcl.tick_params(labelbottom=False)
    axcl.set_title('(b) continuous replay and shooting-node markers', loc='left', fontsize=15)
    contacts = [cv[0][np.argmin(cv[1])] for cv in curves]; mins = [cv[1].min() for cv in curves]
    axz = axcl.inset_axes([.46, .48, .51, .46]); zlo = min(-5., min(mins)*1.3); zhi = max(15., max(mins)*1.4)
    axz.axhspan(zlo, 0., color='.90'); axz.axhline(0., color=C_SURF, ls='--', lw=.8)
    for tt, cl, nt, nv, c, mk, ls in curves:
        axz.plot(tt, cl, color=c, lw=1.6, ls=ls); axz.scatter(nt, nv, color=c, marker=mk, edgecolor='black', s=24, lw=.5)
    axz.set(xlim=(max(0., min(contacts)-.35), min(t_end, max(contacts)+.35)), ylim=(zlo, zhi)); axz.tick_params(labelsize=9)
    axz.set_title('contact detail [mm]', fontsize=10)
    axgl.axhline(0., color=C_SURF, ls='--', lw=1.2); axgl.set(ylabel='glide excess [mm]', xlabel='physical time [s]')
    for ax in (axcl, axgl): ax.grid(alpha=.2, ls='--')
    img = np.ones((4, len(meshes), 3))*.9; text = {}
    for i, arm in enumerate(ARMS):
        for j, n in enumerate(meshes):
            c = sel.get((mode, *arm, n))
            if c is None: text[i, j] = 'not run'; continue
            # Magnitude from the exact replay (1 ms); the certificate stops refining once it has
            # decided pass/fail, so its residual enclosures are not magnitudes.
            _, rcl, rgl, *_ = replay(P, c['policy']); clr = min(rcl.min(), -rgl.max())*1e3
            if c['tier'] == 3: text[i, j] = 'no accepted\nsetting'; continue
            lvl = np.log10(np.clip(abs(clr), .01, 300)/.01)/np.log10(300/.01)
            img[i, j] = (CMAP_IN if clr < 0 else CMAP)(lvl)[:3]
            tm = timings.get(f'{mode}_{arm[0]}_{arm[1]}_N{n}')
            text[i, j] = f'{clr:+.2f} mm' + ('' if tm is None else f'\n{tm*1e3:.0f} ms') + ('' if c['tier'] == 1 else '\niteration cap')
    axmesh.imshow(img, aspect='auto')
    for (i, j), s in text.items():
        axmesh.text(j, i, s, ha='center', va='center', fontsize=10, color='white' if np.mean(img[i, j]) < .52 else '#1a1a19')
    axmesh.set_xticks(range(len(meshes)), [f'N={n}' for n in meshes], fontsize=12)
    axmesh.set_yticks(range(4), [LABEL[a] for a in ARMS], fontsize=12); axmesh.tick_params(length=0)
    axmesh.set_title('(c) worst obstacle/glide clearance (exact replay) and median serial solve time', loc='left', fontsize=15)
    tb = axt.table(cellText=rows, colLabels=[f'N={mesh}', 'obs. [mm]', 'budget', 'cost', 'tf [s]', 'solve [ms]'],
                   cellLoc='center', colWidths=[.27, .15, .12, .19, .12, .15], bbox=[0., .36, 1., .59])
    tb.auto_set_font_size(False); tb.set_fontsize(11)
    for (r, _), cell in tb.get_celld().items():
        cell.set_edgecolor('.85')
        if r == 0: cell.set_facecolor('.96'); cell.set_text_props(weight='bold')
    axt.set_title('(d) highlighted policies', loc='left', fontsize=15)
    B = P['budget_per_interval']; d = P['tightening_m']['obstacle']
    axt.text(0., .27, f'Every arm: same problem, seed and acceptance audit; per-interval budget B={B:g} (Elango eq. 24c),\n'
             f'{d*1e3:g} mm Theorem-14 tightening (|v|≤{P["speed_max"]:g} m/s). Per-cell settings: 16-candidate grid per solver\n(free time: +9 SCP at the authors’ weight ratios).\n'
             f'Budget = certified max_k I_k; Theorem 14 admits I_k ≤ {min(r["admissible_budget"] for r in theorem14_tightening(P).values())/P["budget_per_interval"]:.2f}B at this margin.\n'
             'Times: median of serial reruns, same machine; C++ DDP vs Python/JAX SCP.', va='top', fontsize=10.5, linespacing=1.6)
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.53, .945), ncol=4, fontsize=12, frameon=True, edgecolor='.7')
    fig.suptitle(f'Point mass, {"fixed final time = 4 s" if mode == "fixed" else "free final time"}: DDP / CT-SCvx × node-only / CTCS', y=.99, fontsize=19)
    fig.text(.095, .025, 'Exact replay of returned controls from x0 (1 ms plotting); acceptance and colour from the certified audit. Grey = obstacle interior; red cells = physical violation.', fontsize=11)
    for ext in ('png', 'pdf'): fig.savefig(out/f'point_mass_2x2_{mode}.{ext}', dpi=180, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--grid', type=Path, required=True); ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--timings', type=Path, default=None, help='timing.json from the serial reruns')
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    cells = load(a.grid); sel = {k: select(v) for k, v in cells.items()}
    timings = json.loads(a.timings.read_text()) if a.timings else {}
    table = []
    for k in sorted(sel, key=lambda k: (k[0], k[1], k[2], k[3])):
        c = sel[k]; au = c['audit']; po = c['policy']
        table.append(dict(mode=k[0], solver=k[1], arm=k[2], N=k[3], tier=c['tier'], profile=c['profile'], run=c['run'],
                          candidates=len(cells[k]), accepted_candidates=sum(x['audit']['status']['accepted'] for x in cells[k]),
                          native_status=po['native_status'], objective=au['objective'], final_time_s=au['final_time_s'],
                          obstacle_clearance_mm=-au['max_obstacle_penetration_upper_m']*1e3, glide_margin_mm=-au['max_glide_excess_upper_m']*1e3,
                          max_integral_over_B=au['max_integral_over_B'], physical=au['status']['physical'], integral=au['status']['integral'], integral_theorem14=au['status'].get('integral_theorem14'),
                          terminal_error_inf=au['terminal_error_inf'], wall_seconds_in_grid=po['wall_seconds']))
    (a.out/'selection.json').write_text(json.dumps(table, indent=1)+'\n')
    for mode, mesh in (('fixed', 6), ('free', 10)):
        if any(k[0] == mode and k[3] == mesh for k in sel): figure(sel, mode, mesh, a.out, timings)
    print(json.dumps([{k: r[k] for k in ('mode', 'solver', 'arm', 'N', 'tier', 'profile', 'accepted_candidates', 'objective', 'obstacle_clearance_mm')} for r in table], indent=0))


if __name__ == '__main__':
    main()
