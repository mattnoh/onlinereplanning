"""Selection with the repaired SCP grid: DDP rows unchanged, SCP rows re-selected.

SCP rows are selected from the repaired grid by the unchanged declared rule of
point_mass_2x2_paper_figure.select (tier, then lowest objective, then wall time).
Usage: python point_mass_2x2_select_repaired.py OLD_SELECTION SCP_GRID OUT_SELECTION
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from point_mass_2x2_paper_figure import load, select  # noqa: E402


def main(old, grid, out):
    rows = [r for r in json.loads(old.read_text()) if r['solver'] != 'scp']
    cells = load(grid)
    for k in sorted(cells):
        if k[1] != 'scp': continue
        c = select(cells[k]); au = c['audit']; po = c['policy']
        rows.append(dict(mode=k[0], solver=k[1], arm=k[2], N=k[3], tier=c['tier'], profile=c['profile'], run=c['run'],
                         candidates=len(cells[k]), accepted_candidates=sum(x['audit']['status']['accepted'] for x in cells[k]),
                         native_status=po['native_status'], native_iterations=po['native_iterations'],
                         objective=au['objective'], final_time_s=au['final_time_s'],
                         obstacle_clearance_mm=-au['max_obstacle_penetration_upper_m']*1e3,
                         glide_margin_mm=-au['max_glide_excess_upper_m']*1e3,
                         max_integral_over_B=au['max_integral_over_B'], physical=au['status']['physical'],
                         integral=au['status']['integral'], integral_theorem14=au['status'].get('integral_theorem14'),
                         terminal_error_inf=au['terminal_error_inf'], wall_seconds_in_grid=po['wall_seconds']))
    rows.sort(key=lambda r: (r['mode'], r['solver'], r['arm'], r['N']))
    out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(rows, indent=1)+'\n')
    for r in rows:
        if r['solver'] == 'scp':
            print(r['mode'], r['arm'], r['N'], 'tier', r['tier'], r['profile'], '%d/%d' % (r['accepted_candidates'], r['candidates']),
                  'it', r['native_iterations'], 'J %.6f' % r['objective'], 'clr %.2f' % r['obstacle_clearance_mm'], '%.2fs' % r['wall_seconds_in_grid'])


if __name__ == '__main__':
    main(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
