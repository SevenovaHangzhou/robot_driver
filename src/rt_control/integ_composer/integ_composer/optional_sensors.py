"""Compatibility adapter for the existing head and wrist-force command options."""
from .startup_selection import write_startup_artifact


def write_sensor_artifact(output, *, workspace, scan=None, sensors='auto', head='required',
                          head_probe=None, **selection):
    result, report = write_startup_artifact(output, workspace=workspace, scan=scan,
        choices={'wrist_force': sensors, 'head': head}, head_probe=head_probe, **selection)
    # Keep the existing launcher/selection report fields while consumers migrate.
    sensors_rows = [row for row in report['modules'] if row['option'] == 'wrist_force']
    head_rows = [row for row in report['modules'] if row['option'] == 'head']
    report.update(requested=sensors,
        selected=sorted(row['component'] for row in sensors_rows if row['selected']),
        absent=[row['component'] for row in sensors_rows if row['reason'] == 'absent'],
        head={'requested': head, 'selected': [row['module'] for row in head_rows if row['selected']],
              'modules': {row['module']: row.get('detail', {}) for row in head_rows}})
    from pathlib import Path
    import json
    (Path(workspace) / 'selection.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    return result, report
