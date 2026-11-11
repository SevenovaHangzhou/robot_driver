"""Read-only display of startup selection and existing RT-reported health interfaces."""
import math
from pathlib import Path
import threading
import time
import yaml


class ModuleStatus:
    def __init__(self, generated_dir, feedback_timeout_s):
        path = Path(generated_dir) / 'module_selection.yaml'
        self.rows = yaml.safe_load(path.read_text())['modules'] if generated_dir and path.is_file() else []
        self._expected = {name for row in self.rows if row['selected'] for name in row['health_interfaces']}
        self._samples = {}
        self._lock = threading.Lock()
        self._timeout = feedback_timeout_s

    def observe(self, message, now=None):
        stamp = time.monotonic() if now is None else now
        samples = {}
        for name, fields in zip(message.joint_names, message.interface_values):
            for field, value in zip(fields.interface_names, fields.values):
                key = f'{name}/{field}'
                if key in self._expected:
                    samples[key] = (float(value), stamp)
        with self._lock:
            self._samples.update(samples)

    def snapshot(self, now=None):
        stamp = time.monotonic() if now is None else now
        with self._lock:
            samples = dict(self._samples)
        rows = []
        for row in self.rows:
            item = {key: row[key] for key in ('module', 'component', 'label', 'policy', 'present', 'selected', 'reason', 'domain')}
            item['option'] = row.get('option', '')
            item['communication'] = 'not_loaded'
            item['age_ms'] = None
            if row['selected']:
                names = row['health_interfaces']
                current = {name: samples[name][0] for name in names if name in samples
                           and stamp - samples[name][1] <= self._timeout}
                complete = bool(names) and len(current) == len(names) and all(math.isfinite(v) for v in current.values())
                item['communication'] = 'waiting'
                if complete:
                    valid = all(value == 1.0 for name, value in current.items() if name.endswith('/data_valid'))
                    fault = any(value != 0.0 for name, value in current.items() if name.endswith('/fault_latched'))
                    ages = [value for name, value in current.items() if name.endswith(('/process_data_age_ms', '/feedback_age_ms'))]
                    item['age_ms'] = max(ages) if ages else None
                    age_invalid = any(age < 0 or age > self._timeout * 1000 for age in ages)
                    item['communication'] = 'reported_invalid' if not valid or fault or age_invalid else 'reported_valid'
            rows.append(item)
        return rows
