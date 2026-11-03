// Pure validation: loading a preset never sends a robot command.
const leftJoints = Array.from({ length: 7 }, (_, index) => `left_joint${index + 1}`);
const rightJoints = Array.from({ length: 7 }, (_, index) => `right_joint${index + 1}`);
const jointScopes = [leftJoints, rightJoints, [...leftJoints, ...rightJoints]];
const validScope = (keys) => jointScopes.some((scope) =>
  keys.length === scope.length && scope.every((key) => keys.includes(key)));

export function preparePreset(catalog, id, poses, status, gears) {
  if (status && (status.sequence || status.moving)) throw new Error("运动或序列执行中，不能载入预设");
  if (!catalog || catalog.schema_version !== 1) throw new Error("序列预设版本无效");
  const preset = catalog.presets.find((item) => item.id === id);
  if (preset && preset.blocked_reason) throw new Error(preset.blocked_reason);
  if (!preset || !Array.isArray(preset.names) || !preset.names.length || preset.names.length > 50) {
    throw new Error("序列预设不存在或超出 50 个位姿");
  }
  const dwell = preset.dwell_s ?? 0;
  if (dwell !== 0 && dwell !== 2 && dwell !== 8) throw new Error("预设停留时间无效");
  if (!gears.some((gear) => gear.name === preset.gear)) throw new Error("当前操作台没有预设要求的档位");
  const available = new Map(poses.map((pose) => [pose.name, pose.joints]));
  for (const name of preset.names) {
    const actual = available.get(name);
    const expected = catalog.expected_poses[name];
    if (!actual || !expected) throw new Error(`缺少位姿「${name}」，请重新核对预设`);
    const keys = Object.keys(actual);
    // Validate every commanded joint. A dual-arm reference can also validate a
    // single-arm console, but a seven-joint reference cannot approve fourteen.
    if (!validScope(Object.keys(expected)) || !validScope(keys) ||
        !keys.every((key) => Number.isFinite(expected[key]) && actual[key] === expected[key])) {
      throw new Error(`位姿「${name}」已变化，请重新核对预设`);
    }
  }
  return { names: [...preset.names], gear: preset.gear, dwell_s: dwell };
}
