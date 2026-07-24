# rt-control host real-time boot configuration

This record is specific to host `ar-Default-string` (`192.168.0.40`) as probed
on 2026-07-24. Do not copy its CPU numbers to another machine.

## Frozen host selection

- Kernel: Ubuntu Pro `5.15.0-1032-realtime`, matching the original frozen host
  requirement.
- Dedicated physical P-core: core 7, logical CPU 14.
- SMT sibling: CPU 15. The `nosmt` boot option keeps every secondary SMT thread
  offline, so no task can share core 7 with the control loop.
- Container cpuset: `RT_CONTROL_CPUSET=14`.
- Housekeeping/IRQ CPUs: `0-13,15-27`; offline CPUs in that mask are harmless.
- Controller Manager priority: `SCHED_FIFO 80` (the approved provisional value).

Benefit: CPU 14 has a whole P-core and tick/RCU callbacks are moved away from
the control loop. Drawback: `nosmt` removes eight logical SMT threads from all
P-cores, and CPU 14 is unavailable to ordinary host workloads. The remaining
20 physical cores are ample for this host, but this must be reconsidered if
the existing MoveIt/GPU workload later shows CPU saturation.

## Apply once

Run the topology-checking helper:

```bash
sudo hostsetup/grub-rt-apply.sh
```

It preserves the host's existing non-RT boot arguments, removes any prior
`nosmt`/isolation/IRQ arguments, and writes the following effective arguments
through `/etc/default/grub.d/99-rt-control.cfg`:

```text
nosmt isolcpus=domain,managed_irq,14 nohz_full=14 rcu_nocbs=14 irqaffinity=0-13,15-27
```

Then reboot:

```bash
sudo update-grub
sudo reboot
```

This drop-in is reversible: delete only
`/etc/default/grub.d/99-rt-control.cfg`, run `sudo update-grub`, and reboot.
The retained generic kernels remain GRUB fallback entries. The helper does not
change `GRUB_DEFAULT` or remove any kernel.

## Required post-reboot checks

```bash
uname -r
cat /proc/cmdline
cat /sys/kernel/realtime
cat /sys/devices/system/cpu/isolated
cat /sys/devices/system/cpu/nohz_full
cat /sys/devices/system/cpu/offline
grep -R . /proc/irq/*/effective_affinity_list | grep -E '(^|,|-)14($|,|-)'
```

Expected: realtime kernel, `isolated=14`, `nohz_full=14`, CPU 15 among the SMT
offline set, and no device IRQ effectively targeting CPU 14.

Run the production wrapper only with:

```bash
export RT_CONTROL_CPUSET=14
tools/rt_control_compose.sh up -d rt-control
```

For the final commissioning record, run on CPU 14 after all drivers are stable:

```bash
sudo cyclictest --affinity 14 --threads 1 --priority 90 --policy fifo \
  --mlockall --interval 1000 --duration 30m --quiet
```

The frozen stop criterion is maximum latency above `100 us`. This host's RTX
2000 Ada has no observed PCIe AER fault, but final timing must still be repeated
with the representative GPU/MoveIt workload because the proprietary NVIDIA
module taints the realtime kernel and can affect worst-case latency.
