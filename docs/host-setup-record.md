# Host setup record

## Active target: ar-Default-string (192.168.0.40)

Date: 2026-07-24 CST
Task: T-009
Requirements: REQ-RT-002, REQ-RT-006, REQ-ECAT-008, REQ-CAN-001,
REQ-DEP-002, REQ-DEP-003

The operator moved the rt-control deployment from `alfa-two` to
`ar@192.168.0.40`. Host-specific BQ-090/BQ-092/BQ-098 overrides remain audit
evidence for `alfa-two` only and are not applied to this host. The active host
matches the original frozen kernel and EtherCAT identity:

| Item | Read-only evidence before deployment | Result |
| --- | --- | --- |
| Host | `hostname`, Ubuntu release | `ar-Default-string`, Ubuntu 22.04.5 LTS |
| Kernel | `uname -r`, `/sys/kernel/realtime` | `5.15.0-1032-realtime`, PREEMPT_RT active |
| CPU | `lscpu`, sibling sysfs | i7-14700; 20 cores/28 CPUs; CPU 14/15 are one P-core |
| Existing isolation | `/proc/cmdline` | CPUs 12-15 isolated, SMT still active; replaced by verified CPU-14 whole-core policy at reboot |
| Memory/storage | `free`, `lsblk`, `df` | 32 GiB; 477 GiB SATA SSD; 227 GiB initially free |
| EtherCAT | PCI/sysfs/IgH CLI | I210 MAC `8c:59:3c:14:ff:d3`; 15 PREOP slaves; legacy IgH 1.5.3 |
| Auxiliary I210 | PCI/sysfs | MAC `8c:59:3c:14:ff:d4`, `enp4s0`, not the fieldbus master |
| CAN | `udevadm`, `ip -details` | two CANable 2.0 `gs_usb`; 500 kbit/s; ERROR-ACTIVE; zero bus errors |
| NVIDIA | `nvidia-smi`, kernel journal | RTX 2000 Ada 16 GiB works; no AER/Xid observed |
| Docker | command/package query | not installed before T-009 |

### Active host decisions

- CPU 14 is selected as the production cpuset after confirming the same
  i7-14700 P-core topology used by the approved whole-core policy. `nosmt`
  removes CPU 15 and the other P-core SMT siblings. Benefit: no sibling, IRQ,
  RCU or ordinary scheduler contention on the control core. Drawback: eight
  logical CPUs are removed and a reboot is required.
- Existing `processor.max_cstate=1` and `intel_idle.max_cstate=1` arguments are
  preserved as pre-existing host policy, not introduced by rt-control. Their
  benefit is reduced wake latency; their drawbacks are higher idle power and
  heat. Final acceptance depends on measured latency rather than assuming the
  policy is beneficial.
- The production CAN adapter is bound to `can0` by USB serial
  `004D00675230500720333159`; the existing BMS adapter is bound to `can1` by
  serial `003000265230500720333159`. Benefit: USB enumeration order cannot
  silently swap control and BMS buses. Drawback: both named adapters must be
  present when the naming service runs, otherwise `can0.service` fails closed.
- The existing RTX 2000 Ada remains installed because this host has no observed
  PCIe fault. No `alfa-two` quarantine file or service is installed. Benefit:
  existing display/GPU workloads remain available. Drawback: the proprietary
  NVIDIA 580 module taints PREEMPT_RT, so representative GPU-load timing remains
  part of final commissioning evidence.
- The legacy IgH 1.5.3 tree, modules and configuration plus GRUB, NetworkManager,
  AppArmor and package state were copied to
  `/var/backups/rt-control/predeploy-20260724-2205` before mutation.

### Applied host state and cold-boot evidence

- The Jammy AppArmor parser rejected a pre-existing Cursor-only `userns,`
  rule. The original profile is in the backup above; only that unsupported line
  was commented. `apparmor_parser` and the enabled `apparmor.service` now pass.
- Docker was installed from Docker's signed Jammy repository with key
  fingerprint `9DC858229FC7DD38854AE2D88D81803C0EBFCD88`: Docker CE/CLI
  29.6.2, containerd 2.2.6, Buildx 0.35.0 and Compose 5.3.1. The six
  Docker packages are held, services are enabled, and `ar` was not added to the
  root-equivalent Docker group. `hostsetup/docker-install.sh` pins and verifies
  the same package set.
- IgH was rebuilt from the exact `versions.env` commit
  `2f7f884f1c7d377c02a7d627eb06512126a0e50e` with GCC 11 for the running
  ABI and explicit `--enable-igb --disable-rt-syslog`. The final host CLI and
  module report 1.6.10. After cold boot, master 0 owns I210 MAC
  `8c:59:3c:14:ff:d3`, Link is UP, all 15 positions are PREOP, and the master
  reports zero Tx errors and zero lost frames. Positions 0 and 13 remain
  observed-only hubs. The auxiliary I210 at `...:d4` remains a normal
  `enp4s0` interface even though the upstream `ec_igb` replacement driver also
  binds that PCI function; its existing `/32` management address was preserved.
- The effective boot line contains the pre-existing C-state policy plus
  `nosmt isolcpus=domain,managed_irq,14 nohz_full=14 rcu_nocbs=14
  irqaffinity=0-13,15-27`. CPU 14 is the only isolated/full-nohz CPU, CPU15
  and the other secondary P-core threads are offline, and no effective device
  IRQ affinity contains CPU14. A ten-second sample showed the cumulative CPU14
  interrupt sum unchanged at 11,337; the nonzero boot-era counts are not
  ongoing IRQ routing.
- The serial-binding and can0 units both recovered automatically after cold
  boot. `can0` is the approved adapter at 500 kbit/s, ERROR-ACTIVE, with no CAN
  error/bus-off counters. Six seconds of passive capture received stable 1 Hz
  heartbeats from `0x701`, `0x702`, `0x703` and observed-only `0x714`, all in
  PREOP (`0x7F`). RX `dropped` increased only before a SocketCAN listener was
  attached and remained constant during capture, so it is a no-listener socket
  accounting effect rather than a physical-frame loss. The BMS adapter is
  correctly named `can1` but rt-control deliberately does not configure or
  raise that separate-domain interface.
- The five boot-log lines containing `AER` only say that PCIe root ports enabled
  AER reporting. There is no `PCIe Bus Error` and no NVIDIA Xid; `nvidia-smi`
  reports the RTX 2000 Ada on driver 580.167.08. The kernel is nevertheless
  tainted by the proprietary/unsigned NVIDIA module and the locally built IgH
  modules have no signer. Benefit: the existing GPU remains operational and
  IgH uses the frozen source. Drawback: kernel provenance is not fully signed,
  so final latency evidence must include representative GPU/MoveIt load and a
  kernel update requires an explicit IgH rebuild.
- Replaying the first draft of the Docker installer exposed two maintenance
  hazards: a transient TLS reset while downloading an already-installed key,
  and `apt-get install` upgraded `curl`, `libcurl4` and
  `libcurl4-openssl-dev` from Ubuntu `.23` to `.25`. No Docker or bus state
  changed. The committed installer now reuses an existing key only after exact
  fingerprint validation and uses `--no-upgrade` for prerequisites; a second
  replay completed with zero package changes. The unrelated RealSense apt
  source still warns about missing key `FB0B24895113F120`; it is recorded but
  not changed under rt-control authority.
- Pre-reboot evidence and checksums are retained on the IPC at
  `/var/lib/rt-control/commissioning/t009-pre-reboot-20260724`. The committed
  `hostsetup/verify-host.sh` then passed the realtime/IRQ, systemd, IgH topology,
  CAN identity/heartbeats, Docker version and GPU boot-log checks after reboot.

### Commissioning boundary

T-009 installs and verifies the host, bus enumeration, Docker security fields
and an unpowered/non-enabled rt-control startup only. It does not call
`/rt/enable`, submit FJT goals, publish updown commands or move tracks. Powered
motion remains gated by T-013/T-014 and the frozen safety preconditions.

## Prior target retained for audit: alfa-two

Date: 2026-07-22/23 CST
Task: T-009
Requirements: REQ-RT-002, REQ-RT-006, REQ-ECAT-008, REQ-CAN-001,
REQ-DEP-002, REQ-DEP-003

## Current verified state

| Item | Evidence | Result |
| --- | --- | --- |
| Host | `hostname`, Ubuntu release | `alfa-two`, Ubuntu 22.04.5 LTS |
| Kernel | `uname -r`, kernel config | staged `6.8.0-136-lowlatency`, `CONFIG_PREEMPT=y`, no `CONFIG_PREEMPT_RT` |
| Secure Boot | `mokutil --sb-state` | disabled; platform setup mode |
| CPU topology | `lscpu -e`, `thread_siblings_list` | i7-14700; P-core pairs 0-1 through 14-15; E-cores 16-27 |
| EtherCAT NIC | sysfs, `lspci -nnk` | Intel I210 `8086:1533`, MAC `8c:59:3c:15:01:f8`; current native name `enp2s0` |
| EtherCAT link | `ip -br link` | `NO-CARRIER`; ring not connected, no slave claim made |
| CAN adapter | `ip link`, `lsusb` | no `can0` and no CAN adapter visible |
| NVIDIA | `nvidia-smi`, boot journal, AER counters | both PCI functions quarantined after early sysinit removal, but boot-time AER remains unresolved and blocks production readiness |
| Docker | `docker version`, `docker info` | CE 29.6.2; containerd 2.2.6; Compose 5.3.1; systemd cgroup v2; overlayfs |

The user explicitly approved HWE 6.8 PREEMPT_RT and the observed new I210 MAC.
These supersede only the original T-009 host values (5.15 and
`8c:59:3c:14:ff:d3`); EtherCAT master ID, interface isolation, topology and all
other frozen requirements remain unchanged.

## IgH installation contract

`hostsetup/igh-install.sh` consumes the same `IGH_VERSION` and full
`IGH_COMMIT` as the container. It builds both `ec_igb` (primary) and
`ec_generic` (approved first fallback) for the running kernel, using upstream's
explicit `--enable-igb --with-igb-kernel=6.8` support and GCC 12 matching the
Ubuntu RT kernel. The host prefix is `/usr/local/etherlab`; this is deliberately
the same installed layout used in the container, but the host additionally
installs kernel modules and systemd integration.

The native driver configuration is:

```text
MASTER0_DEVICE="8c:59:3c:15:01:f8"
DEVICE_MODULES="igb"
UPDOWN_INTERFACES=""
```

NetworkManager marks that MAC unmanaged. No IP, DHCP or DDS traffic is allowed
on the EtherCAT port. A kernel update requires rerunning the installer before
booting production because this task does not invent an out-of-tree DKMS
packaging layer for IgH.

Fallback order remains frozen:

1. Change only `DEVICE_MODULES="igb"` to `DEVICE_MODULES="generic"`, then
   restart `ethercat.service`.
2. If stable-1.6 generic also fails, return to the read-only legacy 1.5 path;
   do not alter slave PDO/SDO YAML.

## CPU/GRUB decision

CPU 14 is the dedicated P-core logical CPU; `nosmt` offlines CPU 15 and all
other secondary SMT threads. The exact boot drop-in, verification and rollback
are in `hostsetup/grub-rt.md`. Production compose must use
`RT_CONTROL_CPUSET=14`. Controller Manager uses provisional `SCHED_FIFO 80`.

## Docker installation and deployment boundary

Docker was installed from Docker's signed Ubuntu Jammy apt repository, not the
convenience script. The installation added seven packages and performed no
upgrade or removal: Docker CE/CLI 29.6.2, containerd 2.2.6, Buildx 0.35.0,
Compose 5.3.1, rootless extras and `pigz`. Docker and containerd are enabled and
active. The key fingerprint is
`9DC858229FC7DD38854AE2D88D81803C0EBFCD88`, and apt reports the installed
Docker CE package as the current Jammy stable candidate. The `alfa` account was
deliberately not added to the root-equivalent `docker` group; host operations
use `sudo`. Docker CE/CLI, containerd, Buildx, Compose and rootless extras are
held with `apt-mark`; a maintenance upgrade must explicitly unhold them, run
the deployment checks, then restore the hold. This prevents accidental drift
but also blocks automatic Docker security updates.

Direct access to `download.docker.com` and Docker Hub failed from the IPC. A
temporary SSH reverse forward exposed the already-approved WSL proxy only at
IPC loopback port 18080 for signed apt downloads, then was closed. No proxy is
persisted in apt, Docker, the image or the repository. The validated image was
streamed through a FIFO and loaded without a large intermediate archive:

```text
tag:  rt-control:4e0d586f73c5548f778e5ef889f5854a6c19c604
id:   sha256:69642097ed0079b3c05335aa62c1efa5a987e7a8572bda4b0c0fd083f2ea465f
size: 549109974
```

The committed feature branch at that same SHA is locally checked out under
`/home/alfa/rt-control-deploy/robot`; this was transferred by Git bundle and is
not a remote push. Compose parsing passed. A stopped test container was created
and inspected with CPU 14, host network/IPC, `CAP_SYS_NICE`, `CAP_IPC_LOCK`,
rtprio 98, unlimited memlock, the read-only CycloneDDS bind and
`/dev/EtherCAT0`; it was never started and was removed after inspection. The
host and container both report IgH 1.6.10. Docker/containerd effective CPU
affinity excludes CPU 14, no device IRQ targets CPU 14, and Docker did not
reclaim the I210 from IgH. The only persistent Docker network addition is its
separate `docker0` bridge.

The official firewall warning remains relevant: a future published container
port must be reviewed in the `DOCKER-USER` chain. This deployment currently
publishes no port. The IPC's persistent access path is the active
`reverse-ssh-robot-ipc.service`; the temporary package proxy tunnel is gone.

## Preliminary timing evidence (not final acceptance)

With the host otherwise idle and the approved CPU isolation active, host
rt-tests 2.2-1 / cyclictest 2.20 ran for 30 minutes on CPU 14 with one FIFO90
thread, 1 ms interval and `mlockall`. The unmodified default power policy was
`intel_pstate` + `powersave` + `balance_performance`; C-state and RT throttling
settings were not changed.

```text
T: 0 (12700) P:90 I:1000 C:1800000 Min: 1 Avg: 4 Max: 18 us
```

No scheduler throttling, RCU stall, lockup, AER or kernel warning appeared
during the run. Because this did not include the connected buses and 250 Hz
control loop, it is a preliminary kernel/isolation baseline only. The default
governor remains unchanged to avoid unnecessary heat and power draw.

The final container path was separately exercised for five seconds using the
exact production image, CPU14, FIFO90, production RT capabilities/ulimits, a
read-only bind of the host cyclictest binary and an explicit
`/dev/cpu_dma_latency` device. PM QoS was set to 0 us and the result was
Min 1 / Avg 3 / Max 12 us over 5,000 cycles. The final 30-minute commissioning
run must use that one-off diagnostic container concurrently with the 250 Hz
control empty-run and record the production image ID plus host rt-tests
version; it must not modify the production image or frozen Compose file.

## NVIDIA PREEMPT_RT and PCIe quarantine — HIGH RISK

The Ubuntu 595 proprietary and open DKMS sources both state that PREEMPT_RT is
unsupported and abort their build. A version-scoped host override at
`/etc/dkms/nvidia-595.71.05.conf` uses GCC 12 and
`IGNORE_PREEMPT_RT_PRESENCE=1`. The open modules then built for
`6.8.1-1056-realtime` and both retained generic kernels; module license is
`Dual MIT/GPL`, and `nvidia-smi` identified the RTX A2000 Laptop GPU.

On load, however, PCIe AER immediately entered a sustained correctable-error
storm. Hardware counters reached approximately 80,486,174 correctable errors:
about 80,478,883 `RxErr`, plus `BadTLP` and `BadDLLP`; fatal and non-fatal totals
were zero. The initial module-unload sample was stable for five seconds, but a
post-reboot test disproved the driver-only diagnosis: with no NVIDIA module
loaded, the VGA function remained in D0 at Gen4 x8 and gained 10,414
correctable errors in three seconds; the HDMI audio function was still bound to
`snd_hda_intel`. The physical PCIe path, carrier or power is therefore the
leading fault domain. ASPM was already disabled, so adding `pcie_aspm=off`
would not address the evidence.

The reversible quarantine has two layers. The existing
`/etc/modprobe.d/nvidia-rt-quarantine.conf` blocks the NVIDIA modules. The
enabled `gpu-pcie-quarantine.service` runs
`/usr/local/sbin/rt-control-gpu-pcie-quarantine`, validates the exact NVIDIA
vendor/device IDs at `01:00.1` and `01:00.0`, scans all PCI functions to reject
either target ID at any unexpected BDF, verifies the dedicated upstream root
port `00:01.0=8086:a70d` and rejects any unexpected child on bus 01, then
removes that root port from the running PCI bus. A truly absent/already-removed
root port is an idempotent success. Removing only `01:00.1` and `01:00.0` was
disproved as containment after a physical reseat: despite both endpoint paths
being absent, the root port continued receiving AER messages. Removing the
dedicated root port stopped both AER and log growth. GDM/i915, SSH and EtherCAT
remained active. A reboot enumerates the GPU again, after which the service
reapplies the quarantine. Before removing either layer:

1. power off and inspect/reseat the GPU or its carrier/riser and auxiliary power;
2. inspect BIOS PCIe slot generation and first retest at Gen3 if the physical
   path is sound;
3. remove the quarantine file, regenerate initramfs, and perform a controlled
   module-load test while watching AER counters;
4. only after AER remains stable, compare 30-minute cyclictest idle and under
   representative GPU load.

Benefit: the GPU driver is installed and reproducibly rebuildable, while the
current fault cannot silently degrade the RT loop. Drawback: GPU compute,
GPU-attached display outputs and its HDMI audio are unavailable; the service is
specific to the probed BDFs/root-port identity and deliberately fails rather
than removing an unexpected device. Even after the physical fault is resolved
NVIDIA still does not support this RT-kernel combination.

After the physical reseat and the endpoint-only containment failure, the current
boot accumulated 8,112,545 AER-matching journal lines. Active `syslog` and
`kern.log` reached about 82 GB each and `/var/log` reached 169 GiB. A controlled
two-second PCI rescan enumerated `10de:25b8` and `10de:2291` at Gen4 x8 but still
produced 2,360 AER-matching lines, consistently reporting correctable Physical
Layer `RxErr`; reseating therefore did not repair the link. After root-port
removal stopped growth, the operator-authorized cleanup truncated the two
active files, removed eight AER-inflated rotated `syslog`/`kern.log` files and
vacuumed journal storage (40 MB remained). A second controlled rescan verified
that the revised topology-checking service itself removed the root port with
exit status 0; the short enumeration window produced 1,655 more AER-matching
lines, which were also cleaned. Final `/var/log` usage was 602 MB and the root
filesystem had 371 GB free (17% used); both active log files remained stable
over five seconds. This intentionally discarded historical records mixed into
those files; the material fault evidence is retained here.

A later operator-reported slot/Gen3 change did not produce a Gen3 link in the
live hardware. The cold boot still enumerated the same `00:01.0 -> 01:00.x`
topology and produced 2,462 AER-matching lines before quarantine. A controlled
rescan showed the GPU capability as 16 GT/s x16 but its active link as
**16 GT/s x8**, while the Intel root port capability was 32 GT/s x16 and its
active link was likewise 16 GT/s x8. Thus the tested link remained Gen4. The
short window produced 2,295 AER-matching lines, again correctable Physical
Layer `RxErr`, including reports on both the VGA and HDMI-audio functions. The
root port was re-quarantined immediately. This result only proves that the
reported change did not alter the observed BDF or negotiated generation; it
must not be cited as evidence that Gen3 also fails. A real Gen3 test still
requires `LnkSta: Speed 8GT/s` before measuring AER.

The first persistent-service reboot exposed a startup-order defect: placing the
unit after `systemd-udev-settle` allowed 389,349 AER log lines before/while the
queued kernel work drained, even though the device was eventually removed and
the later root-port counter was stable. That result is explicitly rejected.
The unit was moved to `sysinit.target`, with no default dependencies, waiting
only for root-filesystem remount and running before
`systemd-udev-trigger.service`.

The second reboot proves this is only containment, not a repair. The unit began
about 0.47 seconds into userspace and completed in 107 ms; boot time improved
from roughly 86 to 52 seconds, the GPU functions were absent afterward and the
root-port AER total stayed at 8 over a 15-second sample. Nevertheless that boot
still recorded 127,072 AER log lines, which means the fault is already active
during PCI enumeration/early kernel work. No credible user-space ordering can
eliminate it. Keep the early service enabled to stop the sustained storm, but
do not classify this host as GPU/RT production-ready until a powered-off
carrier/connection/power inspection or BIOS Gen3 test passes. `pci=noaer`
remains forbidden because it would hide rather than repair the link fault.

On 2026-07-24 the operator removed NVIDIA GPU capability from the rt-control
IPC deployment scope. No further Gen3, driver-load, CUDA or GPU-load acceptance
is required for this host. Until the card is physically removed, both
quarantine layers remain enabled and the host may be used only for controlled
bus bring-up after the root port is confirmed absent; the final 30-minute RT
acceptance still requires a cold boot with the faulty GPU physically absent and
zero NVIDIA-link AER. Benefit: the known PCIe storm cannot consume RT or log
resources and the unsupported NVIDIA/PREEMPT_RT combination is eliminated.
Drawback: this IPC provides no CUDA, NVIDIA display or HDMI-audio capability;
any perception workload that requires a GPU must run on other hardware.

## Powered bus discovery — 2026-07-24

With the control hardware powered and no rt-control container running, IgH
reported Link UP, 15 PREOP slaves, zero Tx errors and initially zero lost
frames. The frozen motion topology matched by position and vendor/product:

- positions 0 and 13 are observed-only SG-ECAT-HUB_6 devices;
- ZeroErr drives are at 1, 4, 5, 6, 7, 10, 11, 12 and 14;
- Ti5Robot drives are at 2, 3, 8 and 9.

The newly archived non-motion identities are position 0
`ee000002:00020205:00000000` and position 13
`ee000002:00020206:00000001`. Position 14 reported ZeroErr revision
`00000001`, whereas the hardware export carried `00020111` as the Turn
scan/reference revision. REQ-ECAT-001 explicitly says revision matching must
follow the live scan, and the delivered YAML matches only vendor/product, so
this does not change PDO/SDO configuration; the discrepancy remains explicit
commissioning evidence.

The attached CANable 2.0 (USB `16d0:117e`, serial `208031C05230`) was running
stock SLCAN firmware from a manually launched
`slcand -o -c -s6 /dev/ttyACM0 can0`. The manufacturer's command table defines
`S6` as 500 kbit/s. Five seconds of passive observation received operationally
expected heartbeats `0x701`, `0x702`, `0x703` and diagnostic Node 20 `0x714`,
all with data `0x7F` (PRE-OP), with no SocketCAN error counters and no
transmitted frames. Because SLCAN configures bitrate inside the USB adapter,
Linux reports the virtual interface as `bitrate 0`; the process was also tied
to an interactive user session. This cannot satisfy the frozen native
`can0.service` evidence.

The production decision is to flash the officially supported candleLight
firmware during a physical maintenance step and use the in-tree, signed
`gs_usb` module already present in `6.8.1-1056-realtime`. The existing frozen
unit can then set and expose `bitrate 500000` through netlink. Stock SLCAN is
limited to passive commissioning until that conversion. Benefit: native
SocketCAN, systemd ownership, observable bitrate and better high-load behavior.
Drawback: firmware flashing requires local BOOT-button/unplug/replug actions and
has recoverability risk; CANable 2.0 candleLight lacks CAN-FD, which does not
affect this classic-CAN deployment. Sources:
`https://canable.io/getting-started.html` and
`https://github.com/normaldotcom/canable2-fw`.

The existing Docker image was verified but deliberately not started. It is
tagged with Git HEAD `4e0d586f73c5548f778e5ef889f5854a6c19c604`, has image ID
`69642097ed0079b3c05335aa62c1efa5a987e7a8572bda4b0c0fd083f2ea465f`, and
contains IgH stable-1.6 commit `2f7f884f1c7d377c02a7d627eb06512126a0e50e`;
the frozen Compose wrapper config passes on CPU 14. It predates the T-009
`thread_priority: 80` host decision and its installed controllers file contains
no `thread_priority`, so it is staging-only and must not be promoted under the
same immutable tag.

At 14:22 local time both physical buses disappeared during passive sampling:
the CANable USB device unregistered first, followed about 15 seconds later by
EtherCAT Link DOWN and zero responding slaves. IgH accumulated 389 lost frames.
No container or control process had been started, so this event is classified
as an external power/cabling change, not a software-command result. Testing is
paused until the operator confirms the disconnect was intentional and both
buses are restored.

## Staged low-latency migration — 2026-07-24

The operator superseded the earlier no-GPU software scope and selected an
Ubuntu HWE low-latency migration because NVIDIA does not support the complete
PREEMPT_RT kernel. The broken CUDA-repository 535 DKMS/driver packages were
removed without `autoremove`; `dpkg --configure -a`, dependency repair and
`dpkg --audit` then completed cleanly. The retained boot choices are:

- `6.8.0-136-lowlatency`, installed and booted once via `grub-reboot`;
- `6.8.1-1056-realtime`, still the normal GRUB-default top entry;
- `6.8.0-134-generic`, retained as an additional recovery kernel.

The staged low-latency boot has `CONFIG_PREEMPT=y` and no
`CONFIG_PREEMPT_RT`; the existing `nosmt`, CPU14 `isolcpus/nohz_full/rcu_nocbs`
and complementary `irqaffinity` arguments are present in `/proc/cmdline`. IgH
stable-1.6 commit `2f7f884f1c7d377c02a7d627eb06512126a0e50e` was rebuilt with
GCC 12 for the exact new ABI. `ec_master`, `ec_igb` and `ec_generic` load, the
service is active, and the master reports the frozen MAC
`8c:59:3c:15:01:f8 (attached)`. The physical ring was not connected at that
sample: Link DOWN and zero slaves, so no topology claim is made.

With the native `igb` driver temporarily visible before IgH takeover, the
frozen MAC uniquely belongs to `enp2s0`; `enp3s0` belongs to the other I210 at
MAC `8c:59:3c:15:01:f9`. This corrects the earlier interface-name observation
without changing `MASTER0_DEVICE`. The installer now resolves the current
netdev by exact MAC and fails on zero or multiple matches, instead of pairing a
mutable kernel name with the frozen hardware identity.

For the GPU software path, Ubuntu's precompiled
`linux-modules-nvidia-595-open-6.8.0-136-lowlatency` and
`nvidia-driver-595-open=595.84-0ubuntu0.22.04.1` replaced the incomplete 535
stack. No NVIDIA DKMS package was installed. The module reports version 595.84
and a vermagic matching `6.8.0-136-lowlatency`. This satisfies NVIDIA's stated
CUDA 13.2 Update 1 minimum of 595.58.03 and its recommendation to use open
kernel modules on Turing and newer GPUs.

This is not yet a completed GPU migration. The root-port and modprobe
quarantines remain active because the prior physical-layer AER evidence has
not been invalidated. A controlled PCI rescan did not re-enumerate the root
port, so it provided neither new AER nor a visible GPU. An explicitly approved
boot without the root-port service is required before testing negotiated link
speed, AER growth and `nvidia-smi`. Until that gate passes, low-latency remains
a one-time staged boot and is not written as the permanent GRUB default.

The operator approved that boot test. Execution was narrowed to a one-shot,
two-second sysinit window that kept NVIDIA module loading blocked, captured
the PCI/AER state, then automatically invoked the original quarantine and
removed its own drop-in. On the resulting low-latency boot, the wrapper could
not see `00:01.0`, `01:00.0` or `01:00.1` at either sample; AER-matching kernel
messages remained 4 to 4. This is an inconclusive hardware result because the
GPU path was absent for the entire window, not a zero-error link result. The
normal quarantine ExecStart has been restored, all services are healthy and
log usage remained bounded. If the GPU is still physically installed, a full
power-off/cold-start is required because the software reboot did not restore
the removed root port. The modprobe quarantine and non-permanent GRUB state
remain unchanged until that physical-state question is resolved.

The operator later clarified that the GPU was not installed during that
21:01 test, then inserted it and performed a cold start. The default boot
returned to `6.8.1-1056-realtime`. Before the normal quarantine removed the
root port, the boot logged 2,850 AER-matching messages, again correctable
Physical Layer `RxErr` from `01:00.0`; therefore the hardware fault persists.
The inserted card also caused firmware to stop enumerating the Intel
`00:02.0` iGPU. Once NVIDIA was quarantined, no PCI display device or usable
DRM node remained. GDM stayed running but Xorg exhausted its retries with
`open /dev/dri/card0: Permission denied`, `no primary bus or device found` and
`no screens found`. Restoring the GUI requires forcing the iGPU enabled and
primary in AMI BIOS `RXE26005` and using a motherboard display output;
disabling the NVIDIA quarantine is not an acceptable display workaround while
the AER fault remains.

## Remaining hardware gates

- CPU/GRUB isolation is applied and verified: RT active, CPU 14 isolated and
  tickless, SMT siblings offline, and no effective IRQ targets CPU 14.
- Resolve BQ-098 before any unquarantined GPU boot. If approved, keep module
  loading blocked during the first physical-link/AER sample and restore the
  root-port quarantine immediately on any recurrence. Only a zero-growth link
  sample followed by successful `nvidia-smi` may unlock GPU-load timing tests.
- Connect the EtherCAT ring; start IgH; archive `ethercat master`,
  `ethercat slaves` and `ethercat slaves -v` for all 15 positions. Positions 0
  and 13 remain observed-only identities.
- Attach the CAN adapter; install/enable `hostsetup/can0.service`; verify exact
  `500000` bitrate and observe heartbeats `0x701/0x702/0x703` without writing.
  The current CANable must first be flashed to candleLight/`gs_usb`; manual
  SLCAN does not satisfy this gate.
- Host/container IgH version parity is verified at 1.6.10; retain the exact
  source-commit evidence from `versions.env` in the final commissioning bundle.
- Run the single required 30-minute 250 Hz soak and cyclictest acceptance after
  both buses are connected and the faulty GPU is physically absent.
