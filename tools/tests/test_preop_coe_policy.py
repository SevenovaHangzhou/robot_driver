"""Compile the patched send function and prove forbidden states queue no frame."""
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / "patches/igh/0003-preop-only-coe.patch"


def _patched_function(filename="master/mailbox.c", signature="uint8_t *ec_slave_mbox_prepare_send("):
    section = PATCH.read_text().split(f"diff --git a/{filename} b/{filename}", 1)[1]
    section = section.split("diff --git", 1)[0]
    postimage = "\n".join(line[1:] for line in section.splitlines()
                          if line.startswith((" ", "+")) and not line.startswith("+++"))
    start = postimage.index(signature)
    opened = postimage.index("{", start)
    depth = 1
    end = opened + 1
    while depth:
        depth += (postimage[end] == "{") - (postimage[end] == "}")
        end += 1
    return postimage[start:end]


def test_coe_send_is_rejected_without_queuing_a_datagram_outside_preop(tmp_path):
    # These are transport stubs; the function under test comes from the actual
    # upstream patch, not a second implementation of the state policy.
    harness = r'''
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define unlikely(x) (x)
#define EC_SLAVE_ERR(...) ((void)0)
#define EC_MBOX_HEADER_SIZE 6
#define EC_MBOX_TYPE_COE 3
#define EC_SLAVE_STATE_PREOP 2
#define ERR_PTR(x) ((void *)(intptr_t)(x))
#define EC_WRITE_U16(p,v) do { ((uint8_t *)(p))[0]=(v)&255; ((uint8_t *)(p))[1]=(v)>>8; } while (0)
#define EC_WRITE_U8(p,v) (*(uint8_t *)(p)=(v))
typedef struct { uint8_t data[128]; } ec_datagram_t;
typedef struct {
    struct { unsigned mailbox_protocols; } sii;
    unsigned current_state, configured_rx_mailbox_size, configured_rx_mailbox_offset, station_address;
} ec_slave_t;
static unsigned queued;
static int ec_datagram_fpwr(ec_datagram_t *d, unsigned s, unsigned o, unsigned n) {
    (void)d; (void)s; (void)o; (void)n; ++queued; return 0;
}
'''
    harness += _patched_function()
    harness += r'''
int main(void) {
    unsigned states[] = {1, 2, 4, 8, 0x12, 0x18, 0};
    ec_slave_t slave = {.sii={4}, .configured_rx_mailbox_size=128,
                       .configured_rx_mailbox_offset=0x1000, .station_address=15};
    ec_datagram_t datagram = {{0}};
    for (unsigned i=0; i<sizeof(states)/sizeof(states[0]); ++i) {
        slave.current_state=states[i]; queued=0;
        uint8_t *p=ec_slave_mbox_prepare_send(&slave,&datagram,EC_MBOX_TYPE_COE,10);
        if (states[i]==EC_SLAVE_STATE_PREOP) {
            assert(queued==1 && p==datagram.data+6);
            assert(datagram.data[0]==10 && datagram.data[2]==15 && datagram.data[5]==3);
        } else {
            assert(queued==0 && (intptr_t)p==-EPERM);
        }
    }
    slave.current_state=8; queued=0;
    assert(ec_slave_mbox_prepare_send(&slave,&datagram,15,10)==datagram.data+6);
    assert(queued==1);
    return 0;
}
'''
    source = tmp_path / "policy.c"
    source.write_text(harness)
    binary = tmp_path / "policy"
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_blocked_upload_and_download_report_a_real_permission_error(tmp_path):
    function = _patched_function("master/fsm_coe.c", "void ec_fsm_coe_transfer(")
    harness = r'''
#include <assert.h>
#include <errno.h>
#undef errno
#define EC_SLAVE_STATE_PREOP 2
#define EC_DIR_OUTPUT 1
typedef struct { int current_state; } ec_slave_t;
typedef struct { int dir, errno; } ec_sdo_request_t;
typedef struct { ec_slave_t *slave; ec_sdo_request_t *request; void (*state)(void); } ec_fsm_coe_t;
static void ec_fsm_coe_error(void) {}
static void ec_fsm_coe_up_start(void) {}
static void ec_fsm_coe_down_start(void) {}
''' + function + r'''
int main(void) {
    for (int dir=0; dir<2; ++dir) {
        ec_slave_t slave={8}; ec_sdo_request_t request={dir,0}; ec_fsm_coe_t fsm={0};
        ec_fsm_coe_transfer(&fsm,&slave,&request);
        assert(request.errno==EPERM && fsm.state==ec_fsm_coe_error);
        slave.current_state=2; request.errno=0;
        ec_fsm_coe_transfer(&fsm,&slave,&request);
        assert(request.errno==0 && fsm.state==(dir==EC_DIR_OUTPUT ? ec_fsm_coe_down_start : ec_fsm_coe_up_start));
    }
    return 0;
}
'''
    source=tmp_path/"request.c"; source.write_text(harness)
    binary=tmp_path/"request"
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",str(source),"-o",str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
