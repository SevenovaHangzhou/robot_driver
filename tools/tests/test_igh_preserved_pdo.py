import os
import shutil
import subprocess
from pathlib import Path

import pytest


def extract_function(source, signature):
    start = source.index(signature + "(\n")
    return source[start:source.index("\n}\n", start) + 3]


@pytest.fixture(scope="module")
def pdo_configuration_binary(tmp_path_factory):
    source_root = os.environ.get("IGH_PDO_SOURCE_ROOT")
    if not source_root:
        pytest.skip("set IGH_PDO_SOURCE_ROOT to the patched frozen IgH checkout")
    root = Path(source_root)
    pdo = (root / "master/fsm_pdo.c").read_text()
    config = (root / "master/fsm_slave_config.c").read_text()
    functions = [
        extract_function(pdo, "int ec_fsm_pdo_conf_preserve_config"),
        extract_function(config, "void ec_fsm_slave_config_state_pdo_conf"),
        extract_function(config, "int ec_fsm_slave_config_running"),
    ]
    program = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { unsigned int value; } ec_flag_t;
typedef struct { ec_flag_t *flag; } ec_slave_config_t;
typedef struct { ec_slave_config_t *config; int error_flag; } ec_slave_t;
typedef struct { int unused; } ec_datagram_t;
typedef struct { ec_slave_t *slave; int busy; int success; } ec_fsm_pdo_t;
typedef struct ec_fsm_slave_config ec_fsm_slave_config_t;
struct ec_fsm_slave_config {
    ec_slave_t *slave;
    ec_fsm_pdo_t *fsm_pdo;
    ec_datagram_t *datagram;
    void (*state)(ec_fsm_slave_config_t *);
};
static int watchdog_calls, reconfigure_calls;
#define EC_SLAVE_WARN(slave, ...) ((void)(slave))
#define EC_SLAVE_ERR(slave, ...) ((void)(slave))
static ec_flag_t *ec_slave_config_find_flag(ec_slave_config_t *config, const char *name)
{ assert(strcmp(name, "PreservePdoConfig") == 0); return config->flag; }
static int ec_fsm_pdo_exec(ec_fsm_pdo_t *pdo, ec_datagram_t *datagram)
{ (void)datagram; return pdo->busy; }
static int ec_fsm_pdo_success(ec_fsm_pdo_t *pdo) { return pdo->success; }
static void ec_fsm_slave_config_state_error(ec_fsm_slave_config_t *fsm) { (void)fsm; }
static void ec_fsm_slave_config_state_end(ec_fsm_slave_config_t *fsm) { (void)fsm; }
static void ec_fsm_slave_config_enter_watchdog_divider(ec_fsm_slave_config_t *fsm)
{ (void)fsm; ++watchdog_calls; }
static void ec_fsm_slave_config_reconfigure(ec_fsm_slave_config_t *fsm)
{ (void)fsm; ++reconfigure_calls; }
int ec_fsm_pdo_conf_preserve_config(const ec_fsm_pdo_t *);
''' + "\n".join(functions) + r'''
int main(int argc, char **argv) {
    assert(argc == 2);
    ec_flag_t flag = {1};
    ec_slave_config_t config = {&flag};
    ec_slave_t slave = {&config, 0};
    ec_fsm_pdo_t pdo = {&slave, 0, 0};
    ec_datagram_t datagram = {0};
    ec_fsm_slave_config_t fsm = {&slave, &pdo, &datagram,
                                ec_fsm_slave_config_state_pdo_conf};
    if (!strcmp(argv[1], "absent_flag")) config.flag = NULL;
    else if (!strcmp(argv[1], "zero_flag")) flag.value = 0;
    else if (!strcmp(argv[1], "success")) pdo.success = 1;
    else if (!strcmp(argv[1], "busy")) pdo.busy = 1;
    else if (!strcmp(argv[1], "removed_config")) slave.config = NULL;
    else assert(!strcmp(argv[1], "preserved_failure"));
    ec_fsm_slave_config_state_pdo_conf(&fsm);
    if (!strcmp(argv[1], "preserved_failure")) {
        if (watchdog_calls || !slave.error_flag || ec_fsm_slave_config_running(&fsm)) {
            fprintf(stderr, "Preserved PDO failure continued: watchdog=%d error=%d running=%d\n",
                    watchdog_calls, slave.error_flag, ec_fsm_slave_config_running(&fsm));
            return 1;
        }
        assert(fsm.state == ec_fsm_slave_config_state_error);
    } else {
        assert(!slave.error_flag);
        assert(ec_fsm_slave_config_running(&fsm));
        assert(watchdog_calls == (strcmp(argv[1], "busy") && strcmp(argv[1], "removed_config")));
    }
    assert(reconfigure_calls == !strcmp(argv[1], "removed_config"));
    return 0;
}
'''
    directory = tmp_path_factory.mktemp("igh-preserved-pdo")
    source = directory / "test.c"
    binary = directory / "test"
    source.write_text(program)
    compiler = shutil.which("cc")
    assert compiler
    subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                    str(source), "-o", str(binary)], check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize("scenario", [
    "preserved_failure", "absent_flag", "zero_flag", "success", "busy", "removed_config",
])
def test_preserved_pdo_failure_propagates_without_changing_other_paths(pdo_configuration_binary, scenario):
    result = subprocess.run([str(pdo_configuration_binary), scenario], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
