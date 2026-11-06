import json

import pytest

from infra_socketcan.link import link_problem


def answer(flags, state):
    return lambda *args, **kwargs: json.dumps(
        [{"flags": flags, "linkinfo": {"info_data": {"state": state}}}]
    )


@pytest.mark.parametrize(
    ("flags", "state", "problem"),
    [
        (["UP"], "ERROR-ACTIVE", None),
        ([], "ERROR-ACTIVE", "not up"),
        (["UP"], "BUS-OFF", "not error-active: BUS-OFF"),
    ],
)
def test_only_an_up_error_active_interface_is_admitted(flags, state, problem):
    found = link_problem("can2", run=answer(flags, state))
    if problem is None:
        assert found is None
    else:
        assert found[0] == "offline" and problem in found[1]
