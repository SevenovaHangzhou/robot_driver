"""CiA402 status-word decoder for Python tools (offline preflight).

Same table as include/hw_semantic_components/cia402_state.hpp; both are checked against
test/cia402_cases.txt so the two languages cannot drift.
"""

# (mask, value, state) in the order the C++ decoder tests them.
_TABLE = (
    (0x004F, 0x0000, "not_ready_to_switch_on"),
    (0x004F, 0x0040, "switch_on_disabled"),
    (0x006F, 0x0021, "ready_to_switch_on"),
    (0x006F, 0x0023, "switched_on"),
    (0x006F, 0x0027, "operation_enabled"),
    (0x006F, 0x0007, "quick_stop_active"),
    (0x004F, 0x000F, "fault_reaction_active"),
    (0x004F, 0x0008, "fault"),
)
# Status-word bit 3 "Fault"; the preflight treats any word with it set as faulted.
FAULT_BIT = 0x0008


def decode_state(status_word: int) -> str:
    for mask, value, state in _TABLE:
        if status_word & mask == value:
            return state
    return "unknown"
