import pathlib
import sys


def main():
    control_loop = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
    control_thread = pathlib.Path(sys.argv[2]).read_text(encoding="utf-8")
    forbidden = "copy.initialized = initialized_"
    if forbidden in control_loop:
        raise RuntimeError(
            "status() must not read initialized_ under status_mutex_; use status_ snapshot"
        )
    if "status_.initialized = initialized_" not in control_thread:
        raise RuntimeError("refresh_status() must own the initialized_ snapshot update")
    guarded_call = "try {\n    initialize_odometry(now);"
    if guarded_call not in control_loop:
        raise RuntimeError("initialize_odometry() must remain inside an exception guard")
    guard_tail = control_loop.split(guarded_call, maxsplit=1)[1][:800]
    if "catch (const std::exception & error)" not in guard_tail:
        raise RuntimeError("odometry guard must report standard exceptions")
    if "catch (...)" not in guard_tail:
        raise RuntimeError("odometry guard must contain unknown exceptions")


if __name__ == "__main__":
    main()
