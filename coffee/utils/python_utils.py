import contextlib
import os
import sys
from typing import Iterator


# Reference: https://stackoverflow.com/a/17954769
@contextlib.contextmanager
def suppress_stdout() -> Iterator[None]:
    """A context manager for suppressing stdout from an underlying C library."""
    fd = sys.stdout.fileno()

    def _redirect_stdout(to):
        sys.stdout.close()
        os.dup2(to.fileno(), fd)
        sys.stdout = os.fdopen(fd, "w")

    with os.fdopen(os.dup(fd), "w") as old_stdout:
        with open(os.devnull, "w") as file:
            _redirect_stdout(to=file)
        try:
            yield
        finally:
            _redirect_stdout(to=old_stdout)
