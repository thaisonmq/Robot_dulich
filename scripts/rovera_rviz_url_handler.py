#!/usr/bin/env python3
"""Open the installed Rovera RViz preset for a validated custom URL."""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import parse_qs, urlparse


SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.:-]{0,128}$")


def fail(message: str) -> int:
    print(f"Rovera RViz launcher: {message}", file=sys.stderr)
    return 2


def main() -> int:
    if len(sys.argv) != 2:
        return fail("expected one rovera-rviz:// URL")

    parsed = urlparse(sys.argv[1])
    if parsed.scheme != "rovera-rviz" or parsed.netloc != "mapping":
        return fail("unsupported launcher URL")

    query = parse_qs(parsed.query, strict_parsing=False)
    try:
        domain_id = int(query.get("domain", ["21"])[0])
    except ValueError:
        return fail("invalid ROS domain")
    if not 0 <= domain_id <= 232:
        return fail("ROS domain must be between 0 and 232")

    for field in ("robot_id", "session_id"):
        value = query.get(field, [""])[0]
        if not SAFE_IDENTIFIER.fullmatch(value):
            return fail(f"invalid {field}")

    install_root = Path(__file__).resolve().parent
    opener = install_root / "scripts" / "open_mapping_rviz.sh"
    if not opener.is_file():
        return fail(f"missing installed opener: {opener}")

    state_dir = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
    log_dir = state_dir / "rovera-rviz"
    log_dir.mkdir(parents=True, exist_ok=True)
    log = (log_dir / "launcher.log").open("ab", buffering=0)

    environment = {
        **os.environ,
        "ROS_DOMAIN_ID": str(domain_id),
        "ROS_LOCALHOST_ONLY": "0",
    }
    subprocess.Popen(
        [str(opener)],
        cwd=install_root,
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        close_fds=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
