import os
import tempfile
from pathlib import Path

_test_directory = tempfile.TemporaryDirectory(prefix="rovera-backend-tests-")
_database_path = Path(_test_directory.name) / "tests.db"
os.environ["DATABASE_URL"] = f"sqlite:///{_database_path}"
os.environ["SEED_DEMO_ROBOT"] = "true"
os.environ["ROBOT_ID"] = "ROBOT-001"
os.environ["ROBOT_CREDENTIAL"] = "robot-001-change-me"


def pytest_sessionfinish(session, exitstatus) -> None:
    del session, exitstatus
    _test_directory.cleanup()
