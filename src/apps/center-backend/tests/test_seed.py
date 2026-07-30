from unittest.mock import patch

from app.core.config import get_settings
from app.services.seed import seed_database


def test_demo_robot_seed_is_opt_in() -> None:
    settings = get_settings()
    original = settings.seed_demo_robot
    try:
        with patch("app.services.seed._seed_demo_robot") as seed_robot:
            settings.seed_demo_robot = False
            seed_database()
            seed_robot.assert_not_called()

            settings.seed_demo_robot = True
            seed_database()
            seed_robot.assert_called_once()
    finally:
        settings.seed_demo_robot = original
