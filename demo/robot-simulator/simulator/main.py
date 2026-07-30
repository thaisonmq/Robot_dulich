import asyncio
import logging
import signal

from simulator.client import RobotConnectionClient
from simulator.config import SimulatorConfig


async def async_main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s service=robot-simulator %(name)s %(message)s",
    )
    client = RobotConnectionClient(SimulatorConfig())
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(client.stop()))
    await client.run()


if __name__ == "__main__":
    asyncio.run(async_main())

