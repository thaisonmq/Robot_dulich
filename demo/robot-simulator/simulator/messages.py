from datetime import datetime, timezone
from typing import Any
from uuid import UUID, uuid4

from pydantic import BaseModel, Field


class Message(BaseModel):
    message_id: UUID
    schema_version: str = "1.0"
    message_type: str
    robot_id: str
    session_id: str = ""
    sequence: int = Field(ge=0)
    timestamp: datetime
    ttl_ms: int = Field(ge=0, le=30_000)
    payload: dict[str, Any]

    def expired(self) -> bool:
        if self.ttl_ms == 0:
            return False
        return (datetime.now(timezone.utc) - self.timestamp).total_seconds() * 1000 > self.ttl_ms


def make_message(
    message_type: str,
    robot_id: str,
    sequence: int,
    payload: dict[str, Any],
    session_id: str = "",
    ttl_ms: int = 0,
) -> dict[str, Any]:
    return {
        "message_id": str(uuid4()),
        "schema_version": "1.0",
        "message_type": message_type,
        "robot_id": robot_id,
        "session_id": session_id,
        "sequence": sequence,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "ttl_ms": ttl_ms,
        "payload": payload,
    }
