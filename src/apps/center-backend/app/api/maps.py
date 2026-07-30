from fastapi import APIRouter, Depends, HTTPException

from app.core.security import current_user
from app.services.maps import DESTINATIONS, MAP

router = APIRouter(prefix="/api/maps", tags=["maps"])


@router.get("/{map_id}")
async def get_map(map_id: str, _: str = Depends(current_user)) -> dict:
    if map_id != MAP["map_id"]:
        raise HTTPException(status_code=404, detail="Không tìm thấy bản đồ")
    return MAP


@router.get("/{map_id}/destinations")
async def get_destinations(map_id: str, _: str = Depends(current_user)) -> list[dict]:
    if map_id != MAP["map_id"]:
        raise HTTPException(status_code=404, detail="Không tìm thấy bản đồ")
    return [item for item in DESTINATIONS if item["map_id"] == map_id and item["enabled"]]

