from fastapi import APIRouter, Depends, HTTPException

from app.core.config import Settings, get_settings
from app.core.security import create_access_token
from app.schemas.messages import LoginRequest

router = APIRouter(prefix="/api/auth", tags=["auth"])


@router.post("/login")
async def login(body: LoginRequest, settings: Settings = Depends(get_settings)) -> dict:
    if body.email != settings.demo_email or body.password != settings.demo_password:
        raise HTTPException(status_code=401, detail="Email hoặc mật khẩu không đúng")
    return {
        "access_token": create_access_token(body.email, settings),
        "token_type": "bearer",
        "user": {"id": body.email, "email": body.email, "name": "Nguyễn Minh", "role": "operator"},
    }

