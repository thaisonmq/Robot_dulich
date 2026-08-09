from fastapi.testclient import TestClient

from app.main import app


def auth_headers(token: str) -> dict[str, str]:
    return {"Authorization": f"Bearer {token}"}


def test_bootstrap_admin_login_and_profile() -> None:
    with TestClient(app) as client:
        login = client.post(
            "/api/auth/login",
            json={"identifier": "admin", "password": "admin123"},
        )
        assert login.status_code == 200
        payload = login.json()
        assert payload["user"]["role"] == "admin"
        assert payload["user"]["username"] == "admin"
        assert payload["user"]["must_change_password"] is True
        assert "accounts.manage" in payload["user"]["permissions"]

        profile = client.get(
            "/api/auth/me",
            headers=auth_headers(payload["access_token"]),
        )
        assert profile.status_code == 200
        assert profile.json()["email"] == "admin@rovera.local"


def test_registration_defaults_to_guest_and_rbac_blocks_operations() -> None:
    with TestClient(app) as client:
        registered = client.post(
            "/api/auth/register",
            json={
                "username": "visitor.one",
                "email": "visitor.one@example.com",
                "full_name": "Trần Hà Anh",
                "password": "visitor-password-2026",
            },
        )
        assert registered.status_code == 201
        user = registered.json()["user"]
        assert user["role"] == "guest"
        assert user["active"] is True
        assert user["password_enabled"] is True
        assert "maps.view" not in user["permissions"]
        headers = auth_headers(registered.json()["access_token"])

        assert client.get("/api/robots", headers=headers).status_code == 200
        assert client.get("/api/maps", headers=headers).status_code == 403
        active_maps = client.get("/api/maps?status=ACTIVE", headers=headers)
        assert active_maps.status_code == 200
        assert all(item["status"] == "ACTIVE" for item in active_maps.json())
        active_map = client.get("/api/maps/MAP-001", headers=headers)
        assert active_map.status_code == 200
        assert "versions" not in active_map.json()
        assert client.get(
            "/api/maps/MAP-001/cache?robot_id=ROBOT-001", headers=headers
        ).status_code == 403
        blocked = client.post(
            "/api/robots/quick-add",
            headers=headers,
            json={
                "management_address": "192.168.70.18",
                "username": "robot-user",
                "password": "robot-password",
            },
        )
        assert blocked.status_code == 403
        assert client.get("/api/admin/users", headers=headers).status_code == 403

        duplicate = client.post(
            "/api/auth/register",
            json={
                "username": "visitor.two",
                "email": "visitor.one@example.com",
                "full_name": "Người dùng trùng",
                "password": "another-password-2026",
            },
        )
        assert duplicate.status_code == 409


def test_admin_creates_operator_updates_role_and_resets_password() -> None:
    with TestClient(app) as client:
        admin_login = client.post(
            "/api/auth/login",
            json={"identifier": "admin", "password": "admin123"},
        ).json()
        admin_headers = auth_headers(admin_login["access_token"])

        created = client.post(
            "/api/admin/users",
            headers=admin_headers,
            json={
                "username": "operator.linh",
                "email": "linh.operator@example.com",
                "full_name": "Vũ Thuỳ Linh",
                "password": "temporary-password",
                "role": "operator",
                "must_change_password": True,
            },
        )
        assert created.status_code == 201
        operator = created.json()
        assert operator["role"] == "operator"
        assert operator["created_by_id"] == admin_login["user"]["id"]

        operator_login = client.post(
            "/api/auth/login",
            json={
                "identifier": "operator.linh",
                "password": "temporary-password",
            },
        )
        assert operator_login.status_code == 200
        assert operator_login.json()["user"]["must_change_password"] is True

        changed = client.patch(
            f"/api/admin/users/{operator['id']}",
            headers=admin_headers,
            json={"role": "guest", "active": True},
        )
        assert changed.status_code == 200
        assert changed.json()["role"] == "guest"

        reset = client.post(
            f"/api/admin/users/{operator['id']}/reset-password",
            headers=admin_headers,
            json={
                "new_password": "replacement-password",
                "must_change_password": True,
            },
        )
        assert reset.status_code == 200
        assert client.post(
            "/api/auth/login",
            json={
                "identifier": "operator.linh",
                "password": "replacement-password",
            },
        ).status_code == 200

        user_page = client.get(
            "/api/admin/users?role=guest&search=operator.linh",
            headers=admin_headers,
        )
        assert user_page.status_code == 200
        assert user_page.json()["total"] == 1


def test_profile_and_password_change() -> None:
    with TestClient(app) as client:
        registered = client.post(
            "/api/auth/register",
            json={
                "username": "profile.user",
                "email": "profile.user@example.com",
                "full_name": "Phạm Mai",
                "password": "initial-password",
            },
        ).json()
        headers = auth_headers(registered["access_token"])

        profile = client.patch(
            "/api/auth/me",
            headers=headers,
            json={"full_name": "Phạm Ngọc Mai"},
        )
        assert profile.status_code == 200
        assert profile.json()["full_name"] == "Phạm Ngọc Mai"

        wrong = client.post(
            "/api/auth/me/password",
            headers=headers,
            json={
                "current_password": "wrong-password",
                "new_password": "new-secure-password",
            },
        )
        assert wrong.status_code == 400
        changed = client.post(
            "/api/auth/me/password",
            headers=headers,
            json={
                "current_password": "initial-password",
                "new_password": "new-secure-password",
            },
        )
        assert changed.status_code == 200
        assert client.post(
            "/api/auth/login",
            json={
                "identifier": "profile.user",
                "password": "new-secure-password",
            },
        ).status_code == 200


def test_google_oauth_reports_configuration_and_rejects_unknown_code() -> None:
    with TestClient(app) as client:
        status = client.get("/api/auth/google/status")
        assert status.status_code == 200
        assert status.json() == {"enabled": False}
        assert client.get("/api/auth/google/login").status_code == 503
        assert client.post(
            "/api/auth/google/exchange",
            json={"code": "x" * 48},
        ).status_code == 401


def test_operator_manages_only_guest_accounts() -> None:
    with TestClient(app) as client:
        operator_login = client.post(
            "/api/auth/login",
            json={"identifier": "demo", "password": "demo123"},
        ).json()
        operator_headers = auth_headers(operator_login["access_token"])
        admin_login = client.post(
            "/api/auth/login",
            json={"identifier": "admin", "password": "admin123"},
        ).json()

        created_guest = client.post(
            "/api/admin/users",
            headers=operator_headers,
            json={
                "username": "operator.guest",
                "email": "operator.guest@example.com",
                "full_name": "Khách do vận hành tạo",
                "password": "temporary-password",
                "role": "guest",
                "must_change_password": True,
            },
        )
        assert created_guest.status_code == 201
        assert created_guest.json()["role"] == "guest"

        blocked_operator = client.post(
            "/api/admin/users",
            headers=operator_headers,
            json={
                "username": "forbidden.operator",
                "email": "forbidden.operator@example.com",
                "full_name": "Không được tạo",
                "password": "temporary-password",
                "role": "operator",
                "must_change_password": True,
            },
        )
        assert blocked_operator.status_code == 403

        scoped_page = client.get(
            "/api/admin/users",
            headers=operator_headers,
        )
        assert scoped_page.status_code == 200
        assert scoped_page.json()["items"]
        assert all(
            item["role"] == "guest" for item in scoped_page.json()["items"]
        )
        assert scoped_page.json()["summary"]["admin"] == 0
        assert scoped_page.json()["summary"]["operator"] == 0

        assert client.patch(
            f"/api/admin/users/{admin_login['user']['id']}",
            headers=operator_headers,
            json={"active": False},
        ).status_code == 403
        assert client.patch(
            f"/api/admin/users/{operator_login['user']['id']}",
            headers=operator_headers,
            json={"active": False},
        ).status_code == 403

        guest_id = created_guest.json()["id"]
        assert client.patch(
            f"/api/admin/users/{guest_id}",
            headers=operator_headers,
            json={"active": False},
        ).status_code == 200
        assert client.patch(
            f"/api/admin/users/{guest_id}",
            headers=operator_headers,
            json={"role": "operator"},
        ).status_code == 403
