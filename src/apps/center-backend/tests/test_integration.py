import hashlib
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from uuid import uuid4

import jwt
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from app.main import app
from app.models.database import SessionLocal
from app.models.entities import ControlSession, Robot
from app.services.hub import hub


def envelope(message_type: str, session_id: str, sequence: int, payload: dict) -> dict:
    return {
        "message_id": str(uuid4()),
        "schema_version": "1.0",
        "message_type": message_type,
        "robot_id": "ROBOT-001",
        "session_id": session_id,
        "sequence": sequence,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "ttl_ms": 300,
        "payload": payload,
    }


def test_draft_map_metadata_can_be_edited_and_deleted() -> None:
    client = TestClient(app)
    with client:
        login = client.post(
            "/api/auth/login",
            json={"email": "demo@rovera.local", "password": "demo123"},
        )
        headers = {"Authorization": f"Bearer {login.json()['access_token']}"}
        created = client.post(
            "/api/maps",
            headers=headers,
            json={"name": "Draft map", "site_id": "A", "floor_id": "1", "notes": ""},
        )
        assert created.status_code == 201
        map_id = created.json()["map_id"]

        updated = client.patch(
            f"/api/maps/{map_id}",
            headers=headers,
            json={"name": "Draft map renamed", "floor_id": "2", "notes": "updated"},
        )
        assert updated.status_code == 200
        assert updated.json()["name"] == "Draft map renamed"
        assert updated.json()["floor_id"] == "2"

        removed = client.delete(f"/api/maps/{map_id}", headers=headers)
        assert removed.status_code == 204
        assert client.get(f"/api/maps/{map_id}", headers=headers).status_code == 404

        active = client.delete("/api/maps/MAP-001", headers=headers)
        assert active.status_code == 409


def test_gateway_session_command_and_telemetry_flow() -> None:
    hub.robot_sockets.clear()
    hub.sessions.clear()
    hub.robot_session.clear()
    hub.control_sockets.clear()
    hub.control_clients.clear()
    client = TestClient(app)
    with client:
        hub.robots["ROBOT-001"].status = "offline"
        login = client.post(
            "/api/auth/login",
            json={"email": "demo@rovera.local", "password": "demo123"},
        )
        token = login.json()["access_token"]
        headers = {"Authorization": f"Bearer {token}"}
        robot_login = client.post(
            "/api/robot-auth/token",
            json={
                "robot_id": "ROBOT-001",
                "credential": "robot-001-change-me",
            },
        )
        assert robot_login.status_code == 200
        robot_headers = {
            "Authorization": f"Bearer {robot_login.json()['access_token']}"
        }
        with client.websocket_connect(
            "/ws/robot/connect?robot_id=ROBOT-001", headers=robot_headers
        ) as robot_ws:
            assert robot_ws.receive_json()["message_type"] == "gateway.welcome"
            robots = client.get("/api/robots", headers=headers).json()
            assert robots["items"][0]["status"] == "online"
            session = client.post(
                "/api/sessions", headers=headers, json={"robot_id": "ROBOT-001"}
            ).json()
            session_id = session["session_id"]
            assert session["expires_at"] is None
            media_start = robot_ws.receive_json()
            assert media_start["message_type"] == "media.start"
            assert media_start["payload"]["lease_id"] == f"session:{session_id}"
            query = (
                f"?session_id={session_id}&token={token}"
                "&client_id=primary-tab"
            )
            with client.websocket_connect(
                f"/ws/user/control/ROBOT-001{query}"
            ) as control_ws:
                assert control_ws.receive_json()["message_type"] == "control.ready"
                command = envelope(
                    "control.velocity", session_id, 1,
                    {"linear_x": 0.4, "angular_z": 0.0},
                )
                control_ws.send_json(command)
                assert robot_ws.receive_json()["message_id"] == command["message_id"]
                assert control_ws.receive_json()["payload"]["status"] == "accepted"
                ptz = envelope(
                    "camera.ptz", session_id, 2,
                    {"operation": "move", "pan": -1, "tilt": 0, "speed": "slow"},
                )
                control_ws.send_json(ptz)
                assert robot_ws.receive_json()["message_id"] == ptz["message_id"]
                assert control_ws.receive_json()["payload"]["status"] == "accepted"
                duplicate_query = (
                    f"?session_id={session_id}&token={token}"
                    "&client_id=duplicated-tab"
                )
                with client.websocket_connect(
                    f"/ws/user/control/ROBOT-001{duplicate_query}"
                ) as duplicate_ws:
                    try:
                        duplicate_ws.receive_json()
                    except WebSocketDisconnect as exc:
                        assert exc.code == 4009
                    else:
                        raise AssertionError("duplicated tab unexpectedly took control")
                with client.websocket_connect(
                    f"/ws/user/telemetry/ROBOT-001{query}"
                ) as telemetry_ws:
                    assert telemetry_ws.receive_json()["message_type"] == "robot.pose"
                    pose = envelope(
                        "robot.pose", session_id, 10,
                        {
                            "map_id": "MAP-001", "x": 6.0, "y": 6.0, "yaw": 0.1,
                            "linear_velocity": 0.3, "angular_velocity": 0.0,
                        },
                    )
                    robot_ws.send_json(pose)
                    received = telemetry_ws.receive_json()
                    assert received["payload"]["x"] == 6.0
                heartbeat = envelope("session.heartbeat", session_id, 3, {})
                control_ws.send_json(heartbeat)
                assert control_ws.receive_json()["payload"]["status"] == "accepted"
            with client.websocket_connect(
                f"/ws/user/control/ROBOT-001{query}"
            ) as reconnected_control_ws:
                assert (
                    reconnected_control_ws.receive_json()["message_type"]
                    == "control.ready"
                )
                heartbeat = envelope("session.heartbeat", session_id, 1, {})
                reconnected_control_ws.send_json(heartbeat)
                assert (
                    reconnected_control_ws.receive_json()["payload"]["status"]
                    == "accepted"
                )
                refreshed = client.get(
                    f"/api/sessions/{session_id}", headers=headers
                )
                assert refreshed.status_code == 200
                assert refreshed.json()["media"]["token"]
            assert client.delete(
                f"/api/sessions/{session_id}", headers=headers
            ).status_code == 200


def test_robot_auth_rejects_wrong_secret_and_scopes_media_token() -> None:
    client = TestClient(app)
    with client:
        rejected = client.post(
            "/api/robot-auth/token",
            json={"robot_id": "ROBOT-001", "credential": "wrong-secret-value"},
        )
        assert rejected.status_code == 401

        accepted = client.post(
            "/api/robot-auth/token",
            json={
                "robot_id": "ROBOT-001",
                "credential": "robot-001-change-me",
            },
        )
        robot_token = accepted.json()["access_token"]
        media = client.post(
            "/api/robot-auth/media-token",
            headers={"Authorization": f"Bearer {robot_token}"},
        )
        assert media.status_code == 200
        assert media.json()["room_name"] == "robot-ROBOT-001"
        assert media.json()["token"]
        media_claims = jwt.decode(
            media.json()["token"], options={"verify_signature": False}
        )
        assert media_claims["video"]["room"] == "robot-ROBOT-001"
        assert media_claims["video"]["canPublish"] is True
        assert media_claims["sub"] == "robot:ROBOT-001"

        video_media = client.post(
            "/api/robot-auth/media-token",
            params={"purpose": "video"},
            headers={"Authorization": f"Bearer {robot_token}"},
        )
        assert video_media.status_code == 200
        video_claims = jwt.decode(
            video_media.json()["token"], options={"verify_signature": False}
        )
        assert video_claims["sub"] == "robot:ROBOT-001:video"
        assert video_claims["video"]["canSubscribe"] is False

        user_endpoint = client.get(
            "/api/robots",
            headers={"Authorization": f"Bearer {robot_token}"},
        )
        assert user_endpoint.status_code == 401


def test_robot_auth_accepts_hashed_device_secret() -> None:
    secret = "unique-random-device-secret-123456"
    digest = hashlib.sha256(secret.encode()).hexdigest()
    with SessionLocal.begin() as database:
        robot = database.query(Robot).filter(Robot.robot_id == "ROBOT-001").first()
        previous_hash = robot.credential_hash
        robot.credential_hash = digest
    try:
        with TestClient(app) as client:
            response = client.post(
                "/api/robot-auth/token",
                json={"robot_id": "ROBOT-001", "credential": secret},
            )
            assert response.status_code == 200
    finally:
        with SessionLocal.begin() as database:
            robot = database.query(Robot).filter(Robot.robot_id == "ROBOT-001").first()
            robot.credential_hash = previous_hash


def test_robot_registry_create_enroll_update_paginate_and_delete() -> None:
    with TestClient(app) as client:
        login = client.post(
            "/api/auth/login",
            json={"email": "demo@rovera.local", "password": "demo123"},
        )
        headers = {"Authorization": f"Bearer {login.json()['access_token']}"}
        created = client.post(
            "/api/robots",
            headers=headers,
            json={
                "robot_id": "ROBOT-TEST-99",
                "name": "Robot kiểm thử registry",
                "site_id": "Phòng thử nghiệm",
                "map_id": "MAP-001",
            },
        )
        assert created.status_code == 201
        enrollment_token = created.json()["enrollment_token"]
        assert created.json()["enrollment_status"] == "pending"

        enrolled = client.post(
            "/api/robot-auth/enroll",
            json={
                "enrollment_token": enrollment_token,
                "device_fingerprint": "test-host:test-machine-id",
            },
        )
        assert enrolled.status_code == 200
        credential = enrolled.json()["credential"]
        assert client.post(
            "/api/robot-auth/enroll",
            json={
                "enrollment_token": enrollment_token,
                "device_fingerprint": "replay",
            },
        ).status_code == 401
        assert client.post(
            "/api/robot-auth/token",
            json={"robot_id": "ROBOT-TEST-99", "credential": credential},
        ).status_code == 200

        updated = client.patch(
            "/api/robots/ROBOT-TEST-99",
            headers=headers,
            json={
                "name": "Robot registry đã sửa",
                "site_id": "Sảnh phụ",
                "map_id": "MAP-001",
                "enabled": True,
            },
        )
        assert updated.json()["name"] == "Robot registry đã sửa"
        page = client.get(
            "/api/robots?page=1&page_size=1&search=ROBOT-TEST-99",
            headers=headers,
        ).json()
        assert page["total"] == 1
        assert page["total_pages"] == 1
        assert page["items"][0]["enrollment_status"] == "enrolled"

        deleted = client.delete("/api/robots/ROBOT-TEST-99", headers=headers)
        assert deleted.status_code == 204


def test_operator_quick_add_and_edge_credential_claim_flow() -> None:
    with TestClient(app) as client:
        login = client.post(
            "/api/auth/login",
            json={"email": "demo@rovera.local", "password": "demo123"},
        )
        headers = {"Authorization": f"Bearer {login.json()['access_token']}"}
        created = client.post(
            "/api/robots/quick-add",
            headers=headers,
            json={
                "management_address": "192.168.50.27",
                "username": "robot-operator",
                "password": "local-device-password",
            },
        )
        assert created.status_code == 201
        robot_id = created.json()["robot_id"]
        assert created.json()["status"] == "offline"
        assert created.json()["management_address"] == "192.168.50.27"
        assert "password" not in created.json()

        with SessionLocal() as database:
            entity = (
                database.query(Robot)
                .filter(Robot.robot_id == robot_id)
                .one()
            )
            assert entity.management_password_hash != "local-device-password"
            assert entity.credential_hash is None

        rejected = client.post(
            "/api/robot-auth/claim",
            json={
                "management_address": "192.168.50.27",
                "username": "robot-operator",
                "password": "wrong-password",
                "device_fingerprint": "edge-host:machine-id",
            },
        )
        assert rejected.status_code == 401
        claimed = client.post(
            "/api/robot-auth/claim",
            json={
                "management_address": "192.168.50.27",
                "username": "robot-operator",
                "password": "local-device-password",
                "device_fingerprint": "edge-host:machine-id",
            },
        )
        assert claimed.status_code == 200
        assert claimed.json()["robot_id"] == robot_id
        credential = claimed.json()["credential"]
        robot_login = client.post(
            "/api/robot-auth/token",
            json={"robot_id": robot_id, "credential": credential},
        )
        assert robot_login.status_code == 200

        robot_headers = {
            "Authorization": f"Bearer {robot_login.json()['access_token']}"
        }
        with client.websocket_connect(
            f"/ws/robot/connect?robot_id={robot_id}",
            headers=robot_headers,
        ) as robot_ws:
            assert robot_ws.receive_json()["message_type"] == "gateway.welcome"
            online = client.get(
                f"/api/robots/{robot_id}", headers=headers
            ).json()
            assert online["status"] == "online"
            hot_update = client.patch(
                f"/api/robots/{robot_id}",
                headers=headers,
                json={
                    "name": "Robot cập nhật khi online",
                    "site_id": "Sảnh đang hoạt động",
                    "map_id": "MAP-001",
                    "enabled": True,
                    "management_address": "192.168.50.28",
                    "management_username": "new-operator",
                    "management_password": "new-local-device-password",
                },
            )
            assert hot_update.status_code == 200
            assert hot_update.json()["status"] == "online"
            assert hot_update.json()["management_address"] == "192.168.50.28"
            media_still_authorized = client.post(
                "/api/robot-auth/media-token",
                headers=robot_headers,
            )
            assert media_still_authorized.status_code == 200

        offline = client.get(f"/api/robots/{robot_id}", headers=headers).json()
        assert offline["status"] == "offline"
        assert client.delete(
            f"/api/robots/{robot_id}", headers=headers
        ).status_code == 204


def test_guest_control_camera_privacy_and_supervisor_force_end() -> None:
    hub.robot_sockets.clear()
    hub.sessions.clear()
    hub.robot_session.clear()
    hub.camera_sources.clear()
    with TestClient(app) as client:
        registered = client.post(
            "/api/auth/register",
            json={
                "username": "camera.guest",
                "email": "camera.guest@example.com",
                "full_name": "Khách camera",
                "password": "guest-camera-password",
            },
        ).json()
        guest_token = registered["access_token"]
        guest_headers = {"Authorization": f"Bearer {guest_token}"}
        assert "robots.operate" in registered["user"]["permissions"]

        operator = client.post(
            "/api/auth/login",
            json={"identifier": "demo", "password": "demo123"},
        ).json()
        operator_headers = {
            "Authorization": f"Bearer {operator['access_token']}"
        }
        robot_login = client.post(
            "/api/robot-auth/token",
            json={
                "robot_id": "ROBOT-001",
                "credential": "robot-001-change-me",
            },
        ).json()
        robot_headers = {
            "Authorization": f"Bearer {robot_login['access_token']}"
        }

        with client.websocket_connect(
            "/ws/robot/connect?robot_id=ROBOT-001", headers=robot_headers
        ) as robot_ws:
            assert robot_ws.receive_json()["message_type"] == "gateway.welcome"
            session_response = client.post(
                "/api/sessions",
                headers=guest_headers,
                json={"robot_id": "ROBOT-001"},
            )
            assert session_response.status_code == 200
            session = session_response.json()
            assert session["mode"] == "control"
            assert session["controller"]["role"] == "guest"
            assert robot_ws.receive_json()["message_type"] == "media.start"
            assert client.get(
                "/api/robots/ROBOT-001/configuration",
                headers=guest_headers,
            ).status_code == 403

            with ThreadPoolExecutor(max_workers=1) as executor:
                pending_cameras = executor.submit(
                    client.get,
                    f"/api/sessions/{session['session_id']}/cameras",
                    headers=guest_headers,
                )
                camera_request = robot_ws.receive_json()
                assert camera_request["message_type"] == "media.cameras.get"
                robot_ws.send_json(
                    envelope(
                        "media.cameras",
                        "",
                        20,
                        {
                            "request_id": camera_request["payload"]["request_id"],
                            "ok": True,
                            "selected_source": "/dev/video0",
                            "video_sources": [
                                {
                                    "type": "camera",
                                    "value": "/dev/video0",
                                    "label": "Logitech Brio",
                                },
                                {
                                    "type": "camera",
                                    "value": "/dev/video2",
                                    "label": "Camera hành lang",
                                },
                            ],
                        },
                    )
                )
                cameras_response = pending_cameras.result(timeout=5)

            assert cameras_response.status_code == 200
            cameras = cameras_response.json()["items"]
            assert [item["label"] for item in cameras] == [
                "Camera 1",
                "Camera 2",
            ]
            assert all("source" not in item for item in cameras)

            with ThreadPoolExecutor(max_workers=1) as executor:
                pending_select = executor.submit(
                    client.put,
                    f"/api/sessions/{session['session_id']}/camera",
                    headers=guest_headers,
                    json={"camera_id": cameras[1]["id"]},
                )
                select_request = robot_ws.receive_json()
                assert select_request["message_type"] == "media.source.select"
                assert select_request["payload"]["source"] == "/dev/video2"
                robot_ws.send_json(
                    envelope(
                        "media.source.state",
                        "",
                        21,
                        {
                            "request_id": select_request["payload"]["request_id"],
                            "ok": True,
                            "selected_source": "/dev/video2",
                        },
                    )
                )
                selected_response = pending_select.result(timeout=5)
            assert selected_response.status_code == 200
            assert selected_response.json()["label"] == "Camera 2"
            assert "source" not in selected_response.json()

            configuration = {
                "device_ip": "192.168.1.20",
                "video_source_type": "camera",
                "video_source": "/dev/video2",
                "video_profile": "balanced",
                "rtsp_transport": "auto",
                "camera_label": "Camera hành lang",
                "audio_source_type": "silent",
                "audio_source": "",
                "microphone_label": "Microphone chính",
                "audio_output_type": "disabled",
                "audio_output": "",
                "speaker_label": "Loa chính",
            }
            with ThreadPoolExecutor(max_workers=1) as executor:
                pending_profile = executor.submit(
                    client.get,
                    f"/api/sessions/{session['session_id']}/video-profile",
                    headers=guest_headers,
                )
                profile_request = robot_ws.receive_json()
                assert profile_request["message_type"] == "configuration.get"
                robot_ws.send_json(
                    envelope(
                        "configuration.state",
                        "",
                        22,
                        {
                            "request_id": profile_request["payload"]["request_id"],
                            "ok": True,
                            **configuration,
                        },
                    )
                )
                profile_response = pending_profile.result(timeout=5)

            assert profile_response.status_code == 200
            assert profile_response.json()["video_profile"] == "balanced"
            assert client.put(
                f"/api/sessions/{session['session_id']}/video-profile",
                headers=guest_headers,
                json={"video_profile": "low_bandwidth"},
            ).status_code == 403

            with ThreadPoolExecutor(max_workers=1) as executor:
                pending_quality = executor.submit(
                    client.put,
                    f"/api/sessions/{session['session_id']}/video-profile",
                    headers=operator_headers,
                    json={"video_profile": "low_bandwidth"},
                )
                get_request = robot_ws.receive_json()
                assert get_request["message_type"] == "configuration.get"
                robot_ws.send_json(
                    envelope(
                        "configuration.state",
                        "",
                        23,
                        {
                            "request_id": get_request["payload"]["request_id"],
                            "ok": True,
                            **configuration,
                        },
                    )
                )
                quality_request = robot_ws.receive_json()
                assert quality_request["message_type"] == "configuration.update"
                assert quality_request["payload"]["video_profile"] == "low_bandwidth"
                assert quality_request["payload"]["video_source"] == "/dev/video2"
                robot_ws.send_json(
                    envelope(
                        "configuration.state",
                        "",
                        24,
                        {
                            "request_id": quality_request["payload"]["request_id"],
                            "ok": True,
                            **configuration,
                            "video_profile": "low_bandwidth",
                        },
                    )
                )
                quality_response = pending_quality.result(timeout=5)

            assert quality_response.status_code == 200
            assert quality_response.json()["video_profile"] == "low_bandwidth"

            active = client.get(
                "/api/sessions/active", headers=operator_headers
            )
            assert active.status_code == 200
            assert active.json()[0]["controller"]["username"] == "camera.guest"
            mine = client.get("/api/sessions/mine", headers=guest_headers)
            assert mine.status_code == 200
            assert [item["session_id"] for item in mine.json()] == [
                session["session_id"]
            ]
            assert client.get(
                "/api/sessions/mine", headers=operator_headers
            ).json() == []

            spectator = client.post(
                f"/api/sessions/{session['session_id']}/spectate",
                headers=operator_headers,
            )
            assert spectator.status_code == 200
            assert spectator.json()["mode"] == "spectator"
            assert spectator.json()["control_websocket_url"] == ""
            spectator_claims = jwt.decode(
                spectator.json()["media"]["token"],
                options={"verify_signature": False},
            )
            assert spectator_claims["video"]["canSubscribe"] is True
            assert spectator_claims["video"].get("canPublish", False) is False

            forced = client.post(
                f"/api/sessions/{session['session_id']}/force-end",
                headers=operator_headers,
            )
            assert forced.status_code == 200
            assert robot_ws.receive_json()["message_type"] == "control.stop"
            assert robot_ws.receive_json()["message_type"] == "media.stop"
            assert client.get(
                "/api/sessions/active", headers=operator_headers
            ).json() == []

            with SessionLocal() as database:
                record = database.get(
                    ControlSession, session["session_id"]
                )
                assert record is not None
                assert record.status == "ended"
                assert record.ended_by_user_id == operator["user"]["id"]
                assert record.end_reason == "force_ended_by_supervisor"
