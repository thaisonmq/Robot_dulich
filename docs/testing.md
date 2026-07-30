# Testing

## Unit

```bash
cd src/apps/center-frontend && npm test
PYTHONPATH=src/apps/center-backend .venv/bin/pytest src/apps/center-backend/tests
PYTHONPATH=demo/robot-simulator .venv/bin/pytest demo/robot-simulator/tests
```

Dependency test của simulator nằm trong
`demo/robot-simulator/requirements-dev.txt`; image production chỉ cài
`requirements.txt`.

Coverage trọng tâm: keyboard mapping, tổ hợp trục, keyup/blur STOP, timer,
coordinate conversion, TTL, session lock, robot routing, yaw, pose và watchdog.

## Integration

`test_integration.py` mở robot gateway, đăng nhập, tạo session, chuyển command
từ user sang đúng robot và chuyển pose ngược về telemetry user.

## E2E

Sau `docker compose up --build`:

```bash
cd demo/e2e
npm install
npx playwright install chromium
npm test
```

Test thực hiện login → chọn robot → giữ/nhả ArrowUp → xác nhận pressed/STOP →
preview route → navigation goal → disconnect.
