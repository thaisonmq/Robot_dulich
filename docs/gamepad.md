# Mở rộng Gamepad API

Tạo `GamepadInputAdapter` phát cùng `InputAction` vào `InputManager`.

- Poll `navigator.getGamepads()` bằng một vòng `requestAnimationFrame`.
- Áp dead-zone cho hai trục, ví dụ `0.12`.
- Quantize hoặc mở rộng `CommandComposer` để chấp nhận mức analog.
- Nút B/e-stop gọi `emergencyStop`.
- `gamepaddisconnected`, blur, hidden và unmount phải gọi `clear()`.
- Adapter không được tự tạo WebSocket hay phát lệnh; timer 10 Hz vẫn do
  `InputManager` sở hữu.

Nhờ ranh giới này, keyboard, control pad và gamepad không tạo nhiều timer hoặc
tranh chấp transport.

