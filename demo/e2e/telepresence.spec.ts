import { expect, test } from "@playwright/test";

test("operator controls simulator and previews a route", async ({ page }) => {
  await page.goto("/");
  await page.getByLabel("Email").fill("demo@rovera.local");
  await page.getByLabel("Mật khẩu", { exact: true }).fill("demo123");
  await page.getByRole("button", { name: "Đăng nhập", exact: true }).click();
  await expect(page).toHaveURL(/\/robots/);
  const firstRobot = page.locator(".robot-row").first();
  await expect(firstRobot).toBeVisible();

  await firstRobot.getByRole("button", { name: /Cấu hình/ }).click();
  await expect(page).toHaveURL(/\/robots\/ROBOT-001\/configuration/);
  await expect(page.getByRole("heading", { name: "Thông số robot" })).toBeVisible();
  await expect(page.getByLabel("Nguồn phát video")).toHaveValue(/^rtsp:\/\//);
  await page.getByRole("button", { name: "Lưu cấu hình" }).click();
  await expect(page.getByText("Đã lưu cấu hình")).toBeVisible();
  await page.getByRole("button", { name: "Danh sách robot" }).click();

  await page.locator(".robot-row").first().getByRole("button", { name: "Kết nối" }).click();
  await expect(page).toHaveURL(/\/control\/ROBOT-001/);
  await expect(page.getByLabel("Video trực tiếp từ robot")).toBeVisible();
  await expect(page.getByText("WEBRTC TRỰC TIẾP")).toBeVisible({ timeout: 12_000 });

  await page.keyboard.down("ArrowUp");
  await expect(page.getByRole("button", { name: "Tiến" })).toHaveAttribute("aria-pressed", "true");
  await page.waitForTimeout(350);
  await page.keyboard.up("ArrowUp");
  await expect(page.getByRole("button", { name: "Tiến" })).toHaveAttribute("aria-pressed", "false");
  await expect(page.getByText("Đã dừng an toàn")).toBeVisible();

  await page.locator(".destination-select select").selectOption("DEST-001");
  await expect(page.locator(".map-route")).toBeVisible();
  await page.getByRole("button", { name: "Đi đến" }).click();
  await expect(page.getByRole("button", { name: "Huỷ hành trình" })).toBeVisible();

  await page.getByRole("button", { name: "Ngắt kết nối" }).click();
  await expect(page).toHaveURL(/\/robots/);
});

test("operator creates, edits and deletes an offline robot", async ({ page }) => {
  const address = `10.88.${Math.floor(Date.now() / 1000) % 200}.27`;
  await page.goto("/");
  await page.getByLabel("Email").fill("demo@rovera.local");
  await page.getByLabel("Mật khẩu", { exact: true }).fill("demo123");
  await page.getByRole("button", { name: "Đăng nhập", exact: true }).click();

  await page.getByRole("button", { name: "Thêm robot", exact: true }).click();
  await page.getByLabel("IP hoặc hostname robot").fill(address);
  await page.getByLabel("Tài khoản robot").fill("robot-operator");
  await page.getByLabel("Mật khẩu robot").fill("local-device-password");
  const create = page.waitForResponse(
    (response) => response.request().method() === "POST"
      && response.url().includes("/api/robots/quick-add"),
  );
  await page.getByRole("button", { name: "Thêm robot", exact: true }).click();
  const created = await create;
  expect(created.status()).toBe(201);
  const robotId = (await created.json()).robot_id as string;
  await expect(page.getByText("ĐÃ THÊM ROBOT")).toBeVisible();
  await page.getByRole("button", { name: "Xem danh sách robot" }).click();

  await page.getByPlaceholder("Tìm theo mã, tên hoặc khu vực…").fill(address);
  const robot = page.locator(".robot-row");
  await expect(robot).toContainText("Chờ robot chạy");
  await robot.getByRole("button", { name: /Sửa/ }).click();

  await page.getByLabel("Tên hiển thị").fill("Robot kiểm thử đã sửa");
  const update = page.waitForResponse(
    (response) => response.request().method() === "PATCH"
      && response.url().includes(`/api/robots/${robotId}`),
  );
  await page.getByRole("button", { name: "Lưu thay đổi" }).click();
  expect((await update).ok()).toBeTruthy();

  await page.getByRole("button", { name: "Xoá robot" }).click();
  const remove = page.waitForResponse(
    (response) => response.request().method() === "DELETE"
      && response.url().includes(`/api/robots/${robotId}`),
  );
  await page.getByRole("button", { name: "Nhấn lại để xác nhận xoá" }).click();
  expect((await remove).status()).toBe(204);
  await expect(page).toHaveURL(/\/robots/);
});
