import { translate } from "../src/i18n/translations";

describe("interface translations", () => {
  it("renders the Japanese interface catalogue", () => {
    expect(translate("ja", "Đăng nhập")).toBe("ログイン");
    expect(translate("ja", "Điều khiển")).toBe("操作");
    expect(translate("ja", "Loa phát đàm thoại")).toBe("通話用スピーカー");
  });

  it("renders the additional interface catalogues", () => {
    expect(translate("fr", "Đăng nhập")).toBe("Se connecter");
    expect(translate("de", "Ngôn ngữ")).toBe("Sprache");
    expect(translate("es", "Đăng xuất")).toBe("Cerrar sesión");
    expect(translate("ru", "Mật khẩu")).toBe("Пароль");
    expect(translate("zh", "Điều khiển robot")).not.toBe("Robot controls");
    expect(translate("ko", "Điều khiển robot")).not.toBe("Robot controls");
    expect(translate("th", "Điều khiển robot")).not.toBe("Robot controls");
    expect(translate("zh", "Quản trị hệ thống")).toBe("系统管理员");
    expect(translate("zh", "Chưa phân khu")).toBe("未分区");
  });

  it("interpolates translated values", () => {
    expect(translate("ja", "Hiển thị {shown} trong tổng số {total} robot", {
      shown: 3,
      total: 8,
    })).toBe("全8台中3台を表示");
  });
});
