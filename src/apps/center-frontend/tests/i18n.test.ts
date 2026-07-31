import { translate } from "../src/i18n/translations";

describe("interface translations", () => {
  it("renders the Japanese interface catalogue", () => {
    expect(translate("ja", "Đăng nhập")).toBe("ログイン");
    expect(translate("ja", "Điều khiển")).toBe("操作");
    expect(translate("ja", "Loa phát đàm thoại")).toBe("通話用スピーカー");
  });

  it("uses English as the safe fallback for languages without a UI catalogue", () => {
    expect(translate("fr", "Đăng nhập")).toBe("Sign in");
    expect(translate("fr", "Phát âm kiểm tra loa")).toBe("Play speaker test tone");
  });

  it("interpolates translated values", () => {
    expect(translate("ja", "Hiển thị {shown} trong tổng số {total} robot", {
      shown: 3,
      total: 8,
    })).toBe("全8台中3台を表示");
  });
});
