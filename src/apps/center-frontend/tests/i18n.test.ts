import { translate } from "../src/i18n/translations";
import {
  MAP_LOCALES, MAP_TRANSLATION_KEYS, MAP_TRANSLATIONS,
} from "../src/i18n/mapTranslations";

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

  it("provides a complete Map catalogue for every supported locale", () => {
    expect(MAP_TRANSLATION_KEYS.length).toBeGreaterThanOrEqual(258);
    const expectedKeys = [...MAP_TRANSLATION_KEYS].sort();
    const vietnameseText = /[ăâđêôơưáàảãạấầẩẫậắằẳẵặéèẻẽẹếềểễệíìỉĩịóòỏõọốồổỗộớờởỡợúùủũụứừửữựýỳỷỹỵ]/i;

    for (const locale of MAP_LOCALES) {
      const catalogue = MAP_TRANSLATIONS[locale];
      expect(Object.keys(catalogue).sort()).toEqual(expectedKeys);
      for (const source of MAP_TRANSLATION_KEYS) {
        const translated = catalogue[source];
        expect(translated.trim()).not.toBe("");
        expect(translated).not.toMatch(/ZXQ|⟦|⟧/);
        if (vietnameseText.test(source)) expect(translate(locale, source)).not.toBe(source);

        const sourceVariables = [...source.matchAll(/\{([^}]+)\}/g)].map((match) => match[1]).sort();
        const translatedVariables = [...translated.matchAll(/\{([^}]+)\}/g)].map((match) => match[1]).sort();
        expect(translatedVariables).toEqual(sourceVariables);
      }
    }
  });

  it("uses professional English labels throughout the Map workflow", () => {
    expect(translate("en", "Bản đồ")).toBe("Maps");
    expect(translate("en", "Tạo bản đồ")).toBe("Create map");
    expect(translate("en", "Danh mục vận hành")).toBe("Operations menu");
    expect(translate("en", "Tổng quan")).toBe("Overview");
    expect(translate("en", "Chờ đồng bộ")).toBe("Pending sync");
    expect(translate("en", "Thu gọn menu")).toBe("Collapse menu");
    expect(translate("en", "Đang kích hoạt · v{version}", { version: 4 })).toBe("Active · v4");
  });
});
