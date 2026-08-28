import { readFileSync, readdirSync, statSync } from "node:fs";
import { resolve } from "node:path";
import * as ts from "typescript";
import { SUPPORTED_LANGUAGE_CODES } from "../src/data/languages";
import { hasTranslation, translate } from "../src/i18n/translations";
import {
  MAP_LOCALES, MAP_TRANSLATION_KEYS, MAP_TRANSLATIONS,
} from "../src/i18n/mapTranslations";

describe("interface translations", () => {
  it("has a native translation for every literal passed to t()", () => {
    const sourceRoot = resolve(process.cwd(), "src");
    const files: string[] = [];
    const visit = (directory: string) => {
      for (const name of readdirSync(directory)) {
        const path = resolve(directory, name);
        if (statSync(path).isDirectory()) visit(path);
        else if (/\.(?:ts|tsx)$/.test(name)) files.push(path);
      }
    };
    visit(sourceRoot);
    const keys = new Set<string>();
    const untranslatedJsx: string[] = [];
    for (const file of files) {
      const source = readFileSync(file, "utf8");
      for (const match of source.matchAll(/\bt\(\s*["`]([^"`]+)["`]/g)) {
        if (!match[1].includes("${")) keys.add(match[1]);
      }
      const ast = ts.createSourceFile(file, source, ts.ScriptTarget.Latest, false, ts.ScriptKind.TSX);
      const inspect = (node: ts.Node) => {
        if (ts.isJsxText(node)
          && /[ăâđêôơưĂÂĐÊÔƠƯáàảãạấầẩẫậắằẳẵặéèẻẽẹếềểễệíìỉĩịóòỏõọốồổỗộớờởỡợúùủũụứừửữựýỳỷỹỵ]/iu.test(node.text)) {
          untranslatedJsx.push(`${file.replace(`${sourceRoot}/`, "")}: ${node.text.trim()}`);
        }
        ts.forEachChild(node, inspect);
      };
      inspect(ast);
    }
    const missing = [...keys].flatMap((key) => {
      const locales = SUPPORTED_LANGUAGE_CODES
        .filter((locale) => locale !== "vi" && !hasTranslation(locale, key));
      return locales.length > 0 ? [`${key} [${locales.join(", ")}]`] : [];
    });
    expect(missing).toEqual([]);
    expect(untranslatedJsx).toEqual([]);

    for (const locale of SUPPORTED_LANGUAGE_CODES.filter((code) => code !== "vi")) {
      for (const key of keys) {
        if (/[ăâđêôơưĂÂĐÊÔƠƯáàảãạấầẩẫậắằẳẵặéèẻẽẹếềểễệíìỉĩịóòỏõọốồổỗộớờởỡợúùủũụứừửữựýỳỷỹỵ]/iu.test(key)) {
          expect(translate(locale, key), `${locale}: ${key}`).not.toBe(key);
        }
        const sourceVariables = [...key.matchAll(/\{([^}]+)\}/g)].map((match) => match[1]).sort();
        const translatedVariables = [...translate(locale, key).matchAll(/\{([^}]+)\}/g)]
          .map((match) => match[1]).sort();
        expect(translatedVariables, `${locale}: ${key}`).toEqual(sourceVariables);
      }
    }
  });

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
    expect(translate("ko", "Hành trình")).toBe("경로 안내");
    expect(translate("ko", "Điểm đến đã lưu")).toBe("저장된 목적지");
    expect(translate("ko", "Đồng hồ sensor")).toBe("센서 시간");
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
