import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { describe, expect, it } from "vitest";

describe("robot editor and configuration UX", () => {
  it("uses shared navigation and real active maps instead of a hardcoded option", () => {
    const editor = readFileSync(resolve("src/pages/RobotEditorPage.tsx"), "utf8");
    expect(editor).toContain("<OperationsShell");
    expect(editor).toContain("api.maps()");
    expect(editor).toContain("assignableMaps.map");
    expect(editor).not.toContain('<option value="MAP-001">');
    expect(editor).not.toContain("robot-editor-guide");
  });

  it("assigns maps and opens camera preview in a centered dialog", () => {
    const configuration = readFileSync(resolve("src/pages/RobotConfigurationPage.tsx"), "utf8");
    expect(configuration).toContain("<OperationsShell");
    expect(configuration).toContain("api.maps()");
    expect(configuration).toContain("robot-map-assignment");
    expect(configuration).toContain("api.updateRobot(robotId");
    expect(configuration).toContain('useState<"connection" | "video" | "audio">("connection")');
    expect(configuration).toContain('className="video-preview-backdrop"');
    expect(configuration).toContain('role="dialog" aria-modal="true"');
    expect(configuration).not.toContain("configuration-summary");
  });

  it("routes short success feedback through the global fading toast", () => {
    const editor = readFileSync(resolve("src/pages/RobotEditorPage.tsx"), "utf8");
    const configuration = readFileSync(resolve("src/pages/RobotConfigurationPage.tsx"), "utf8");
    const toast = readFileSync(resolve("src/components/ToastViewport.tsx"), "utf8");
    expect(editor).toContain("showToast(");
    expect(configuration).toContain("showToast(");
    expect(toast).toContain("2600");
    expect(toast).toContain('aria-live="polite"');
  });

  it("keeps configuration typography and buttons aligned with the robot editor", () => {
    const styles = readFileSync(resolve("src/styles.css"), "utf8");
    const operationsStyles = readFileSync(resolve("src/operations-shell.css"), "utf8");

    expect(styles).toMatch(/\.configuration-tabs button \{[\s\S]*?min-height: 40px;[\s\S]*?font-size: 9px;/);
    expect(styles).toMatch(/\.configuration-form \.button \{[\s\S]*?min-height: 40px;[\s\S]*?font-size: 9px;/);
    expect(styles).toMatch(/\.source-scan-button \{[\s\S]*?font-size: 9px;/);
    expect(operationsStyles).toContain(".robot-map-assignment__copy strong { color: #172b46; font-size: 10px; }");
    expect(operationsStyles).toContain(".robot-map-assignment__copy small { color: #66788f; font-size: 8px;");
  });
});
