import { hasPermission } from "../src/utils/permissions";

describe("map permissions", () => {
  it("allows technical accounts that receive maps.view", () => {
    expect(hasPermission({ permissions: ["robots.view", "maps.view"] }, "maps.view")).toBe(true);
  });

  it("does not grant Maps access to passenger accounts", () => {
    expect(hasPermission({ permissions: ["robots.view", "robots.operate"] }, "maps.view")).toBe(false);
    expect(hasPermission(null, "maps.view")).toBe(false);
  });
});
