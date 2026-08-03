import { afterEach, describe, expect, it, vi } from "vitest";
import { createUuid } from "../src/utils/uuid";

describe("createUuid", () => {
  afterEach(() => vi.unstubAllGlobals());

  it("uses the browser UUID implementation when available", () => {
    const randomUUID = vi.fn(() => "123e4567-e89b-42d3-a456-426614174000");
    vi.stubGlobal("crypto", { randomUUID });

    expect(createUuid()).toBe("123e4567-e89b-42d3-a456-426614174000");
    expect(randomUUID).toHaveBeenCalledOnce();
  });

  it("creates an RFC 4122 UUID on plain HTTP LAN origins", () => {
    const getRandomValues = vi.fn((bytes: Uint8Array) => {
      bytes.forEach((_, index) => { bytes[index] = index; });
      return bytes;
    });
    vi.stubGlobal("crypto", { getRandomValues });

    const uuid = createUuid();

    expect(uuid).toBe("00010203-0405-4607-8809-0a0b0c0d0e0f");
    expect(uuid).toMatch(/^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/);
    expect(getRandomValues).toHaveBeenCalledOnce();
  });
});
