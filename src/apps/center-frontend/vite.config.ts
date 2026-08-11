import { defineConfig } from "vitest/config";
import react from "@vitejs/plugin-react";

const backendPort = process.env.CENTER_BACKEND_PORT ?? "8888";
const backendHttpUrl = `http://127.0.0.1:${backendPort}`;
const backendWebSocketUrl = `ws://127.0.0.1:${backendPort}`;

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      "/api": backendHttpUrl,
      "/health": backendHttpUrl,
      // Keep React's /maps, /maps/create and /maps/:id routes in Vite. Only
      // legacy static artifacts such as /maps/map-001.svg belong to Center.
      "^/maps/.+\\.[^/]+$": backendHttpUrl,
      "/ws": { target: backendWebSocketUrl, ws: true },
    },
  },
  test: {
    environment: "jsdom",
    setupFiles: "./tests/setup.ts",
    globals: true,
  },
});
