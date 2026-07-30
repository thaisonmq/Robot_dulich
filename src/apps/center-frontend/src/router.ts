import { useCallback, useSyncExternalStore } from "react";

const listeners = new Set<() => void>();
const notify = () => listeners.forEach((listener) => listener());

if (typeof window !== "undefined") {
  window.addEventListener("popstate", notify);
}

export function navigate(path: string, options: { replace?: boolean } = {}): void {
  if (options.replace) window.history.replaceState(null, "", path);
  else window.history.pushState(null, "", path);
  notify();
}

export function usePathname(): string {
  return useSyncExternalStore(
    (listener) => {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    () => window.location.pathname,
    () => "/",
  );
}

export function useNavigate() {
  return useCallback((path: string, options?: { replace?: boolean }) => {
    navigate(path, options);
  }, []);
}

export function useParams(): { robotId?: string } {
  const pathname = usePathname();
  const match = pathname.match(
    /^\/(?:control|robots)\/([^/]+)(?:\/(?:configuration|edit))?$/
  );
  return { robotId: match ? decodeURIComponent(match[1]) : undefined };
}
