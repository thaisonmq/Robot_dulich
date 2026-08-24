import type { Route } from "../types";

/**
 * Keep a browser selection only while it still belongs to the exact preview
 * mission being started. A relocalization can invalidate and recompute the
 * preview inside the same click; carrying the old route id into that new
 * mission makes the server correctly reject it as non-authoritative.
 */
export function authoritativeRouteId(
  prepared: Route,
  displayed: Route | null,
  selectedRouteId: string,
): string {
  const candidateIds = new Set(
    (prepared.candidates ?? [])
      .filter((candidate) => candidate.valid !== false)
      .map((candidate) => candidate.route_id),
  );
  candidateIds.add(prepared.route_id);
  if (prepared.selected_route_id) candidateIds.add(prepared.selected_route_id);

  if (
    selectedRouteId
    && prepared.mission_id
    && prepared.mission_id === displayed?.mission_id
    && candidateIds.has(selectedRouteId)
  ) {
    return selectedRouteId;
  }
  return prepared.selected_route_id
    || prepared.candidates?.find((candidate) => candidate.valid !== false && candidate.recommended)?.route_id
    || prepared.candidates?.find((candidate) => candidate.valid !== false)?.route_id
    || prepared.route_id;
}
