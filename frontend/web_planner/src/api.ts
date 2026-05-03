import type { DemInfo, MissileProfile, PlannedPath, Point, TerminalLog } from "./types";

async function request<T>(url: string, options?: RequestInit): Promise<T> {
  const response = await fetch(url, {
    headers: {
      "Content-Type": "application/json",
      ...(options?.headers ?? {})
    },
    ...options
  });

  if (!response.ok) {
    let message = response.statusText;
    try {
      const payload = await response.json();
      message = payload.detail ?? message;
    } catch {
      // Keep the status text when the response is not JSON.
    }
    throw new Error(message);
  }

  return response.json() as Promise<T>;
}

export async function getDems(): Promise<string[]> {
  const payload = await request<{ dems: string[] }>("/api/dems");
  return payload.dems;
}

export async function getMissileProfiles(): Promise<MissileProfile[]> {
  const payload = await request<{ configs: MissileProfile[] }>("/api/missile-configs");
  return payload.configs;
}

export async function getDemInfo(demName: string): Promise<DemInfo> {
  return request<DemInfo>(`/api/dem-info/${encodeURIComponent(demName)}`);
}

export async function planPath(input: {
  runId: string;
  demName: string;
  start: Point;
  target: Point;
  waypoints: Point[];
  heuristicWeight: number;
  minAltitude: number;
  signal?: AbortSignal;
}): Promise<PlannedPath> {
  return request<PlannedPath>("/api/plan-path", {
    method: "POST",
    signal: input.signal,
    body: JSON.stringify({
      run_id: input.runId,
      dem_name: input.demName,
      start: input.start,
      target: input.target,
      waypoints: input.waypoints,
      heuristic_weight: input.heuristicWeight,
      min_altitude: input.minAltitude
    })
  });
}

export async function getPathfindingTerminal(after = 0): Promise<TerminalLog[]> {
  const payload = await request<{ logs: TerminalLog[] }>(`/api/pathfinding-terminal?after=${after}`);
  return payload.logs;
}

export async function cancelPathfinding(runId: string): Promise<void> {
  await request<{ cancelled: boolean; run_id: string }>(`/api/cancel-pathfinding/${encodeURIComponent(runId)}`, {
    method: "POST"
  });
}
