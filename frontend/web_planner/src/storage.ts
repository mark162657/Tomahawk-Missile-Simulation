import type { Mission, MissileProfile } from "./types";

const MISSIONS_KEY = "mg:web-planner:missions";
const CUSTOM_PROFILES_KEY = "mg:web-planner:custom-profiles";
const THEME_KEY = "mg:web-planner:theme";

function readJson<T>(key: string, fallback: T): T {
  try {
    const raw = localStorage.getItem(key);
    return raw ? (JSON.parse(raw) as T) : fallback;
  } catch {
    return fallback;
  }
}

export function loadMissions(): Mission[] {
  return readJson<Mission[]>(MISSIONS_KEY, []);
}

export function saveMissions(missions: Mission[]): void {
  localStorage.setItem(MISSIONS_KEY, JSON.stringify(missions));
}

export function upsertMission(mission: Mission): Mission[] {
  const missions = loadMissions();
  const index = missions.findIndex((item) => item.id === mission.id);
  const next = index >= 0 ? [...missions.slice(0, index), mission, ...missions.slice(index + 1)] : [mission, ...missions];
  saveMissions(next);
  return next;
}

export function deleteMission(id: string): Mission[] {
  const next = loadMissions().filter((mission) => mission.id !== id);
  saveMissions(next);
  return next;
}

export function loadCustomProfiles(): MissileProfile[] {
  return readJson<MissileProfile[]>(CUSTOM_PROFILES_KEY, []);
}

export function saveCustomProfiles(profiles: MissileProfile[]): void {
  localStorage.setItem(CUSTOM_PROFILES_KEY, JSON.stringify(profiles));
}

export function loadTheme(): "light" | "dark" {
  const stored = localStorage.getItem(THEME_KEY);
  if (stored === "light" || stored === "dark") return stored;
  return window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}

export function saveTheme(theme: "light" | "dark"): void {
  localStorage.setItem(THEME_KEY, theme);
}
