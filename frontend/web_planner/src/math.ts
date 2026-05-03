import type { PathPoint, Point } from "./types";

const EARTH_RADIUS_M = 6371000;

export function formatCoordinate(value: number): string {
  return Number.isFinite(value) ? value.toFixed(6) : "-";
}

export function formatMeters(value: number): string {
  if (!Number.isFinite(value)) return "-";
  if (Math.abs(value) >= 1000) return `${(value / 1000).toFixed(1)} km`;
  return `${value.toFixed(0)} m`;
}

export function formatSpeedKmh(value: number): string {
  return Number.isFinite(value) ? `${value.toFixed(0)} km/h` : "-";
}

export function haversineDistance(a: Point, b: Point): number {
  const lat1 = toRad(a.lat);
  const lat2 = toRad(b.lat);
  const dLat = toRad(b.lat - a.lat);
  const dLon = toRad(b.lon - a.lon);
  const sinLat = Math.sin(dLat / 2);
  const sinLon = Math.sin(dLon / 2);
  const h = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon;
  return 2 * EARTH_RADIUS_M * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

export function pathDistance(path: PathPoint[]): number {
  let total = 0;
  for (let index = 1; index < path.length; index += 1) {
    total += haversineDistance(
      { lat: path[index - 1][0], lon: path[index - 1][1] },
      { lat: path[index][0], lon: path[index][1] }
    );
  }
  return total;
}

export function pathBounds(points: Point[]): [[number, number], [number, number]] | null {
  if (!points.length) return null;
  const lats = points.map((point) => point.lat);
  const lons = points.map((point) => point.lon);
  return [
    [Math.min(...lats), Math.min(...lons)],
    [Math.max(...lats), Math.max(...lons)]
  ];
}

export function interpolatePath(path: PathPoint[], index: number): PathPoint | null {
  if (!path.length) return null;
  const clamped = Math.max(0, Math.min(path.length - 1, index));
  return path[Math.floor(clamped)];
}

export function deterministicNoise(seed: number, scale: number): number {
  const x = Math.sin(seed * 12.9898) * 43758.5453;
  return (x - Math.floor(x) - 0.5) * scale;
}

function toRad(value: number): number {
  return (value * Math.PI) / 180;
}
