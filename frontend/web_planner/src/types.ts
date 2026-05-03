export type Point = {
  lat: number;
  lon: number;
};

export type PathPoint = [number, number, number];

export type DemInfo = {
  name: string;
  bounds: [number, number, number, number];
  center: [number, number];
  shape?: [number, number];
  nodata?: number | null;
};

export type MissileProfile = {
  name: string;
  cruise_speed: number;
  min_speed: number;
  max_speed: number;
  max_acceleration: number;
  min_altitude: number;
  max_altitude: number;
  max_g_force: number;
  sustained_turn_rate: number;
  sustained_g_force: number;
  evasive_turn_rate: number;
};

export type Mission = {
  id: string;
  name: string;
  demName: string;
  profileName: string;
  customProfile?: MissileProfile;
  start: Point | null;
  target: Point | null;
  waypoints: Point[];
  heuristicWeight: number;
  createdAt: string;
  updatedAt: string;
};

export type PlannedPath = {
  run_id?: string;
  path: PathPoint[];
  bounds: [number, number, number, number];
};

export type TerminalLog = {
  id: number;
  time: string;
  stream: string;
  run_id?: string | null;
  message: string;
};

export type SensorEstimate = {
  name: string;
  point: Point;
  altitude: number;
  errorMeters: number;
  confidence: number;
  freshnessMs: number;
};

export type TrackingState = {
  index: number;
  current: PathPoint | null;
  trace: PathPoint[];
  sources: SensorEstimate[];
  velocity: [number, number, number];
  covariance: number[];
  tercomCorrelation: number;
  sensedPatch: number[][];
  databasePatch: number[][];
};
