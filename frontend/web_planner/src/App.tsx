import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import L from "leaflet";
import {
  Activity,
  ChevronDown,
  Crosshair,
  Gauge,
  Layers,
  LocateFixed,
  MapPin,
  Moon,
  Navigation,
  Pause,
  Play,
  Plus,
  Radar,
  RotateCcw,
  Route,
  Save,
  Sun,
  Target,
  Trash2,
  UploadCloud,
  Waypoints
} from "lucide-react";
import { cancelPathfinding, getDemInfo, getDems, getMissileProfiles, getPathfindingTerminal, planPath } from "./api";
import {
  deleteMission,
  loadCustomProfiles,
  loadMissions,
  loadTheme,
  saveCustomProfiles,
  saveTheme,
  upsertMission
} from "./storage";
import {
  deterministicNoise,
  formatCoordinate,
  formatMeters,
  formatSpeedKmh,
  haversineDistance,
  pathDistance
} from "./math";
import type { DemInfo, Mission, MissileProfile, PathPoint, PlannedPath, Point, SensorEstimate, TerminalLog, TrackingState } from "./types";

type Tab = "planner" | "tracking";
type PlacementMode = "none" | "start" | "target" | "waypoint";
type Basemap = "terrain" | "street" | "satellite" | "topo";
type LatHemisphere = "N" | "S";
type LonHemisphere = "E" | "W";
type CoordinateAxis = "lat" | "lon";

const DEFAULT_PROFILE: MissileProfile = {
  name: "Custom profile",
  cruise_speed: 800,
  min_speed: 400,
  max_speed: 920,
  max_acceleration: 9.8,
  min_altitude: 30,
  max_altitude: 1200,
  max_g_force: 6.8,
  sustained_turn_rate: 8,
  sustained_g_force: 2,
  evasive_turn_rate: 25
};

const PROFILE_FIELDS: Array<[keyof MissileProfile, string, string]> = [
  ["cruise_speed", "Cruise", "km/h"],
  ["min_speed", "Min speed", "km/h"],
  ["max_speed", "Max speed", "km/h"],
  ["max_acceleration", "Acceleration", "m/s2"],
  ["min_altitude", "Min AGL", "m"],
  ["max_altitude", "Max AGL", "m"],
  ["max_g_force", "Max G", "g"],
  ["sustained_turn_rate", "Sustained turn", "deg/s"],
  ["sustained_g_force", "Sustained G", "g"],
  ["evasive_turn_rate", "Evasive turn", "deg/s"]
];

const BASEMAPS: Record<Basemap, { label: string; url: string; attribution: string; maxZoom: number }> = {
  terrain: {
    label: "Terrain",
    url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Shaded_Relief/MapServer/tile/{z}/{y}/{x}",
    attribution: "Terrain: Esri, USGS, SRTM, GTOPO30, NED",
    maxZoom: 17
  },
  street: {
    label: "Street",
    url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    attribution: "Map data: OpenStreetMap contributors",
    maxZoom: 19
  },
  satellite: {
    label: "Satellite",
    url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attribution: "Tiles: Esri, Maxar, Earthstar Geographics",
    maxZoom: 18
  },
  topo: {
    label: "Topo",
    url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
    attribution: "Map data: OpenStreetMap, SRTM | Style: OpenTopoMap",
    maxZoom: 17
  }
};

function createMission(demName = "", profileName = DEFAULT_PROFILE.name): Mission {
  const now = new Date().toISOString();
  return {
    id: crypto.randomUUID(),
    name: "New mission",
    demName,
    profileName,
    start: null,
    target: null,
    waypoints: [],
    heuristicWeight: 1.5,
    createdAt: now,
    updatedAt: now
  };
}

function cloneProfile(profile: MissileProfile, name = "Custom profile"): MissileProfile {
  return { ...profile, name };
}

function pointFromPath(point: PathPoint | null): Point | null {
  return point ? { lat: point[0], lon: point[1] } : null;
}

function markerIcon(kind: "start" | "target" | "waypoint" | "missile" | "source", label?: string): L.DivIcon {
  return L.divIcon({
    className: "",
    html: `<div class="map-marker ${kind}">${label ?? ""}</div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14]
  });
}

function coordinateMagnitude(point: Point | null, axis: CoordinateAxis): string {
  if (!point) return "";
  const value = axis === "lat" ? point.lat : point.lon;
  return Number.isFinite(value) ? String(Math.abs(value)) : "";
}

function coordinateHemisphere(point: Point | null, axis: CoordinateAxis): LatHemisphere | LonHemisphere {
  const value = point ? (axis === "lat" ? point.lat : point.lon) : 0;
  if (axis === "lat") return value < 0 ? "S" : "N";
  return value < 0 ? "W" : "E";
}

function signedCoordinate(value: string | number, hemisphere: LatHemisphere | LonHemisphere): number {
  const magnitude = Math.abs(Number(value));
  if (!Number.isFinite(magnitude)) return 0;
  return hemisphere === "S" || hemisphere === "W" ? -magnitude : magnitude;
}

export default function App() {
  const [theme, setTheme] = useState<"light" | "dark">(() => loadTheme());
  const [tab, setTab] = useState<Tab>("planner");
  const [basemap, setBasemap] = useState<Basemap>("terrain");
  const [dems, setDems] = useState<string[]>([]);
  const [profiles, setProfiles] = useState<MissileProfile[]>([]);
  const [customProfiles, setCustomProfiles] = useState<MissileProfile[]>(() => loadCustomProfiles());
  const [missions, setMissions] = useState<Mission[]>(() => loadMissions());
  const [mission, setMission] = useState<Mission>(() => createMission());
  const [demInfo, setDemInfo] = useState<DemInfo | null>(null);
  const [plannedPath, setPlannedPath] = useState<PlannedPath | null>(null);
  const [placementMode, setPlacementMode] = useState<PlacementMode>("none");
  const [status, setStatus] = useState("Loading terrain catalog");
  const [planning, setPlanning] = useState(false);
  const [trackingActive, setTrackingActive] = useState(false);
  const [trackingSpeed, setTrackingSpeed] = useState(4);
  const [trackingState, setTrackingState] = useState<TrackingState>(() => makeTrackingState([], 0));
  const [terminalLogs, setTerminalLogs] = useState<TerminalLog[]>([]);
  const [activePathRunId, setActivePathRunId] = useState<string | null>(null);
  const terminalLastIdRef = useRef(0);
  const terminalPollInFlightRef = useRef(false);
  const terminalSeenIdsRef = useRef<Set<number>>(new Set());
  const pathAbortRef = useRef<AbortController | null>(null);

  const allProfiles = useMemo(() => [...profiles, ...customProfiles], [profiles, customProfiles]);

  const selectedProfile = useMemo(() => {
    if (mission.customProfile) return mission.customProfile;
    return allProfiles.find((profile) => profile.name === mission.profileName) ?? allProfiles[0] ?? DEFAULT_PROFILE;
  }, [allProfiles, mission.customProfile, mission.profileName]);

  useEffect(() => {
    document.documentElement.dataset.theme = theme;
    saveTheme(theme);
  }, [theme]);

  useEffect(() => {
    let cancelled = false;
    Promise.all([getDems(), getMissileProfiles()])
      .then(([demList, profileList]) => {
        if (cancelled) return;
        const preferredDem = demList.includes("srtm_43_02.tif") ? "srtm_43_02.tif" : demList[0] || "";
        setDems(demList);
        setProfiles(profileList);
        setMission((current) => ({
          ...current,
          demName: current.demName || preferredDem,
          profileName: current.profileName || profileList[0]?.name || DEFAULT_PROFILE.name
        }));
        setStatus(demList.length ? "Terrain catalog ready" : "No DEM files found in data/dem");
      })
      .catch((error) => setStatus(`Startup error: ${error.message}`));
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    if (!mission.demName) return;
    let cancelled = false;
    getDemInfo(mission.demName)
      .then((info) => {
        if (!cancelled) setDemInfo(info);
      })
      .catch((error) => {
        if (!cancelled) setStatus(`DEM error: ${error.message}`);
      });
    return () => {
      cancelled = true;
    };
  }, [mission.demName]);

  useEffect(() => {
    let cancelled = false;
    const refreshTerminal = async () => {
      if (terminalPollInFlightRef.current) return;
      terminalPollInFlightRef.current = true;
      try {
        const logs = await getPathfindingTerminal(terminalLastIdRef.current);
        if (cancelled || !logs.length) return;
        const freshLogs = logs.filter((log) => !terminalSeenIdsRef.current.has(log.id));
        if (!freshLogs.length) return;
        freshLogs.forEach((log) => terminalSeenIdsRef.current.add(log.id));
        terminalLastIdRef.current = Math.max(terminalLastIdRef.current, ...freshLogs.map((log) => log.id));
        setTerminalLogs((current) => {
          const merged = [...current, ...freshLogs].slice(-240);
          terminalSeenIdsRef.current = new Set(merged.map((log) => log.id));
          return merged;
        });
      } catch {
        // The web terminal is best-effort; keep the control panel usable if polling misses a beat.
      } finally {
        terminalPollInFlightRef.current = false;
      }
    };

    refreshTerminal();
    const interval = window.setInterval(refreshTerminal, 1000);
    return () => {
      cancelled = true;
      window.clearInterval(interval);
    };
  }, []);

  useEffect(() => {
    if (!trackingActive || !plannedPath?.path.length) return;
    const interval = window.setInterval(() => {
      setTrackingState((current) => {
        const nextIndex = Math.min(plannedPath.path.length - 1, current.index + Math.max(1, Math.round(trackingSpeed)));
        if (nextIndex >= plannedPath.path.length - 1) {
          setTrackingActive(false);
        }
        return makeTrackingState(plannedPath.path, nextIndex);
      });
    }, 125);
    return () => window.clearInterval(interval);
  }, [plannedPath, trackingActive, trackingSpeed]);

  const updateMission = useCallback((patch: Partial<Mission>) => {
    setMission((current) => ({ ...current, ...patch }));
  }, []);

  const setPoint = useCallback(
    (mode: PlacementMode, point: Point) => {
      if (mode === "start") updateMission({ start: point });
      if (mode === "target") updateMission({ target: point });
      if (mode === "waypoint") updateMission({ waypoints: [...mission.waypoints, point] });
      setPlacementMode("none");
    },
    [mission.waypoints, updateMission]
  );

  const saveCurrentMission = useCallback(() => {
    const saved = {
      ...mission,
      name: mission.name.trim() || "Untitled mission",
      updatedAt: new Date().toISOString()
    };
    setMission(saved);
    setMissions(upsertMission(saved));
    setStatus(`Saved ${saved.name}`);
  }, [mission]);

  const saveCurrentCustomProfile = useCallback(() => {
    const profile = cloneProfile(selectedProfile, selectedProfile.name.trim() || "Custom profile");
    const next = [profile, ...customProfiles.filter((item) => item.name.toLowerCase() !== profile.name.toLowerCase())];
    setCustomProfiles(next);
    saveCustomProfiles(next);
    updateMission({ profileName: profile.name, customProfile: undefined });
    setStatus(`Saved missile profile ${profile.name}`);
  }, [customProfiles, selectedProfile, updateMission]);

  const runPathfinding = useCallback(async () => {
    if (!mission.demName || !mission.start || !mission.target) {
      setStatus("Select a DEM, start point, and target before pathfinding");
      return;
    }
    const runId = crypto.randomUUID();
    const controller = new AbortController();
    pathAbortRef.current = controller;
    setActivePathRunId(runId);
    setPlanning(true);
    setStatus("Pathfinding running");
    try {
      const result = await planPath({
        runId,
        demName: mission.demName,
        start: mission.start,
        target: mission.target,
        waypoints: mission.waypoints,
        heuristicWeight: mission.heuristicWeight,
        minAltitude: selectedProfile.min_altitude,
        signal: controller.signal
      });
      if (controller.signal.aborted) return;
      setPlannedPath(result);
      setTrackingState(makeTrackingState(result.path, 0));
      setStatus(`Path ready: ${result.path.length.toLocaleString()} trajectory points`);
    } catch (error) {
      if ((error as Error).name === "AbortError") {
        setStatus("Pathfinding abort requested");
      } else {
        setStatus(`Pathfinding error: ${(error as Error).message}`);
      }
    } finally {
      if (pathAbortRef.current === controller) {
        pathAbortRef.current = null;
        setActivePathRunId(null);
        setPlanning(false);
      }
    }
  }, [mission, selectedProfile.min_altitude]);

  const abortPathfinding = useCallback(() => {
    if (!activePathRunId) return;
    setStatus("Pathfinding abort requested");
    cancelPathfinding(activePathRunId).catch(() => undefined);
    pathAbortRef.current?.abort();
    pathAbortRef.current = null;
    setActivePathRunId(null);
    setPlanning(false);
  }, [activePathRunId]);

  const resetTracking = useCallback(() => {
    setTrackingActive(false);
    setTrackingState(makeTrackingState(plannedPath?.path ?? [], 0));
  }, [plannedPath]);

  const telemetryPoint = pointFromPath(trackingState.current);

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark">
            <Navigation size={18} />
          </div>
          <div>
            <strong>Mission Planner</strong>
            <span>{status}</span>
          </div>
        </div>

        <nav className="tabs" aria-label="Primary">
          <button className={tab === "planner" ? "active" : ""} onClick={() => setTab("planner")}>
            <Route size={16} />
            Planner
          </button>
          <button className={tab === "tracking" ? "active" : ""} onClick={() => setTab("tracking")}>
            <Radar size={16} />
            Tracking
          </button>
        </nav>

        <div className="topbar-actions">
          <select value={basemap} onChange={(event) => setBasemap(event.target.value as Basemap)} aria-label="Basemap">
            {Object.entries(BASEMAPS).map(([key, value]) => (
              <option value={key} key={key}>
                {value.label}
              </option>
            ))}
          </select>
          <button className="icon-button" title="Toggle theme" onClick={() => setTheme(theme === "dark" ? "light" : "dark")}>
            {theme === "dark" ? <Sun size={17} /> : <Moon size={17} />}
          </button>
        </div>
      </header>

      <main className="workspace">
        {tab === "planner" ? (
          <>
            <PlannerPanel
              mission={mission}
              missions={missions}
              dems={dems}
              profiles={allProfiles}
              selectedProfile={selectedProfile}
              placementMode={placementMode}
              planning={planning}
              updateMission={updateMission}
              setMission={(next) => {
                setMission(next);
                setPlannedPath(null);
                setStatus(`Loaded ${next.name}`);
              }}
              setMissions={setMissions}
              setPlacementMode={setPlacementMode}
              saveMission={saveCurrentMission}
              saveCustomProfile={saveCurrentCustomProfile}
              runPathfinding={runPathfinding}
              abortPathfinding={abortPathfinding}
              terminalLogs={terminalLogs}
            />
            <section className="map-column">
              <MissionMap
                basemap={basemap}
                demInfo={demInfo}
                mission={mission}
                plannedPath={plannedPath?.path ?? []}
                trackingTrace={[]}
                trackingPoint={null}
                sourceEstimates={[]}
                placementMode={placementMode}
                onMapClick={(point) => setPoint(placementMode, point)}
              />
              <ElevationPanel path={plannedPath?.path ?? []} profile={selectedProfile} />
            </section>
          </>
        ) : (
          <>
            <TrackingPanel
              mission={mission}
              plannedPath={plannedPath?.path ?? []}
              profile={selectedProfile}
              active={trackingActive}
              speed={trackingSpeed}
              state={trackingState}
              setActive={setTrackingActive}
              setSpeed={setTrackingSpeed}
              reset={resetTracking}
              terminalLogs={terminalLogs}
            />
            <section className="map-column">
              <MissionMap
                basemap={basemap}
                demInfo={demInfo}
                mission={mission}
                plannedPath={plannedPath?.path ?? []}
                trackingTrace={trackingState.trace}
                trackingPoint={telemetryPoint}
                sourceEstimates={trackingState.sources}
                placementMode="none"
                onMapClick={() => undefined}
              />
              <TrackingDetails state={trackingState} path={plannedPath?.path ?? []} />
            </section>
          </>
        )}
      </main>
    </div>
  );
}

function PlannerPanel(props: {
  mission: Mission;
  missions: Mission[];
  dems: string[];
  profiles: MissileProfile[];
  selectedProfile: MissileProfile;
  placementMode: PlacementMode;
  planning: boolean;
  updateMission: (patch: Partial<Mission>) => void;
  setMission: (mission: Mission) => void;
  setMissions: (missions: Mission[]) => void;
  setPlacementMode: (mode: PlacementMode) => void;
  saveMission: () => void;
  saveCustomProfile: () => void;
  runPathfinding: () => void;
  abortPathfinding: () => void;
  terminalLogs: TerminalLog[];
}) {
  const {
    mission,
    missions,
    dems,
    profiles,
    selectedProfile,
    placementMode,
    planning,
    updateMission,
    setMission,
    setMissions,
    setPlacementMode,
    saveMission,
    saveCustomProfile,
    runPathfinding,
    abortPathfinding,
    terminalLogs
  } = props;

  const editProfile = mission.customProfile ?? selectedProfile;
  const [manualWaypoint, setManualWaypoint] = useState({
    lat: "",
    latHemisphere: "N" as LatHemisphere,
    lon: "",
    lonHemisphere: "E" as LonHemisphere
  });

  const updatePoint = (field: "start" | "target", axis: CoordinateAxis, value: string, hemisphere?: LatHemisphere | LonHemisphere) => {
    const previous = mission[field] ?? { lat: 0, lon: 0 };
    const nextHemisphere = hemisphere ?? coordinateHemisphere(previous, axis);
    updateMission({ [field]: { ...previous, [axis]: signedCoordinate(value, nextHemisphere) } });
  };

  const updateWaypoint = (index: number, axis: CoordinateAxis, value: string, hemisphere?: LatHemisphere | LonHemisphere) => {
    const waypoints = mission.waypoints.map((point, waypointIndex) =>
      waypointIndex === index ? { ...point, [axis]: signedCoordinate(value, hemisphere ?? coordinateHemisphere(point, axis)) } : point
    );
    updateMission({ waypoints });
  };

  const addManualWaypoint = () => {
    if (!manualWaypoint.lat.trim() || !manualWaypoint.lon.trim()) return;
    const point = {
      lat: signedCoordinate(manualWaypoint.lat, manualWaypoint.latHemisphere),
      lon: signedCoordinate(manualWaypoint.lon, manualWaypoint.lonHemisphere)
    };
    updateMission({ waypoints: [...mission.waypoints, point] });
    setManualWaypoint({ lat: "", latHemisphere: manualWaypoint.latHemisphere, lon: "", lonHemisphere: manualWaypoint.lonHemisphere });
  };

  const makeCustom = () => {
    updateMission({
      profileName: "Custom profile",
      customProfile: cloneProfile(selectedProfile, selectedProfile.name.includes("Custom") ? selectedProfile.name : `${selectedProfile.name} custom`)
    });
  };

  const updateProfileField = (key: keyof MissileProfile, value: string) => {
    const next = {
      ...editProfile,
      [key]: key === "name" ? value : Number(value)
    };
    updateMission({ profileName: next.name, customProfile: next });
  };

  return (
    <aside className="side-panel planner-panel">
      <section className="panel-section">
        <div className="section-heading">
          <h2>Mission</h2>
          <button className="icon-button" title="Save mission" onClick={saveMission}>
            <Save size={16} />
          </button>
        </div>
        <label>
          Name
          <input value={mission.name} onChange={(event) => updateMission({ name: event.target.value })} />
        </label>
        <div className="split-row">
          <label>
            DEM
            <select value={mission.demName} onChange={(event) => updateMission({ demName: event.target.value })}>
              {dems.map((dem) => (
                <option value={dem} key={dem}>
                  {dem}
                </option>
              ))}
            </select>
          </label>
          <label>
            Saved
            <select
              value=""
              onChange={(event) => {
                const selected = missions.find((item) => item.id === event.target.value);
                if (selected) setMission(selected);
              }}
            >
              <option value="">Load</option>
              {missions.map((item) => (
                <option value={item.id} key={item.id}>
                  {item.name}
                </option>
              ))}
            </select>
          </label>
        </div>
        <div className="button-row">
          <button onClick={saveMission}>
            <Save size={15} />
            Save
          </button>
          <button
            className="quiet"
            onClick={() => {
              const fresh = createMission(mission.demName, mission.profileName);
              setMission(fresh);
            }}
          >
            <Plus size={15} />
            New
          </button>
          <button
            className="quiet danger"
            disabled={!missions.some((item) => item.id === mission.id)}
            onClick={() => {
              setMissions(deleteMission(mission.id));
            }}
          >
            <Trash2 size={15} />
            Delete
          </button>
        </div>
      </section>

      <section className="panel-section">
        <div className="section-heading">
          <h2>Profile</h2>
          <Gauge size={16} />
        </div>
        <label>
          Missile
          <select
            value={mission.customProfile ? "__custom__" : mission.profileName}
            onChange={(event) => {
              if (event.target.value === "__custom__") {
                makeCustom();
              } else {
                updateMission({ profileName: event.target.value, customProfile: undefined });
              }
            }}
          >
            {profiles.map((profile) => (
              <option value={profile.name} key={profile.name}>
                {profile.name}
              </option>
            ))}
            <option value="__custom__">Custom profile</option>
          </select>
        </label>

        <div className="profile-summary">
          <span>{formatSpeedKmh(selectedProfile.cruise_speed)}</span>
          <span>{formatMeters(selectedProfile.min_altitude)} AGL</span>
          <span>{selectedProfile.max_g_force.toFixed(1)} g</span>
        </div>

        <details className="inline-details" open={Boolean(mission.customProfile)}>
          <summary>
            <ChevronDown size={15} />
            Edit profile
          </summary>
          <label>
            Profile name
            <input value={editProfile.name} onChange={(event) => updateProfileField("name", event.target.value)} />
          </label>
          <div className="field-grid">
            {PROFILE_FIELDS.map(([key, label, unit]) => (
              <label key={key}>
                {label}
                <span className="unit-input">
                  <input
                    type="number"
                    value={Number(editProfile[key]).toString()}
                    onChange={(event) => updateProfileField(key, event.target.value)}
                  />
                  <small>{unit}</small>
                </span>
              </label>
            ))}
          </div>
          <button onClick={saveCustomProfile}>
            <UploadCloud size={15} />
            Save profile
          </button>
        </details>
      </section>

      <section className="panel-section">
        <div className="section-heading">
          <h2>Coordinates</h2>
          <Crosshair size={16} />
        </div>
        <CoordinateEditor
          title="Start"
          point={mission.start}
          active={placementMode === "start"}
          onPlace={() => setPlacementMode(placementMode === "start" ? "none" : "start")}
          onChange={(axis, value, hemisphere) => updatePoint("start", axis, value, hemisphere)}
        />
        <CoordinateEditor
          title="Target"
          point={mission.target}
          active={placementMode === "target"}
          onPlace={() => setPlacementMode(placementMode === "target" ? "none" : "target")}
          onChange={(axis, value, hemisphere) => updatePoint("target", axis, value, hemisphere)}
        />
      </section>

      <section className="panel-section">
        <div className="section-heading">
          <h2>Waypoints</h2>
          <button
            className={placementMode === "waypoint" ? "icon-button active" : "icon-button"}
            title="Add waypoint from map"
            onClick={() => setPlacementMode(placementMode === "waypoint" ? "none" : "waypoint")}
          >
            <Waypoints size={16} />
          </button>
        </div>
        <div className="waypoint-list">
          <div className="manual-waypoint-row">
            <CoordinateField
              axis="lat"
              label="Lat"
              value={manualWaypoint.lat}
              hemisphere={manualWaypoint.latHemisphere}
              onValueChange={(value) => setManualWaypoint((current) => ({ ...current, lat: value }))}
              onHemisphereChange={(hemisphere) => setManualWaypoint((current) => ({ ...current, latHemisphere: hemisphere as LatHemisphere }))}
            />
            <CoordinateField
              axis="lon"
              label="Lon"
              value={manualWaypoint.lon}
              hemisphere={manualWaypoint.lonHemisphere}
              onValueChange={(value) => setManualWaypoint((current) => ({ ...current, lon: value }))}
              onHemisphereChange={(hemisphere) => setManualWaypoint((current) => ({ ...current, lonHemisphere: hemisphere as LonHemisphere }))}
            />
            <button
              title="Add waypoint manually"
              disabled={!manualWaypoint.lat.trim() || !manualWaypoint.lon.trim()}
              onClick={addManualWaypoint}
            >
              <Plus size={15} />
              Add
            </button>
          </div>
          {mission.waypoints.map((point, index) => (
            <div className="waypoint-row" key={`${point.lat}-${point.lon}-${index}`}>
              <span>{index + 1}</span>
              <CoordinateField
                axis="lat"
                label="Lat"
                value={coordinateMagnitude(point, "lat")}
                hemisphere={coordinateHemisphere(point, "lat")}
                onValueChange={(value) => updateWaypoint(index, "lat", value)}
                onHemisphereChange={(hemisphere) => updateWaypoint(index, "lat", coordinateMagnitude(point, "lat") || "0", hemisphere)}
              />
              <CoordinateField
                axis="lon"
                label="Lon"
                value={coordinateMagnitude(point, "lon")}
                hemisphere={coordinateHemisphere(point, "lon")}
                onValueChange={(value) => updateWaypoint(index, "lon", value)}
                onHemisphereChange={(hemisphere) => updateWaypoint(index, "lon", coordinateMagnitude(point, "lon") || "0", hemisphere)}
              />
              <button
                className="icon-button danger"
                title="Remove waypoint"
                onClick={() => updateMission({ waypoints: mission.waypoints.filter((_, itemIndex) => itemIndex !== index) })}
              >
                <Trash2 size={14} />
              </button>
            </div>
          ))}
          {!mission.waypoints.length && <div className="empty-state">Optional</div>}
        </div>
      </section>

      <section className="panel-section">
        <div className="section-heading">
          <h2>Pathfinding</h2>
          <Route size={16} />
        </div>
        <label>
          Heuristic weight
          <input
            type="range"
            min="1"
            max="4"
            step="0.1"
            value={mission.heuristicWeight}
            onChange={(event) => updateMission({ heuristicWeight: Number(event.target.value) })}
          />
        </label>
        <div className="metric-row">
          <span>Weight</span>
          <strong>{mission.heuristicWeight.toFixed(1)}</strong>
        </div>
        <div className="button-row">
          <button className="primary-action" disabled={planning} onClick={runPathfinding}>
            <Route size={16} />
            {planning ? "Planning" : "Run pathfinding"}
          </button>
          {planning && (
            <button className="quiet danger" onClick={abortPathfinding}>
              <Pause size={15} />
              Abort
            </button>
          )}
        </div>
      </section>
      <TerminalPanel logs={terminalLogs} />
    </aside>
  );
}

function CoordinateEditor(props: {
  title: string;
  point: Point | null;
  active: boolean;
  onPlace: () => void;
  onChange: (axis: CoordinateAxis, value: string, hemisphere: LatHemisphere | LonHemisphere) => void;
}) {
  const { title, point, active, onPlace, onChange } = props;
  const [latHemisphere, setLatHemisphere] = useState<LatHemisphere>(coordinateHemisphere(point, "lat") as LatHemisphere);
  const [lonHemisphere, setLonHemisphere] = useState<LonHemisphere>(coordinateHemisphere(point, "lon") as LonHemisphere);
  const latValue = coordinateMagnitude(point, "lat");
  const lonValue = coordinateMagnitude(point, "lon");

  useEffect(() => {
    if (!point) return;
    setLatHemisphere(coordinateHemisphere(point, "lat") as LatHemisphere);
    setLonHemisphere(coordinateHemisphere(point, "lon") as LonHemisphere);
  }, [point]);

  return (
    <div className="coordinate-editor">
      <div className="coordinate-title">
        <strong>{title}</strong>
        <button className={active ? "icon-button active" : "icon-button"} title={`Place ${title.toLowerCase()}`} onClick={onPlace}>
          <MapPin size={15} />
        </button>
      </div>
      <div className="coordinate-grid">
        <CoordinateField
          axis="lat"
          label="Lat"
          value={latValue}
          hemisphere={latHemisphere}
          onValueChange={(value) => onChange("lat", value, latHemisphere)}
          onHemisphereChange={(hemisphere) => {
            const next = hemisphere as LatHemisphere;
            setLatHemisphere(next);
            if (latValue) onChange("lat", latValue, next);
          }}
        />
        <CoordinateField
          axis="lon"
          label="Lon"
          value={lonValue}
          hemisphere={lonHemisphere}
          onValueChange={(value) => onChange("lon", value, lonHemisphere)}
          onHemisphereChange={(hemisphere) => {
            const next = hemisphere as LonHemisphere;
            setLonHemisphere(next);
            if (lonValue) onChange("lon", lonValue, next);
          }}
        />
      </div>
    </div>
  );
}

function CoordinateField(props: {
  axis: CoordinateAxis;
  label: string;
  value: string;
  hemisphere: LatHemisphere | LonHemisphere;
  onValueChange: (value: string) => void;
  onHemisphereChange: (hemisphere: LatHemisphere | LonHemisphere) => void;
}) {
  const { axis, label, value, hemisphere, onValueChange, onHemisphereChange } = props;
  const options = axis === "lat" ? ["N", "S"] : ["E", "W"];
  return (
    <label>
      {label}
      <span className="coordinate-input">
        <input type="number" min="0" step="0.0001" value={value} onChange={(event) => onValueChange(event.target.value)} />
        <select value={hemisphere} onChange={(event) => onHemisphereChange(event.target.value as LatHemisphere | LonHemisphere)}>
          {options.map((option) => (
            <option value={option} key={option}>
              {option}
            </option>
          ))}
        </select>
      </span>
    </label>
  );
}

function TerminalPanel(props: { logs: TerminalLog[] }) {
  const outputRef = useRef<HTMLDivElement | null>(null);
  const visibleLogs = props.logs.slice(-120);

  useEffect(() => {
    const node = outputRef.current;
    if (node) node.scrollTop = node.scrollHeight;
  }, [visibleLogs.length]);

  return (
    <section className="panel-section terminal-panel">
      <div className="section-heading">
        <h2>Terminal</h2>
        <Activity size={16} />
      </div>
      <div className="terminal-output" ref={outputRef}>
        {visibleLogs.length ? (
          visibleLogs.map((line) => (
            <div className={`terminal-line ${line.stream}`} key={line.id}>
              <span>{line.time}</span>
              <code>{line.message}</code>
            </div>
          ))
        ) : (
          <div className="terminal-line muted">
            <span>--:--:--</span>
            <code>Waiting for pathfinding_backend output</code>
          </div>
        )}
      </div>
    </section>
  );
}

function MissionMap(props: {
  basemap: Basemap;
  demInfo: DemInfo | null;
  mission: Mission;
  plannedPath: PathPoint[];
  trackingTrace: PathPoint[];
  trackingPoint: Point | null;
  sourceEstimates: SensorEstimate[];
  placementMode: PlacementMode;
  onMapClick: (point: Point) => void;
}) {
  const { basemap, demInfo, mission, plannedPath, trackingTrace, trackingPoint, sourceEstimates, placementMode, onMapClick } = props;
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const tileRef = useRef<L.TileLayer | null>(null);
  const staticLayersRef = useRef<L.LayerGroup | null>(null);
  const dynamicLayersRef = useRef<L.LayerGroup | null>(null);
  const fitKeyRef = useRef("");
  const clickRef = useRef(onMapClick);

  useEffect(() => {
    clickRef.current = onMapClick;
  }, [onMapClick]);

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;
    const map = L.map(containerRef.current, {
      zoomControl: false,
      preferCanvas: true,
      worldCopyJump: true
    }).setView([0, 0], 2);

    L.control.zoom({ position: "bottomright" }).addTo(map);
    staticLayersRef.current = L.layerGroup().addTo(map);
    dynamicLayersRef.current = L.layerGroup().addTo(map);
    map.on("click", (event) => {
      clickRef.current({ lat: event.latlng.lat, lon: event.latlng.lng });
    });
    mapRef.current = map;
  }, []);

  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    if (tileRef.current) {
      tileRef.current.removeFrom(map);
    }
    const tile = BASEMAPS[basemap];
    tileRef.current = L.tileLayer(tile.url, {
      maxZoom: tile.maxZoom,
      attribution: tile.attribution
    }).addTo(map);
  }, [basemap]);

  useEffect(() => {
    const map = mapRef.current;
    const layers = staticLayersRef.current;
    if (!map || !layers) return;
    layers.clearLayers();

    if (demInfo) {
      const [west, south, east, north] = demInfo.bounds;
      if (basemap === "terrain") {
        L.imageOverlay(
          `/api/dem-preview/${encodeURIComponent(demInfo.name)}?max_size=1600`,
          [
            [south, west],
            [north, east]
          ],
          {
            opacity: 0.86,
            interactive: false,
            className: "dem-elevation-overlay"
          }
        ).addTo(layers);
      }
      L.rectangle(
        [
          [south, west],
          [north, east]
        ],
        {
          color: "#16a34a",
          weight: 2,
          fillOpacity: 0.01,
          dashArray: "8 6"
        }
      )
        .bindTooltip(`DEM coverage: ${demInfo.name}`)
        .addTo(layers);
    }

    if (plannedPath.length) {
      L.polyline(
        plannedPath.map((point) => [point[0], point[1]]),
        {
          color: "#f59e0b",
          weight: 4,
          opacity: 0.95
        }
      ).addTo(layers);
    }

    if (mission.start) {
      L.marker([mission.start.lat, mission.start.lon], { icon: markerIcon("start", "S") })
        .bindTooltip("Start")
        .addTo(layers);
    }
    if (mission.target) {
      L.marker([mission.target.lat, mission.target.lon], { icon: markerIcon("target", "T") })
        .bindTooltip("Target")
        .addTo(layers);
    }
    mission.waypoints.forEach((point, index) => {
      L.marker([point.lat, point.lon], { icon: markerIcon("waypoint", String(index + 1)) })
        .bindTooltip(`Waypoint ${index + 1}`)
        .addTo(layers);
    });
  }, [basemap, demInfo, mission.start, mission.target, mission.waypoints, plannedPath]);

  useEffect(() => {
    const map = mapRef.current;
    const layers = dynamicLayersRef.current;
    if (!map || !layers) return;
    layers.clearLayers();

    if (trackingTrace.length > 1) {
      L.polyline(
        trackingTrace.map((point) => [point[0], point[1]]),
        {
          color: "#14b8a6",
          weight: 5,
          opacity: 0.95
        }
      ).addTo(layers);
    }

    sourceEstimates.forEach((source) => {
      L.circleMarker([source.point.lat, source.point.lon], {
        radius: 6,
        color: source.name === "Kalman" ? "#14b8a6" : "#38bdf8",
        weight: 2,
        fillOpacity: 0.18
      })
        .bindTooltip(`${source.name}: ${source.errorMeters.toFixed(1)} m`)
        .addTo(layers);
    });

    if (trackingPoint) {
      L.marker([trackingPoint.lat, trackingPoint.lon], { icon: markerIcon("missile", "") })
        .bindTooltip("Current position")
        .addTo(layers);
    }
  }, [sourceEstimates, trackingPoint, trackingTrace]);

  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    const key = plannedPath.length ? `path:${plannedPath.length}` : demInfo ? `dem:${demInfo.name}` : "";
    if (!key || fitKeyRef.current === key) return;
    fitKeyRef.current = key;

    if (plannedPath.length > 1) {
      const bounds = L.latLngBounds(plannedPath.map((point) => [point[0], point[1]]));
      map.fitBounds(bounds.pad(0.18), { animate: false });
      return;
    }

    if (demInfo) {
      const [west, south, east, north] = demInfo.bounds;
      map.fitBounds(
        [
          [south, west],
          [north, east]
        ],
        { animate: false }
      );
    }
  }, [demInfo, plannedPath]);

  return (
    <div className="map-wrap">
      <div ref={containerRef} className="mission-map" />
      <div className="map-hud left">
        <span className="hud-chip">
          <Layers size={14} />
          {BASEMAPS[basemap].label}
        </span>
        {demInfo && <span className="hud-chip">DEM {demInfo.bounds.map((value) => value.toFixed(2)).join(", ")}</span>}
      </div>
      {demInfo && basemap === "terrain" && (
        <div className="map-hud bottom-left">
          <div className="elevation-legend">
            <span>Low</span>
            <i />
            <span>High</span>
          </div>
        </div>
      )}
      {placementMode !== "none" && (
        <div className="map-hud center">
          <span className="hud-chip active">
            <LocateFixed size={14} />
            Place {placementMode}
          </span>
        </div>
      )}
    </div>
  );
}

function ElevationPanel(props: { path: PathPoint[]; profile: MissileProfile }) {
  const { path, profile } = props;
  const chart = useMemo(() => makeElevationChart(path), [path]);
  const distance = useMemo(() => pathDistance(path), [path]);
  const minAlt = chart?.minAlt ?? 0;
  const maxAlt = chart?.maxAlt ?? 0;

  return (
    <section className="bottom-panel">
      <div className="section-heading">
        <h2>Elevation</h2>
        <Activity size={16} />
      </div>
      {chart ? (
        <>
          <div className="elevation-grid">
            <Metric label="Path distance" value={formatMeters(distance)} />
            <Metric label="Points" value={path.length.toLocaleString()} />
            <Metric label="Min altitude" value={formatMeters(minAlt)} />
            <Metric label="Max altitude" value={formatMeters(maxAlt)} />
            <Metric label="Clearance target" value={`${profile.min_altitude.toFixed(0)} m AGL`} />
          </div>
          <svg className="elevation-chart" viewBox="0 0 640 120" preserveAspectRatio="none" role="img" aria-label="Trajectory altitude profile">
            <path d={chart.areaPath} className="chart-area" />
            <path d={chart.linePath} className="chart-line" />
          </svg>
        </>
      ) : (
        <div className="empty-state wide">Run pathfinding to draw the trajectory profile</div>
      )}
    </section>
  );
}

function TrackingPanel(props: {
  mission: Mission;
  plannedPath: PathPoint[];
  profile: MissileProfile;
  active: boolean;
  speed: number;
  state: TrackingState;
  setActive: (active: boolean) => void;
  setSpeed: (speed: number) => void;
  reset: () => void;
  terminalLogs: TerminalLog[];
}) {
  const { mission, plannedPath, profile, active, speed, state, setActive, setSpeed, reset, terminalLogs } = props;
  const current = state.current;
  const progress = plannedPath.length ? Math.round((state.index / Math.max(1, plannedPath.length - 1)) * 100) : 0;

  return (
    <aside className="side-panel tracking-panel">
      <section className="panel-section">
        <div className="section-heading">
          <h2>Track</h2>
          <Radar size={16} />
        </div>
        <div className="telemetry-hero">
          <span>{mission.name}</span>
          <strong>{current ? `${formatCoordinate(current[0])}, ${formatCoordinate(current[1])}` : "No active path"}</strong>
          <small>{current ? `${formatMeters(current[2])} MSL` : "Run a mission path first"}</small>
        </div>
        <div className="progress-track">
          <span style={{ width: `${progress}%` }} />
        </div>
        <div className="button-row">
          <button className="primary-action" disabled={!plannedPath.length} onClick={() => setActive(!active)}>
            {active ? <Pause size={16} /> : <Play size={16} />}
            {active ? "Pause" : "Run"}
          </button>
          <button className="quiet" disabled={!plannedPath.length} onClick={reset}>
            <RotateCcw size={15} />
            Reset
          </button>
        </div>
        <label>
          Update rate
          <input type="range" min="1" max="12" step="1" value={speed} onChange={(event) => setSpeed(Number(event.target.value))} />
        </label>
        <div className="metric-row">
          <span>Frontend simulation</span>
          <strong>{(speed * 8).toFixed(0)} Hz-equivalent</strong>
        </div>
      </section>

      <section className="panel-section">
        <div className="section-heading">
          <h2>Sources</h2>
          <Target size={16} />
        </div>
        <div className="source-list">
          {state.sources.map((source) => (
            <div className="source-row" key={source.name}>
              <div>
                <strong>{source.name}</strong>
                <span>
                  {formatCoordinate(source.point.lat)}, {formatCoordinate(source.point.lon)}
                </span>
              </div>
              <div>
                <strong>{source.confidence.toFixed(0)}%</strong>
                <span>{source.errorMeters.toFixed(1)} m</span>
              </div>
            </div>
          ))}
          {!state.sources.length && <div className="empty-state">Telemetry idle</div>}
        </div>
      </section>

      <section className="panel-section">
        <div className="section-heading">
          <h2>Missile</h2>
          <Gauge size={16} />
        </div>
        <div className="elevation-grid compact">
          <Metric label="Cruise" value={formatSpeedKmh(profile.cruise_speed)} />
          <Metric label="Max G" value={`${profile.max_g_force.toFixed(1)} g`} />
          <Metric label="Velocity N" value={`${state.velocity[0].toFixed(1)} m/s`} />
          <Metric label="Velocity E" value={`${state.velocity[1].toFixed(1)} m/s`} />
        </div>
      </section>
      <TerminalPanel logs={terminalLogs} />
    </aside>
  );
}

function TrackingDetails(props: { state: TrackingState; path: PathPoint[] }) {
  const { state, path } = props;
  const current = state.current;

  return (
    <section className="bottom-panel tracking-details">
      <div className="section-heading">
        <h2>Navigation Detail</h2>
        <Activity size={16} />
      </div>
      {current ? (
        <div className="details-grid">
          <details open>
            <summary>
              <ChevronDown size={15} />
              Current GPS
            </summary>
            <div className="detail-metrics">
              <Metric label="Latitude" value={formatCoordinate(current[0])} />
              <Metric label="Longitude" value={formatCoordinate(current[1])} />
              <Metric label="Altitude" value={formatMeters(current[2])} />
              <Metric label="Path index" value={`${state.index + 1} / ${path.length}`} />
            </div>
          </details>

          <details open>
            <summary>
              <ChevronDown size={15} />
              TERCOM comparison
            </summary>
            <div className="patch-pair">
              <PatchGrid title="Sensed" values={state.sensedPatch} />
              <PatchGrid title="Database" values={state.databasePatch} />
              <Metric label="Correlation" value={state.tercomCorrelation.toFixed(4)} />
            </div>
          </details>

          <details open>
            <summary>
              <ChevronDown size={15} />
              Kalman filter
            </summary>
            <div className="detail-metrics">
              <Metric label="Cov N" value={state.covariance[0].toFixed(2)} />
              <Metric label="Cov E" value={state.covariance[1].toFixed(2)} />
              <Metric label="Cov Alt" value={state.covariance[2].toFixed(2)} />
              <Metric label="Vertical speed" value={`${state.velocity[2].toFixed(2)} m/s`} />
            </div>
          </details>
        </div>
      ) : (
        <div className="empty-state wide">Tracking will use the latest planned path</div>
      )}
    </section>
  );
}

function Metric(props: { label: string; value: string }) {
  return (
    <div className="metric">
      <span>{props.label}</span>
      <strong>{props.value}</strong>
    </div>
  );
}

function PatchGrid(props: { title: string; values: number[][] }) {
  return (
    <div className="patch-grid-wrap">
      <strong>{props.title}</strong>
      <div className="patch-grid">
        {props.values.flatMap((row, rowIndex) =>
          row.map((value, colIndex) => (
            <span
              key={`${rowIndex}-${colIndex}`}
              style={{
                backgroundColor: `color-mix(in srgb, #0f766e ${Math.max(10, Math.min(95, value * 100))}%, #f8fafc)`
              }}
            />
          ))
        )}
      </div>
    </div>
  );
}

function makeElevationChart(path: PathPoint[]): { linePath: string; areaPath: string; minAlt: number; maxAlt: number } | null {
  if (path.length < 2) return null;

  const width = 640;
  const height = 120;
  const padding = 10;
  const distances = [0];
  for (let index = 1; index < path.length; index += 1) {
    distances.push(
      distances[index - 1] +
        haversineDistance({ lat: path[index - 1][0], lon: path[index - 1][1] }, { lat: path[index][0], lon: path[index][1] })
    );
  }
  const alts = path.map((point) => point[2]);
  const maxDistance = Math.max(...distances);
  const minAlt = Math.min(...alts);
  const maxAlt = Math.max(...alts);
  const altRange = Math.max(1, maxAlt - minAlt);
  const samples = path.map((point, index) => {
    const x = padding + (distances[index] / Math.max(1, maxDistance)) * (width - padding * 2);
    const y = height - padding - ((point[2] - minAlt) / altRange) * (height - padding * 2);
    return [x, y];
  });

  const linePath = samples.map(([x, y], index) => `${index === 0 ? "M" : "L"} ${x.toFixed(2)} ${y.toFixed(2)}`).join(" ");
  const areaPath = `${linePath} L ${width - padding} ${height - padding} L ${padding} ${height - padding} Z`;
  return { linePath, areaPath, minAlt, maxAlt };
}

function makeTrackingState(path: PathPoint[], index: number): TrackingState {
  if (!path.length) {
    return {
      index: 0,
      current: null,
      trace: [],
      sources: [],
      velocity: [0, 0, 0],
      covariance: [0, 0, 0],
      tercomCorrelation: 0,
      sensedPatch: generatePatch(0, 0),
      databasePatch: generatePatch(0, 0)
    };
  }

  const currentIndex = Math.max(0, Math.min(path.length - 1, index));
  const current = path[currentIndex];
  const previous = path[Math.max(0, currentIndex - 1)];
  const latMeter = 1 / 111320;
  const lonMeter = 1 / (111320 * Math.max(0.2, Math.cos((current[0] * Math.PI) / 180)));

  const gpsOffset = {
    lat: deterministicNoise(currentIndex + 1, 5) * latMeter,
    lon: deterministicNoise(currentIndex + 2, 5) * lonMeter
  };
  const insDriftMeters = Math.min(70, currentIndex * 0.04);
  const insOffset = {
    lat: (deterministicNoise(currentIndex + 3, 1) * 2 + insDriftMeters) * latMeter,
    lon: (deterministicNoise(currentIndex + 4, 1) * 2 - insDriftMeters * 0.45) * lonMeter
  };
  const tercomOffset = {
    lat: deterministicNoise(currentIndex + 5, 10) * latMeter,
    lon: deterministicNoise(currentIndex + 6, 10) * lonMeter
  };
  const kalmanPoint = {
    lat: current[0] + (gpsOffset.lat * 0.45 + tercomOffset.lat * 0.45 + insOffset.lat * 0.1),
    lon: current[1] + (gpsOffset.lon * 0.45 + tercomOffset.lon * 0.45 + insOffset.lon * 0.1)
  };

  const sources: SensorEstimate[] = [
    {
      name: "GPS",
      point: { lat: current[0] + gpsOffset.lat, lon: current[1] + gpsOffset.lon },
      altitude: current[2] + deterministicNoise(currentIndex + 8, 3),
      errorMeters: Math.hypot(gpsOffset.lat / latMeter, gpsOffset.lon / lonMeter),
      confidence: 92 + deterministicNoise(currentIndex, 4),
      freshnessMs: 125
    },
    {
      name: "INS",
      point: { lat: current[0] + insOffset.lat, lon: current[1] + insOffset.lon },
      altitude: current[2] + deterministicNoise(currentIndex + 9, 8),
      errorMeters: Math.hypot(insOffset.lat / latMeter, insOffset.lon / lonMeter),
      confidence: Math.max(55, 88 - insDriftMeters * 0.3),
      freshnessMs: 4
    },
    {
      name: "TERCOM",
      point: { lat: current[0] + tercomOffset.lat, lon: current[1] + tercomOffset.lon },
      altitude: current[2] + deterministicNoise(currentIndex + 10, 2),
      errorMeters: Math.hypot(tercomOffset.lat / latMeter, tercomOffset.lon / lonMeter),
      confidence: 86 + deterministicNoise(currentIndex + 1, 6),
      freshnessMs: 1000
    },
    {
      name: "Kalman",
      point: kalmanPoint,
      altitude: current[2] + deterministicNoise(currentIndex + 11, 1.2),
      errorMeters: Math.hypot((kalmanPoint.lat - current[0]) / latMeter, (kalmanPoint.lon - current[1]) / lonMeter),
      confidence: 96,
      freshnessMs: 20
    }
  ];

  const north = (current[0] - previous[0]) / latMeter;
  const east = (current[1] - previous[1]) / lonMeter;
  const vertical = current[2] - previous[2];
  const updateRate = 8;

  return {
    index: currentIndex,
    current,
    trace: path.slice(Math.max(0, currentIndex - 800), currentIndex + 1),
    sources,
    velocity: [north * updateRate, east * updateRate, vertical * updateRate],
    covariance: [
      Math.max(0.8, 14 / (1 + currentIndex * 0.01)),
      Math.max(0.8, 14 / (1 + currentIndex * 0.01)),
      Math.max(0.5, 7 / (1 + currentIndex * 0.008))
    ],
    tercomCorrelation: 0.982 + Math.abs(deterministicNoise(currentIndex + 7, 0.016)),
    sensedPatch: generatePatch(currentIndex, 0),
    databasePatch: generatePatch(currentIndex, 0.04)
  };
}

function generatePatch(seed: number, offset: number): number[][] {
  return Array.from({ length: 7 }, (_, row) =>
    Array.from({ length: 7 }, (_, col) => {
      const value = 0.5 + Math.sin((row + seed * 0.05) * 1.2) * 0.22 + Math.cos((col - seed * 0.04) * 0.9) * 0.18 + offset;
      return Math.max(0, Math.min(1, value));
    })
  );
}
