import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from src.missile.profile import MissileProfile
from src.terrain.dem_loader import DEMLoader


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEM_DIR = PROJECT_ROOT / "data" / "dem"
PLANS_DIR = PROJECT_ROOT / "plans"
DEFAULT_MAX_PREVIEW_PIXELS = 900_000


DEFAULT_PROFILE = {
    "name": "Tomahawk Block V",
    "cruise_speed": 800.0,
    "min_speed": 400.0,
    "max_speed": 920.0,
    "max_acceleration": 9.8,
    "min_altitude": 30.0,
    "max_altitude": 1200.0,
    "max_g_force": 6.89,
    "sustained_turn_rate": 8.0,
    "sustained_g_force": 2.0,
    "evasive_turn_rate": 25.0,
}


FIELD_ORDER = [
    "cruise_speed",
    "min_speed",
    "max_speed",
    "max_acceleration",
    "min_altitude",
    "max_altitude",
    "max_g_force",
    "sustained_turn_rate",
    "sustained_g_force",
    "evasive_turn_rate",
]


FIELD_UNITS = {
    "cruise_speed": "km/h",
    "min_speed": "km/h",
    "max_speed": "km/h",
    "max_acceleration": "m/s^2",
    "min_altitude": "m AGL",
    "max_altitude": "m AGL",
    "max_g_force": "g",
    "sustained_turn_rate": "deg/s",
    "sustained_g_force": "g",
    "evasive_turn_rate": "deg/s",
}


@dataclass
class MissionPlan:
    plan_name: str
    dem_name: str
    start_lat: float
    start_lon: float
    target_lat: float
    target_lon: float
    missile_profile: dict
    notes: str


def parse_gps_input(gps_input: str) -> tuple[float, float]:
    clean_str = gps_input.strip().upper().replace(",", " ")
    numbers = []
    token = ""

    for char in clean_str:
        if char.isdigit() or char in ".-":
            token += char
            continue
        if token:
            numbers.append(float(token))
            token = ""
    if token:
        numbers.append(float(token))

    if len(numbers) != 2:
        raise ValueError("Expected two coordinates.")

    lat, lon = numbers[0], numbers[1]

    if "S" in clean_str:
        lat = -abs(lat)
    if "N" in clean_str:
        lat = abs(lat)
    if "W" in clean_str:
        lon = -abs(lon)
    if "E" in clean_str:
        lon = abs(lon)

    if not (-90 <= lat <= 90):
        raise ValueError("Latitude must be between -90 and 90.")
    if not (-180 <= lon <= 180):
        raise ValueError("Longitude must be between -180 and 180.")

    return lat, lon


def haversine_distance_m(start: tuple[float, float], end: tuple[float, float]) -> float:
    radius_m = 6_371_000
    lat1, lon1 = map(math.radians, start)
    lat2, lon2 = map(math.radians, end)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * radius_m * math.asin(math.sqrt(a))


def create_missile_profile(data: dict) -> MissileProfile:
    kmh_to_ms = 1.0 / 3.6
    deg_to_rad = math.pi / 180.0

    return MissileProfile(
        cruise_speed=data["cruise_speed"] * kmh_to_ms,
        min_speed=data["min_speed"] * kmh_to_ms,
        max_speed=data["max_speed"] * kmh_to_ms,
        max_acceleration=data["max_acceleration"],
        min_altitude=data["min_altitude"],
        max_altitude=data["max_altitude"],
        max_g_force=data["max_g_force"],
        sustained_turn_rate=data["sustained_turn_rate"] * deg_to_rad,
        sustained_g_force=data["sustained_g_force"],
        evasive_turn_rate=data["evasive_turn_rate"] * deg_to_rad,
    )


class MissionPlannerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Missile Guidance Mission Planner")
        self.root.geometry("1180x760")
        self.root.minsize(1000, 680)

        self.dem_loader: DEMLoader | None = None
        self.preview_canvas: FigureCanvasTkAgg | None = None
        self.preview_figure: Figure | None = None
        self.preview_axes = None
        self.preview_status_var = tk.StringVar(value="DEM preview idle.")

        self.plan_name_var = tk.StringVar(value="Planning Draft")
        self.dem_name_var = tk.StringVar(value=self._default_dem_name())
        self.start_var = tk.StringVar(value="55.0, 91.0")
        self.target_var = tk.StringVar(value="56.5, 96.0")

        self.profile_vars = {
            key: tk.StringVar(value=str(DEFAULT_PROFILE[key]))
            for key in FIELD_ORDER
        }

        self._build_layout()
        self._set_summary(
            "Mission planner ready.\n\n"
            "Use Validate Plan to check terrain coverage, waypoint elevations, and rough mission timing."
        )

    def _default_dem_name(self) -> str:
        tif_files = sorted(path.name for path in DEM_DIR.glob("*.tif"))
        if "merged_dem_sib_N54_N59_E090_E100.tif" in tif_files:
            return "merged_dem_sib_N54_N59_E090_E100.tif"
        return tif_files[0] if tif_files else ""

    def _build_layout(self) -> None:
        self.root.configure(bg="#d8d3c4")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Card.TLabelframe", background="#f7f1e3", borderwidth=1)
        style.configure("Card.TLabelframe.Label", background="#f7f1e3", foreground="#30261c")
        style.configure("Action.TButton", padding=8)

        shell = tk.Frame(self.root, bg="#d8d3c4")
        shell.pack(fill="both", expand=True, padx=16, pady=16)
        shell.grid_columnconfigure(0, weight=3)
        shell.grid_columnconfigure(1, weight=2)
        shell.grid_rowconfigure(1, weight=1)

        header = tk.Frame(shell, bg="#5b4b3a", padx=18, pady=16)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        tk.Label(
            header,
            text="Mission Planner",
            font=("Helvetica", 22, "bold"),
            bg="#5b4b3a",
            fg="#f8f4ec",
        ).pack(anchor="w")
        tk.Label(
            header,
            text="Planning-stage interface for terrain selection, mission drafting, and pre-flight checks.",
            font=("Helvetica", 11),
            bg="#5b4b3a",
            fg="#eadfce",
        ).pack(anchor="w", pady=(4, 0))

        left = tk.Frame(shell, bg="#d8d3c4")
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        left.grid_columnconfigure(0, weight=1)

        right = tk.Frame(shell, bg="#d8d3c4")
        right.grid(row=1, column=1, sticky="nsew")
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)
        right.grid_rowconfigure(2, weight=1)

        mission_card = ttk.LabelFrame(left, text="Mission Setup", style="Card.TLabelframe")
        mission_card.pack(fill="x", pady=(0, 12))
        self._build_mission_section(mission_card)

        profile_card = ttk.LabelFrame(left, text="Missile Profile", style="Card.TLabelframe")
        profile_card.pack(fill="both", expand=True)
        self._build_profile_section(profile_card)

        actions_card = ttk.LabelFrame(right, text="Actions", style="Card.TLabelframe")
        actions_card.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        self._build_actions_section(actions_card)

        preview_card = ttk.LabelFrame(right, text="DEM Preview", style="Card.TLabelframe")
        preview_card.grid(row=1, column=0, sticky="nsew", pady=(0, 12))
        self._build_preview_section(preview_card)

        summary_card = ttk.LabelFrame(right, text="Mission Summary", style="Card.TLabelframe")
        summary_card.grid(row=2, column=0, sticky="nsew")
        self._build_summary_section(summary_card)

    def _build_mission_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="x")
        frame.grid_columnconfigure(1, weight=1)

        self._add_labeled_entry(frame, "Plan Name", self.plan_name_var, 0)

        tk.Label(frame, text="Terrain DEM", bg="#f7f1e3", fg="#30261c").grid(row=1, column=0, sticky="w", pady=6)
        dem_values = sorted(path.name for path in DEM_DIR.glob("*.tif"))
        self.dem_combo = ttk.Combobox(
            frame,
            textvariable=self.dem_name_var,
            values=dem_values,
            state="readonly" if dem_values else "normal",
        )
        self.dem_combo.grid(row=1, column=1, sticky="ew", pady=6, padx=(12, 0))

        self._add_labeled_entry(frame, "Start GPS", self.start_var, 2)
        self._add_labeled_entry(frame, "Target GPS", self.target_var, 3)

        hint = "Accepted formats: 55.5, 95.0 or 55.5 N, 95.0 E"
        tk.Label(frame, text=hint, bg="#f7f1e3", fg="#6d5f52", font=("Helvetica", 10)).grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )

    def _build_profile_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="both", expand=True)
        frame.grid_columnconfigure(1, weight=1)

        for row_index, key in enumerate(FIELD_ORDER):
            label = key.replace("_", " ").title()
            unit = FIELD_UNITS[key]
            tk.Label(frame, text=f"{label} ({unit})", bg="#f7f1e3", fg="#30261c").grid(
                row=row_index, column=0, sticky="w", pady=4
            )
            tk.Entry(frame, textvariable=self.profile_vars[key], bg="#fffdf7", relief="flat").grid(
                row=row_index, column=1, sticky="ew", padx=(12, 0), pady=4
            )

    def _build_actions_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="x")
        frame.grid_columnconfigure((0, 1), weight=1)

        ttk.Button(frame, text="Validate Plan", style="Action.TButton", command=self.validate_plan).grid(
            row=0, column=0, sticky="ew", padx=(0, 8), pady=(0, 8)
        )
        ttk.Button(frame, text="Preview DEM", style="Action.TButton", command=self.refresh_preview).grid(
            row=0, column=1, sticky="ew", pady=(0, 8)
        )
        ttk.Button(frame, text="Save Plan", style="Action.TButton", command=self.save_plan).grid(
            row=1, column=0, sticky="ew", padx=(0, 8)
        )
        ttk.Button(frame, text="GPU Preview", style="Action.TButton", command=self.open_gpu_preview).grid(
            row=1, column=1, sticky="ew"
        )
        ttk.Button(frame, text="Load Plan", style="Action.TButton", command=self.load_plan).grid(
            row=2, column=0, sticky="ew", padx=(0, 8), pady=(8, 0)
        )
        ttk.Button(frame, text="Reset Defaults", style="Action.TButton", command=self.reset_defaults).grid(
            row=2, column=1, sticky="ew", pady=(8, 0)
        )

    def _build_preview_section(self, parent: ttk.LabelFrame) -> None:
        container = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        container.pack(fill="both", expand=True)
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(1, weight=1)

        tk.Label(
            container,
            textvariable=self.preview_status_var,
            bg="#f7f1e3",
            fg="#6d5f52",
            anchor="w",
            justify="left",
            font=("Helvetica", 10),
        ).grid(row=0, column=0, sticky="ew")

        canvas_host = tk.Frame(container, bg="#fffdf7", highlightbackground="#d7cdbd", highlightthickness=1)
        canvas_host.grid(row=1, column=0, sticky="nsew", pady=(10, 0))

        self.preview_figure = Figure(figsize=(5.2, 3.6), dpi=100)
        self.preview_axes = self.preview_figure.add_subplot(111)
        self.preview_axes.set_facecolor("#f4efe5")
        self.preview_axes.set_xticks([])
        self.preview_axes.set_yticks([])
        self.preview_axes.text(
            0.5,
            0.5,
            "DEM preview will appear here",
            ha="center",
            va="center",
            transform=self.preview_axes.transAxes,
            color="#6d5f52",
            fontsize=11,
        )

        self.preview_canvas = FigureCanvasTkAgg(self.preview_figure, master=canvas_host)
        self.preview_canvas.draw()
        self.preview_canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_summary_section(self, parent: ttk.LabelFrame) -> None:
        container = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        container.pack(fill="both", expand=True)
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(1, weight=1)

        tk.Label(
            container,
            text="Planner Notes",
            bg="#f7f1e3",
            fg="#30261c",
            font=("Helvetica", 12, "bold"),
        ).grid(row=0, column=0, sticky="w")

        self.summary_text = tk.Text(
            container,
            wrap="word",
            bg="#fffdf7",
            fg="#1f1a16",
            relief="flat",
            font=("Menlo", 11),
            padx=10,
            pady=10,
        )
        self.summary_text.grid(row=1, column=0, sticky="nsew", pady=(10, 0))

    def _add_labeled_entry(self, parent: tk.Widget, label: str, variable: tk.StringVar, row: int) -> None:
        tk.Label(parent, text=label, bg="#f7f1e3", fg="#30261c").grid(row=row, column=0, sticky="w", pady=6)
        tk.Entry(parent, textvariable=variable, bg="#fffdf7", relief="flat").grid(
            row=row, column=1, sticky="ew", padx=(12, 0), pady=6
        )

    def _set_summary(self, text: str) -> None:
        self.summary_text.delete("1.0", tk.END)
        self.summary_text.insert("1.0", text)

    def _profile_data_from_form(self) -> dict:
        profile = {"name": DEFAULT_PROFILE["name"]}
        for key in FIELD_ORDER:
            try:
                profile[key] = float(self.profile_vars[key].get().strip())
            except ValueError as exc:
                raise ValueError(f"Invalid value for {key.replace('_', ' ')}.") from exc
        return profile

    def _build_plan(self) -> MissionPlan:
        start_lat, start_lon = parse_gps_input(self.start_var.get())
        target_lat, target_lon = parse_gps_input(self.target_var.get())
        return MissionPlan(
            plan_name=self.plan_name_var.get().strip() or "Planning Draft",
            dem_name=self.dem_name_var.get().strip(),
            start_lat=start_lat,
            start_lon=start_lon,
            target_lat=target_lat,
            target_lon=target_lon,
            missile_profile=self._profile_data_from_form(),
            notes=self.summary_text.get("1.0", tk.END).strip(),
        )

    def _load_dem(self, dem_name: str) -> DEMLoader:
        dem_path = DEM_DIR / dem_name
        return DEMLoader(dem_path)

    def _coordinate_status(self, dem: DEMLoader, lat: float, lon: float, label: str) -> list[str]:
        lines = []
        elevation = dem.get_elevation(lat, lon)
        if elevation is None:
            lines.append(f"{label}: outside DEM coverage or invalid terrain cell")
        else:
            lines.append(f"{label}: inside terrain bounds at {elevation:.1f} m elevation")
        return lines

    def validate_plan(self) -> None:
        try:
            plan = self._build_plan()
            if not plan.dem_name:
                raise ValueError("No DEM file is available.")

            self.dem_loader = self._load_dem(plan.dem_name)
            profile = create_missile_profile(plan.missile_profile)
        except Exception as exc:
            messagebox.showerror("Validation Error", str(exc))
            return

        start = (plan.start_lat, plan.start_lon)
        target = (plan.target_lat, plan.target_lon)
        distance_m = haversine_distance_m(start, target)
        cruise_time_s = distance_m / max(profile.cruise_speed, 1e-6)
        turning_radius = profile.calculate_turning_radius(profile.cruise_speed, profile.sustained_turn_rate)

        lines = [
            f"Plan: {plan.plan_name}",
            f"Terrain: {plan.dem_name}",
            "",
            "Validation:",
        ]
        lines.extend(self._coordinate_status(self.dem_loader, *start, "Start"))
        lines.extend(self._coordinate_status(self.dem_loader, *target, "Target"))
        lines.extend(
            [
                "",
                "Mission Estimates:",
                f"Straight-line distance: {distance_m / 1000:.2f} km",
                f"Nominal cruise speed: {plan.missile_profile['cruise_speed']:.1f} km/h",
                f"Estimated direct-flight time: {cruise_time_s / 60:.1f} min",
                f"Altitude window: {plan.missile_profile['min_altitude']:.0f} m to {plan.missile_profile['max_altitude']:.0f} m",
                f"Approx. sustained turn radius: {turning_radius:.1f} m",
                "",
                "Status:",
            ]
        )

        if self.dem_loader.get_elevation(*start) is None or self.dem_loader.get_elevation(*target) is None:
            lines.append("Mission needs different coordinates or a different DEM before route planning.")
        else:
            lines.append("Mission is ready for planning-stage review and future pathfinding integration.")

        self._set_summary("\n".join(lines))
        self.refresh_preview()

    def _compute_preview_window(
        self,
        dem: DEMLoader,
        start: tuple[float, float],
        target: tuple[float, float],
    ) -> tuple[int, int, int, int, int]:
        start_elev = dem.get_elevation(*start)
        target_elev = dem.get_elevation(*target)

        if start_elev is not None and target_elev is not None:
            start_row, start_col = dem.lat_lon_to_pixel(*start)
            target_row, target_col = dem.lat_lon_to_pixel(*target)

            row_min = min(start_row, target_row)
            row_max = max(start_row, target_row)
            col_min = min(start_col, target_col)
            col_max = max(start_col, target_col)

            row_padding = max(120, int((row_max - row_min) * 0.2))
            col_padding = max(120, int((col_max - col_min) * 0.2))

            row_start = max(0, row_min - row_padding)
            row_end = min(dem.shape[0], row_max + row_padding)
            col_start = max(0, col_min - col_padding)
            col_end = min(dem.shape[1], col_max + col_padding)
        else:
            row_start = 0
            row_end = dem.shape[0]
            col_start = 0
            col_end = dem.shape[1]

        height = max(1, row_end - row_start)
        width = max(1, col_end - col_start)
        downsample = 1
        while (height // downsample) * (width // downsample) > DEFAULT_MAX_PREVIEW_PIXELS:
            downsample += 1

        return row_start, row_end, col_start, col_end, downsample

    def _render_preview(
        self,
        dem: DEMLoader,
        start: tuple[float, float],
        target: tuple[float, float],
    ) -> None:
        row_start, row_end, col_start, col_end, downsample = self._compute_preview_window(dem, start, target)
        dem_patch = dem.data[row_start:row_end:downsample, col_start:col_end:downsample].astype(np.float32)

        nodata = dem.nodata
        if nodata is not None:
            dem_patch[dem_patch == nodata] = np.nan
        dem_patch[dem_patch <= -10000] = np.nan

        valid = dem_patch[~np.isnan(dem_patch)]
        if valid.size == 0:
            raise ValueError("The selected DEM region contains no valid elevation values.")

        vmin = float(np.percentile(valid, 2))
        vmax = float(np.percentile(valid, 98))

        top_lat, left_lon = dem.pixel_to_lat_lon(row_start, col_start)
        bottom_lat, right_lon = dem.pixel_to_lat_lon(max(row_start, row_end - 1), max(col_start, col_end - 1))

        self.preview_axes.clear()
        self.preview_axes.imshow(
            dem_patch,
            cmap="terrain",
            origin="upper",
            extent=[left_lon, right_lon, bottom_lat, top_lat],
            vmin=vmin,
            vmax=vmax,
            interpolation="nearest",
            aspect="auto",
        )

        start_valid = dem.get_elevation(*start) is not None
        target_valid = dem.get_elevation(*target) is not None

        if start_valid:
            self.preview_axes.scatter(start[1], start[0], c="#00a86b", s=45, edgecolors="black", linewidths=0.7)
            self.preview_axes.annotate("Start", (start[1], start[0]), xytext=(6, 8), textcoords="offset points")
        if target_valid:
            self.preview_axes.scatter(target[1], target[0], c="#d1495b", s=45, edgecolors="black", linewidths=0.7)
            self.preview_axes.annotate("Target", (target[1], target[0]), xytext=(6, 8), textcoords="offset points")
        if start_valid and target_valid:
            self.preview_axes.plot([start[1], target[1]], [start[0], target[0]], "--", color="#13293d", linewidth=1.5)

        self.preview_axes.set_title(f"Terrain Window ({dem_patch.shape[1]} x {dem_patch.shape[0]})", fontsize=11)
        self.preview_axes.set_xlabel("Longitude")
        self.preview_axes.set_ylabel("Latitude")
        self.preview_figure.tight_layout()
        self.preview_canvas.draw()

        coverage_text = "mission crop" if start_valid and target_valid else "full DEM fallback"
        self.preview_status_var.set(
            f"Showing {coverage_text}. Downsample {downsample}x, preview shape {dem_patch.shape[0]} x {dem_patch.shape[1]}."
        )

    def refresh_preview(self) -> None:
        try:
            plan = self._build_plan()
            if not plan.dem_name:
                raise ValueError("No DEM file is available.")
            self.dem_loader = self._load_dem(plan.dem_name)
            start = (plan.start_lat, plan.start_lon)
            target = (plan.target_lat, plan.target_lon)
            self._render_preview(self.dem_loader, start, target)
        except Exception as exc:
            self.preview_axes.clear()
            self.preview_axes.set_xticks([])
            self.preview_axes.set_yticks([])
            self.preview_axes.text(
                0.5,
                0.5,
                str(exc),
                ha="center",
                va="center",
                wrap=True,
                transform=self.preview_axes.transAxes,
                color="#8a1c1c",
                fontsize=10,
            )
            self.preview_canvas.draw()
            self.preview_status_var.set("Preview unavailable.")

    def open_gpu_preview(self) -> None:
        try:
            plan = self._build_plan()
            gpu_script = PROJECT_ROOT / "src" / "ui" / "gpu_dem_preview.py"
            command = [
                sys.executable,
                str(gpu_script),
                "--dem",
                plan.dem_name,
                "--start-lat",
                str(plan.start_lat),
                "--start-lon",
                str(plan.start_lon),
                "--target-lat",
                str(plan.target_lat),
                "--target-lon",
                str(plan.target_lon),
            ]
            subprocess.Popen(command, cwd=PROJECT_ROOT)
            self.preview_status_var.set("Opened external GPU preview window.")
        except FileNotFoundError:
            messagebox.showerror("GPU Preview Error", "Python executable was not found.")
        except Exception as exc:
            messagebox.showerror("GPU Preview Error", str(exc))

    def save_plan(self) -> None:
        try:
            plan = self._build_plan()
        except Exception as exc:
            messagebox.showerror("Save Error", str(exc))
            return

        PLANS_DIR.mkdir(exist_ok=True)
        suggested_name = f"{plan.plan_name.lower().replace(' ', '_')}.json"
        destination = filedialog.asksaveasfilename(
            title="Save Mission Plan",
            initialdir=PLANS_DIR,
            initialfile=suggested_name,
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
        )
        if not destination:
            return

        with open(destination, "w", encoding="utf-8") as handle:
            json.dump(asdict(plan), handle, indent=2)

        messagebox.showinfo("Plan Saved", f"Mission plan saved to:\n{destination}")

    def load_plan(self) -> None:
        if not PLANS_DIR.exists():
            PLANS_DIR.mkdir(exist_ok=True)

        source = filedialog.askopenfilename(
            title="Load Mission Plan",
            initialdir=PLANS_DIR,
            filetypes=[("JSON files", "*.json")],
        )
        if not source:
            return

        with open(source, "r", encoding="utf-8") as handle:
            payload = json.load(handle)

        self.plan_name_var.set(payload.get("plan_name", "Planning Draft"))
        self.dem_name_var.set(payload.get("dem_name", self._default_dem_name()))
        self.start_var.set(f"{payload.get('start_lat', 0.0)}, {payload.get('start_lon', 0.0)}")
        self.target_var.set(f"{payload.get('target_lat', 0.0)}, {payload.get('target_lon', 0.0)}")

        missile_profile = payload.get("missile_profile", {})
        for key in FIELD_ORDER:
            self.profile_vars[key].set(str(missile_profile.get(key, DEFAULT_PROFILE[key])))

        self._set_summary(payload.get("notes", "Plan loaded."))
        self.refresh_preview()

    def reset_defaults(self) -> None:
        self.plan_name_var.set("Planning Draft")
        self.dem_name_var.set(self._default_dem_name())
        self.start_var.set("55.0, 91.0")
        self.target_var.set("56.5, 96.0")
        for key in FIELD_ORDER:
            self.profile_vars[key].set(str(DEFAULT_PROFILE[key]))
        self._set_summary(
            "Mission planner reset.\n\n"
            "Validate the default mission or enter new coordinates to draft a different route."
        )
        self.refresh_preview()


def main() -> None:
    root = tk.Tk()
    MissionPlannerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
