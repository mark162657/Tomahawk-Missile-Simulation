import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from src.pathfinder.pathfinding_backend import Pathfinding
from src.missile.config_store import DEFAULT_CONFIGURATION, get_configuration, load_configurations
from src.missile.profile import MissileProfile
from src.terrain.dem_loader import DEMLoader


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEM_DIR = PROJECT_ROOT / "data" / "dem"
PLANS_DIR = PROJECT_ROOT / "plans"
DEFAULT_MAX_PREVIEW_PIXELS = 900_000
DEFAULT_HEURISTIC_WEIGHT = 1.5


@dataclass
class MissionPlan:
    plan_name: str
    dem_name: str
    missile_config_name: str
    start_lat: float
    start_lon: float
    target_lat: float
    target_lon: float
    waypoints: list[dict]
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


def format_gps(lat: float, lon: float) -> str:
    return f"{lat:.6f}, {lon:.6f}"


def haversine_distance_m(start: tuple[float, float], end: tuple[float, float]) -> float:
    radius_m = 6_371_000
    lat1, lon1 = map(math.radians, start)
    lat2, lon2 = map(math.radians, end)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * radius_m * math.asin(math.sqrt(a))


def polyline_distance_m(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(haversine_distance_m(points[index], points[index + 1]) for index in range(len(points) - 1))


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
        self.root.geometry("1380x880")
        self.root.minsize(1180, 760)

        self.dem_loader: DEMLoader | None = None
        self.pathfinder: Pathfinding | None = None
        self.pathfinder_dem_name: str | None = None
        self.route_points: list[tuple[float, float]] = []
        self.pathfinding_in_progress = False
        self.waypoints: list[tuple[float, float]] = []

        self.preview_canvas: FigureCanvasTkAgg | None = None
        self.preview_figure: Figure | None = None
        self.preview_axes = None
        self.preview_status_var = tk.StringVar(value="Interactive map ready.")
        self.map_click_mode_var = tk.StringVar(value="none")
        self.path_status_var = tk.StringVar(value="No pathfinding run yet.")
        self.config_summary_var = tk.StringVar(value="No missile configuration loaded.")

        self.plan_name_var = tk.StringVar(value="Planning Draft")
        self.dem_name_var = tk.StringVar(value=self._default_dem_name())
        self.start_var = tk.StringVar(value="55.000000, 91.000000")
        self.target_var = tk.StringVar(value="56.500000, 96.000000")
        self.waypoint_input_var = tk.StringVar()
        self.missile_config_var = tk.StringVar()

        self.available_configs = []

        self._build_layout()
        self.refresh_configurations(select_name=DEFAULT_CONFIGURATION["name"])
        self._refresh_waypoint_list()
        self._set_summary(
            "Planner ready.\n\n"
            "Use the map toolbar to pan and zoom. Switch the click mode to place start, target, "
            "or new waypoint positions directly on the map, then run pathfinding from the planner."
        )
        self.refresh_preview()

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
        shell.grid_columnconfigure(0, weight=2)
        shell.grid_columnconfigure(1, weight=3)
        shell.grid_rowconfigure(1, weight=1)

        header = tk.Frame(shell, bg="#5b4b3a", padx=18, pady=16)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        tk.Label(
            header,
            text="Mission Planner",
            font=("Helvetica", 24, "bold"),
            bg="#5b4b3a",
            fg="#f8f4ec",
        ).pack(anchor="w")
        tk.Label(
            header,
            text="Interactive planning workspace for terrain review, missile selection, waypoint drafting, and pathfinding.",
            font=("Helvetica", 11),
            bg="#5b4b3a",
            fg="#eadfce",
        ).pack(anchor="w", pady=(4, 0))

        left = tk.Frame(shell, bg="#d8d3c4")
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 12))
        left.grid_columnconfigure(0, weight=1)

        right = tk.Frame(shell, bg="#d8d3c4")
        right.grid(row=1, column=1, sticky="nsew")
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=3)
        right.grid_rowconfigure(2, weight=2)

        mission_card = ttk.LabelFrame(left, text="Mission Setup", style="Card.TLabelframe")
        mission_card.pack(fill="x", pady=(0, 12))
        self._build_mission_section(mission_card)

        missile_card = ttk.LabelFrame(left, text="Missile Configuration", style="Card.TLabelframe")
        missile_card.pack(fill="x", pady=(0, 12))
        self._build_missile_section(missile_card)

        waypoint_card = ttk.LabelFrame(left, text="Route & Waypoints", style="Card.TLabelframe")
        waypoint_card.pack(fill="both", expand=True)
        self._build_waypoint_section(waypoint_card)

        actions_card = ttk.LabelFrame(right, text="Actions", style="Card.TLabelframe")
        actions_card.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        self._build_actions_section(actions_card)

        preview_card = ttk.LabelFrame(right, text="Interactive Map", style="Card.TLabelframe")
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
        self.dem_combo.bind("<<ComboboxSelected>>", lambda _event: self._on_dem_changed())

        self._add_labeled_entry(frame, "Start GPS", self.start_var, 2)
        self._add_labeled_entry(frame, "Target GPS", self.target_var, 3)

        tk.Label(
            frame,
            text="Map click modes can place start, target, or new waypoint coordinates directly from the preview.",
            bg="#f7f1e3",
            fg="#6d5f52",
            font=("Helvetica", 10),
            wraplength=420,
            justify="left",
        ).grid(row=4, column=0, columnspan=2, sticky="w", pady=(8, 0))

    def _build_missile_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="x")
        frame.grid_columnconfigure(1, weight=1)

        tk.Label(frame, text="Selected Config", bg="#f7f1e3", fg="#30261c").grid(row=0, column=0, sticky="w", pady=6)
        self.config_combo = ttk.Combobox(frame, textvariable=self.missile_config_var, state="readonly")
        self.config_combo.grid(row=0, column=1, sticky="ew", pady=6, padx=(12, 0))
        self.config_combo.bind("<<ComboboxSelected>>", lambda _event: self._update_config_summary())

        button_row = tk.Frame(frame, bg="#f7f1e3")
        button_row.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        button_row.grid_columnconfigure((0, 1), weight=1)

        ttk.Button(button_row, text="Manage Configs", command=self.open_config_manager).grid(
            row=0, column=0, sticky="ew", padx=(0, 8)
        )
        ttk.Button(button_row, text="Reload Configs", command=self.refresh_configurations).grid(
            row=0, column=1, sticky="ew"
        )

        tk.Label(
            frame,
            textvariable=self.config_summary_var,
            bg="#f7f1e3",
            fg="#6d5f52",
            justify="left",
            anchor="w",
            wraplength=460,
            font=("Helvetica", 10),
        ).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))

    def _build_waypoint_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="both", expand=True)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(4, weight=1)

        tk.Label(frame, text="Waypoint Input", bg="#f7f1e3", fg="#30261c").grid(row=0, column=0, sticky="w")
        tk.Entry(frame, textvariable=self.waypoint_input_var, bg="#fffdf7", relief="flat").grid(
            row=1, column=0, sticky="ew", pady=(6, 8)
        )

        edit_row = tk.Frame(frame, bg="#f7f1e3")
        edit_row.grid(row=2, column=0, sticky="ew")
        edit_row.grid_columnconfigure((0, 1, 2), weight=1)

        ttk.Button(edit_row, text="Add Waypoint", command=self.add_waypoint_from_entry).grid(
            row=0, column=0, sticky="ew", padx=(0, 8)
        )
        ttk.Button(edit_row, text="Update Selected", command=self.update_selected_waypoint).grid(
            row=0, column=1, sticky="ew", padx=(0, 8)
        )
        ttk.Button(edit_row, text="Use Selected", command=self.load_selected_waypoint_into_entry).grid(
            row=0, column=2, sticky="ew"
        )

        click_mode_frame = tk.Frame(frame, bg="#f7f1e3")
        click_mode_frame.grid(row=3, column=0, sticky="ew", pady=(12, 10))

        tk.Label(click_mode_frame, text="Map Click Mode:", bg="#f7f1e3", fg="#30261c").pack(anchor="w")
        for label, value in [
            ("Pan Only", "none"),
            ("Set Start", "start"),
            ("Set Target", "target"),
            ("Add Waypoint", "waypoint"),
        ]:
            ttk.Radiobutton(click_mode_frame, text=label, value=value, variable=self.map_click_mode_var).pack(
                anchor="w"
            )

        list_frame = tk.Frame(frame, bg="#f7f1e3")
        list_frame.grid(row=4, column=0, sticky="nsew")
        list_frame.grid_columnconfigure(0, weight=1)
        list_frame.grid_rowconfigure(0, weight=1)

        self.waypoint_listbox = tk.Listbox(
            list_frame,
            activestyle="none",
            exportselection=False,
            font=("Menlo", 10),
            height=8,
        )
        self.waypoint_listbox.grid(row=0, column=0, sticky="nsew")
        self.waypoint_listbox.bind("<<ListboxSelect>>", lambda _event: self._on_waypoint_selected())

        action_row = tk.Frame(frame, bg="#f7f1e3")
        action_row.grid(row=5, column=0, sticky="ew", pady=(12, 0))
        action_row.grid_columnconfigure((0, 1, 2, 3, 4), weight=1)

        ttk.Button(action_row, text="Move Up", command=self.move_waypoint_up).grid(
            row=0, column=0, sticky="ew", padx=(0, 8)
        )
        ttk.Button(action_row, text="Move Down", command=self.move_waypoint_down).grid(
            row=0, column=1, sticky="ew", padx=(0, 8)
        )
        ttk.Button(action_row, text="Remove", command=self.remove_selected_waypoint).grid(
            row=0, column=2, sticky="ew", padx=(0, 8)
        )
        ttk.Button(action_row, text="Clear", command=self.clear_waypoints).grid(
            row=0, column=3, sticky="ew", padx=(0, 8)
        )
        ttk.Button(action_row, text="Clear Route", command=self.clear_route).grid(
            row=0, column=4, sticky="ew"
        )

    def _build_actions_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="x")
        frame.grid_columnconfigure((0, 1, 2), weight=1)

        ttk.Button(frame, text="Validate Plan", style="Action.TButton", command=self.validate_plan).grid(
            row=0, column=0, sticky="ew", padx=(0, 8), pady=(0, 8)
        )
        ttk.Button(frame, text="Preview DEM", style="Action.TButton", command=self.refresh_preview).grid(
            row=0, column=1, sticky="ew", padx=(0, 8), pady=(0, 8)
        )
        ttk.Button(frame, text="Run Pathfinding", style="Action.TButton", command=self.run_pathfinding).grid(
            row=0, column=2, sticky="ew", pady=(0, 8)
        )

        ttk.Button(frame, text="Save Plan", style="Action.TButton", command=self.save_plan).grid(
            row=1, column=0, sticky="ew", padx=(0, 8)
        )
        ttk.Button(frame, text="Load Plan", style="Action.TButton", command=self.load_plan).grid(
            row=1, column=1, sticky="ew", padx=(0, 8)
        )
        ttk.Button(frame, text="GPU Preview", style="Action.TButton", command=self.open_gpu_preview).grid(
            row=1, column=2, sticky="ew"
        )

        tk.Label(
            frame,
            textvariable=self.path_status_var,
            bg="#f7f1e3",
            fg="#6d5f52",
            justify="left",
            anchor="w",
            font=("Helvetica", 10),
            wraplength=740,
        ).grid(row=2, column=0, columnspan=3, sticky="ew", pady=(12, 0))

    def _build_preview_section(self, parent: ttk.LabelFrame) -> None:
        container = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        container.pack(fill="both", expand=True)
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(2, weight=1)

        tk.Label(
            container,
            textvariable=self.preview_status_var,
            bg="#f7f1e3",
            fg="#6d5f52",
            anchor="w",
            justify="left",
            font=("Helvetica", 10),
        ).grid(row=0, column=0, sticky="ew")

        toolbar_host = tk.Frame(container, bg="#f7f1e3")
        toolbar_host.grid(row=1, column=0, sticky="ew", pady=(8, 8))

        canvas_host = tk.Frame(container, bg="#fffdf7", highlightbackground="#d7cdbd", highlightthickness=1)
        canvas_host.grid(row=2, column=0, sticky="nsew")

        self.preview_figure = Figure(figsize=(7.4, 5.6), dpi=100)
        self.preview_axes = self.preview_figure.add_subplot(111)
        self.preview_axes.set_facecolor("#f4efe5")
        self.preview_axes.set_xticks([])
        self.preview_axes.set_yticks([])
        self.preview_axes.text(
            0.5,
            0.5,
            "Interactive DEM preview will appear here",
            ha="center",
            va="center",
            transform=self.preview_axes.transAxes,
            color="#6d5f52",
            fontsize=11,
        )

        self.preview_canvas = FigureCanvasTkAgg(self.preview_figure, master=canvas_host)
        self.preview_canvas.draw()
        self.preview_canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.preview_canvas, toolbar_host, pack_toolbar=True)
        self.preview_canvas.mpl_connect("button_press_event", self._on_map_click)

    def _build_summary_section(self, parent: ttk.LabelFrame) -> None:
        container = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        container.pack(fill="both", expand=True)
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(1, weight=1)

        tk.Label(
            container,
            text="Planner Output",
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

    def _waypoint_payload(self) -> list[dict]:
        return [{"lat": lat, "lon": lon} for lat, lon in self.waypoints]

    def _build_plan(self) -> MissionPlan:
        start_lat, start_lon = parse_gps_input(self.start_var.get())
        target_lat, target_lon = parse_gps_input(self.target_var.get())
        missile_config_name = self.missile_config_var.get().strip()
        if not missile_config_name:
            raise ValueError("Select a missile configuration first.")

        return MissionPlan(
            plan_name=self.plan_name_var.get().strip() or "Planning Draft",
            dem_name=self.dem_name_var.get().strip(),
            missile_config_name=missile_config_name,
            start_lat=start_lat,
            start_lon=start_lon,
            target_lat=target_lat,
            target_lon=target_lon,
            waypoints=self._waypoint_payload(),
            notes=self.summary_text.get("1.0", tk.END).strip(),
        )

    def _load_dem(self, dem_name: str) -> DEMLoader:
        dem_path = DEM_DIR / dem_name
        return DEMLoader(dem_path)

    def _current_mission_points(self) -> list[tuple[float, float]]:
        try:
            start = parse_gps_input(self.start_var.get())
            target = parse_gps_input(self.target_var.get())
            return [start, *self.waypoints, target]
        except ValueError:
            return self.waypoints.copy()

    def refresh_configurations(self, select_name: str | None = None) -> None:
        self.available_configs = load_configurations()
        config_names = [config["name"] for config in self.available_configs]
        self.config_combo["values"] = config_names

        chosen_name = select_name or self.missile_config_var.get().strip()
        if chosen_name not in config_names:
            chosen_name = config_names[0] if config_names else ""

        self.missile_config_var.set(chosen_name)
        self._update_config_summary()

    def _selected_configuration(self) -> dict:
        config = get_configuration(self.missile_config_var.get())
        if not config:
            raise ValueError("Selected missile configuration could not be loaded.")
        return config

    def _update_config_summary(self) -> None:
        try:
            config = self._selected_configuration()
            self.config_summary_var.set(
                f"{config['name']}: cruise {config['cruise_speed']:.0f} km/h, "
                f"altitude {config['min_altitude']:.0f}-{config['max_altitude']:.0f} m, "
                f"max g {config['max_g_force']:.2f}."
            )
        except Exception as exc:
            self.config_summary_var.set(str(exc))

    def open_config_manager(self) -> None:
        try:
            command = [sys.executable, str(PROJECT_ROOT / "missile_configurator.py")]
            subprocess.Popen(command, cwd=PROJECT_ROOT)
        except Exception as exc:
            messagebox.showerror("Configuration Manager Error", str(exc))

    def _on_dem_changed(self) -> None:
        self.pathfinder = None
        self.pathfinder_dem_name = None
        self.clear_route()
        self.refresh_preview()

    def _refresh_waypoint_list(self) -> None:
        self.waypoint_listbox.delete(0, tk.END)
        for index, (lat, lon) in enumerate(self.waypoints, start=1):
            self.waypoint_listbox.insert(tk.END, f"W{index}: {format_gps(lat, lon)}")

    def _on_waypoint_selected(self) -> None:
        selection = self.waypoint_listbox.curselection()
        if selection:
            lat, lon = self.waypoints[selection[0]]
            self.waypoint_input_var.set(format_gps(lat, lon))

    def load_selected_waypoint_into_entry(self) -> None:
        self._on_waypoint_selected()

    def add_waypoint_from_entry(self) -> None:
        try:
            waypoint = parse_gps_input(self.waypoint_input_var.get())
        except ValueError as exc:
            messagebox.showerror("Waypoint Error", str(exc))
            return

        self.waypoints.append(waypoint)
        self.waypoint_input_var.set("")
        self._refresh_waypoint_list()
        self.clear_route()
        self.refresh_preview()

    def update_selected_waypoint(self) -> None:
        selection = self.waypoint_listbox.curselection()
        if not selection:
            messagebox.showerror("Waypoint Error", "Select a waypoint first.")
            return

        try:
            waypoint = parse_gps_input(self.waypoint_input_var.get())
        except ValueError as exc:
            messagebox.showerror("Waypoint Error", str(exc))
            return

        self.waypoints[selection[0]] = waypoint
        self._refresh_waypoint_list()
        self.waypoint_listbox.selection_set(selection[0])
        self.clear_route()
        self.refresh_preview()

    def remove_selected_waypoint(self) -> None:
        selection = self.waypoint_listbox.curselection()
        if not selection:
            return
        self.waypoints.pop(selection[0])
        self._refresh_waypoint_list()
        self.clear_route()
        self.refresh_preview()

    def move_waypoint_up(self) -> None:
        selection = self.waypoint_listbox.curselection()
        if not selection or selection[0] == 0:
            return
        index = selection[0]
        self.waypoints[index - 1], self.waypoints[index] = self.waypoints[index], self.waypoints[index - 1]
        self._refresh_waypoint_list()
        self.waypoint_listbox.selection_set(index - 1)
        self.clear_route()
        self.refresh_preview()

    def move_waypoint_down(self) -> None:
        selection = self.waypoint_listbox.curselection()
        if not selection or selection[0] >= len(self.waypoints) - 1:
            return
        index = selection[0]
        self.waypoints[index + 1], self.waypoints[index] = self.waypoints[index], self.waypoints[index + 1]
        self._refresh_waypoint_list()
        self.waypoint_listbox.selection_set(index + 1)
        self.clear_route()
        self.refresh_preview()

    def clear_waypoints(self) -> None:
        self.waypoints.clear()
        self.waypoint_input_var.set("")
        self._refresh_waypoint_list()
        self.clear_route()
        self.refresh_preview()

    def clear_route(self) -> None:
        self.route_points = []
        self.path_status_var.set("Route cleared.")
        self.refresh_preview()

    def _on_map_click(self, event) -> None:
        if event.inaxes != self.preview_axes or event.xdata is None or event.ydata is None:
            return

        click_mode = self.map_click_mode_var.get()
        if click_mode == "none":
            return

        point = (float(event.ydata), float(event.xdata))
        if click_mode == "start":
            self.start_var.set(format_gps(*point))
        elif click_mode == "target":
            self.target_var.set(format_gps(*point))
        elif click_mode == "waypoint":
            self.waypoints.append(point)
            self._refresh_waypoint_list()
        else:
            return

        self.clear_route()
        self.refresh_preview()

    def _coordinate_status(self, dem: DEMLoader, lat: float, lon: float, label: str) -> list[str]:
        elevation = dem.get_elevation(lat, lon)
        if elevation is None:
            return [f"{label}: outside DEM coverage or invalid terrain cell"]
        return [f"{label}: inside terrain bounds at {elevation:.1f} m elevation"]

    def validate_plan(self) -> None:
        try:
            plan = self._build_plan()
            if not plan.dem_name:
                raise ValueError("No DEM file is available.")

            self.dem_loader = self._load_dem(plan.dem_name)
            config = self._selected_configuration()
            profile = create_missile_profile(config)
        except Exception as exc:
            messagebox.showerror("Validation Error", str(exc))
            return

        mission_points = [(plan.start_lat, plan.start_lon), *self.waypoints, (plan.target_lat, plan.target_lon)]
        direct_distance_m = haversine_distance_m(mission_points[0], mission_points[-1])
        route_hint_distance_m = polyline_distance_m(mission_points)
        cruise_time_s = route_hint_distance_m / max(profile.cruise_speed, 1e-6)
        turning_radius = profile.calculate_turning_radius(profile.cruise_speed, profile.sustained_turn_rate)

        lines = [
            f"Plan: {plan.plan_name}",
            f"Terrain: {plan.dem_name}",
            f"Missile Config: {plan.missile_config_name}",
            "",
            "Validation:",
        ]
        lines.extend(self._coordinate_status(self.dem_loader, plan.start_lat, plan.start_lon, "Start"))
        for index, waypoint in enumerate(self.waypoints, start=1):
            lines.extend(self._coordinate_status(self.dem_loader, waypoint[0], waypoint[1], f"Waypoint {index}"))
        lines.extend(self._coordinate_status(self.dem_loader, plan.target_lat, plan.target_lon, "Target"))
        lines.extend(
            [
                "",
                "Mission Estimates:",
                f"Direct distance: {direct_distance_m / 1000:.2f} km",
                f"Route hint distance via waypoints: {route_hint_distance_m / 1000:.2f} km",
                f"Estimated route time at cruise: {cruise_time_s / 60:.1f} min",
                f"Altitude window: {config['min_altitude']:.0f} m to {config['max_altitude']:.0f} m",
                f"Approx. sustained turn radius: {turning_radius:.1f} m",
                "",
                "Status:",
            ]
        )

        if any(self.dem_loader.get_elevation(lat, lon) is None for lat, lon in mission_points):
            lines.append("Mission needs different coordinates or a different DEM before pathfinding.")
        else:
            lines.append("Mission is valid for planner-driven pathfinding.")

        self._set_summary("\n".join(lines))
        self.refresh_preview()

    def _mission_points_for_crop(self, dem: DEMLoader) -> list[tuple[float, float]]:
        points = []
        for lat, lon in self._current_mission_points():
            if dem.get_elevation(lat, lon) is not None:
                points.append((lat, lon))
        if self.route_points:
            sample_stride = max(1, len(self.route_points) // 100)
            for point in self.route_points[::sample_stride]:
                if dem.get_elevation(point[0], point[1]) is not None:
                    points.append(point)
        return points

    def _compute_preview_window(self, dem: DEMLoader) -> tuple[int, int, int, int, int]:
        mission_points = self._mission_points_for_crop(dem)
        if mission_points:
            rows = []
            cols = []
            for lat, lon in mission_points:
                row, col = dem.lat_lon_to_pixel(lat, lon)
                rows.append(row)
                cols.append(col)

            row_min = min(rows)
            row_max = max(rows)
            col_min = min(cols)
            col_max = max(cols)

            row_padding = max(150, int((row_max - row_min) * 0.2))
            col_padding = max(150, int((col_max - col_min) * 0.2))

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

    def _render_preview(self, dem: DEMLoader) -> None:
        row_start, row_end, col_start, col_end, downsample = self._compute_preview_window(dem)
        dem_patch = dem.data[row_start:row_end:downsample, col_start:col_end:downsample].astype(np.float32)

        if dem.nodata is not None:
            dem_patch[dem_patch == dem.nodata] = np.nan
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

        mission_points = self._current_mission_points()
        mission_points_valid = bool(mission_points) and all(dem.get_elevation(lat, lon) is not None for lat, lon in mission_points)

        if mission_points_valid and len(mission_points) >= 2:
            lats = [point[0] for point in mission_points]
            lons = [point[1] for point in mission_points]
            self.preview_axes.plot(lons, lats, "--", color="#8390fa", linewidth=1.5, label="Mission Legs")

        start = mission_points[0] if mission_points else None
        target = mission_points[-1] if len(mission_points) >= 2 else None
        mid_waypoints = mission_points[1:-1] if len(mission_points) > 2 else []

        if start and dem.get_elevation(*start) is not None:
            self.preview_axes.scatter(start[1], start[0], c="#00a86b", s=55, edgecolors="black", linewidths=0.7)
            self.preview_axes.annotate("Start", (start[1], start[0]), xytext=(6, 8), textcoords="offset points")

        for index, waypoint in enumerate(mid_waypoints, start=1):
            if dem.get_elevation(*waypoint) is None:
                continue
            self.preview_axes.scatter(waypoint[1], waypoint[0], c="#ffb000", s=45, edgecolors="black", linewidths=0.7)
            self.preview_axes.annotate(f"W{index}", (waypoint[1], waypoint[0]), xytext=(6, 8), textcoords="offset points")

        if target and dem.get_elevation(*target) is not None:
            self.preview_axes.scatter(target[1], target[0], c="#d1495b", s=55, edgecolors="black", linewidths=0.7)
            self.preview_axes.annotate("Target", (target[1], target[0]), xytext=(6, 8), textcoords="offset points")

        if self.route_points:
            route_lats = [point[0] for point in self.route_points]
            route_lons = [point[1] for point in self.route_points]
            self.preview_axes.plot(route_lons, route_lats, color="#13293d", linewidth=2.2, label="Computed Route")

        # Keep the axes locked to the DEM extent so out-of-range mission data cannot
        # force autoscaling and visually shrink the terrain image into one corner.
        self.preview_axes.set_xlim(left_lon, right_lon)
        self.preview_axes.set_ylim(bottom_lat, top_lat)
        self.preview_axes.set_title(
            f"Terrain Window ({dem_patch.shape[1]} x {dem_patch.shape[0]}, downsample {downsample}x)",
            fontsize=11,
        )
        self.preview_axes.set_xlabel("Longitude")
        self.preview_axes.set_ylabel("Latitude")
        handles, labels = self.preview_axes.get_legend_handles_labels()
        if handles:
            self.preview_axes.legend(loc="upper right")
        self.preview_figure.tight_layout()
        self.preview_canvas.draw()

        click_mode = self.map_click_mode_var.get().replace("_", " ")
        leg_status = "visible" if mission_points_valid else "hidden (outside current DEM)"
        self.preview_status_var.set(
            f"Map ready. Click mode: {click_mode}. Mission legs: {leg_status}. Route points: {len(self.route_points)}."
        )

    def refresh_preview(self) -> None:
        try:
            dem_name = self.dem_name_var.get().strip()
            if not dem_name:
                raise ValueError("No DEM file is available.")
            self.dem_loader = self._load_dem(dem_name)
            self._render_preview(self.dem_loader)
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

    def _get_pathfinding_engine(self, dem_name: str) -> Pathfinding:
        if self.pathfinder is None or self.pathfinder_dem_name != dem_name:
            self.pathfinder = Pathfinding(dem_name)
            self.pathfinder_dem_name = dem_name
        return self.pathfinder

    def _pixels_to_gps(self, pf: Pathfinding, pixel_path: list[tuple[int, int]]) -> list[tuple[float, float]]:
        route = []
        for row, col in pixel_path:
            lat, lon = pf.dem_loader.pixel_to_lat_lon(row, col)
            route.append((float(lat), float(lon)))
        return route

    def run_pathfinding(self) -> None:
        if self.pathfinding_in_progress:
            return

        try:
            plan = self._build_plan()
            self._selected_configuration()
        except Exception as exc:
            messagebox.showerror("Pathfinding Error", str(exc))
            return

        self.pathfinding_in_progress = True
        self.path_status_var.set("Pathfinding in progress...")

        worker = threading.Thread(target=self._run_pathfinding_worker, args=(plan,), daemon=True)
        worker.start()

    def _run_pathfinding_worker(self, plan: MissionPlan) -> None:
        try:
            pf = self._get_pathfinding_engine(plan.dem_name)
            mission_points = [
                (plan.start_lat, plan.start_lon),
                *[(waypoint["lat"], waypoint["lon"]) for waypoint in plan.waypoints],
                (plan.target_lat, plan.target_lon),
            ]

            for lat, lon in mission_points:
                if pf.dem_loader.get_elevation(lat, lon) is None:
                    raise ValueError(f"Coordinate outside DEM coverage: {format_gps(lat, lon)}")

            raw_route_pixels: list[tuple[int, int]] = []
            for index in range(len(mission_points) - 1):
                start_pixel = pf.dem_loader.lat_lon_to_pixel(*mission_points[index])
                end_pixel = pf.dem_loader.lat_lon_to_pixel(*mission_points[index + 1])
                segment = pf.find_path(start_pixel, end_pixel, heuristic_weight=DEFAULT_HEURISTIC_WEIGHT)
                if not segment:
                    raise RuntimeError(f"No path found for leg {index + 1}.")

                if raw_route_pixels:
                    raw_route_pixels.extend(segment[1:])
                else:
                    raw_route_pixels.extend(segment)

            route_points = self._pixels_to_gps(pf, raw_route_pixels)
            route_distance_m = polyline_distance_m(route_points)
            config = get_configuration(plan.missile_config_name)
            if not config:
                raise ValueError("Selected missile configuration could not be loaded.")
            cruise_speed_ms = create_missile_profile(config).cruise_speed
            eta_min = route_distance_m / max(cruise_speed_ms, 1e-6) / 60.0

            lines = [
                f"Plan: {plan.plan_name}",
                f"Terrain: {plan.dem_name}",
                f"Missile Config: {plan.missile_config_name}",
                "",
                "Pathfinding Result:",
                f"Leg count: {len(mission_points) - 1}",
                f"Waypoint count: {len(plan.waypoints)}",
                f"Raw route points: {len(route_points)}",
                f"Computed route distance: {route_distance_m / 1000:.2f} km",
                f"Estimated route time at cruise: {eta_min:.1f} min",
                "",
                "Status:",
                "C++ pathfinder completed successfully and route overlay was updated.",
            ]

            self.root.after(
                0,
                lambda: self._finish_pathfinding_success(route_points, "\n".join(lines), len(route_points), route_distance_m),
            )
        except Exception as exc:
            self.root.after(0, lambda: self._finish_pathfinding_error(str(exc)))

    def _finish_pathfinding_success(
        self,
        route_points: list[tuple[float, float]],
        summary_text: str,
        point_count: int,
        route_distance_m: float,
    ) -> None:
        self.route_points = route_points
        self.pathfinding_in_progress = False
        self.path_status_var.set(
            f"Pathfinding finished: {point_count} route points, {route_distance_m / 1000:.2f} km total route."
        )
        self._set_summary(summary_text)
        self.refresh_preview()

    def _finish_pathfinding_error(self, message: str) -> None:
        self.pathfinding_in_progress = False
        self.path_status_var.set(f"Pathfinding failed: {message}")
        messagebox.showerror("Pathfinding Error", message)

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
            for lat, lon in self.waypoints:
                command.extend(["--waypoint", f"{lat},{lon}"])
            subprocess.Popen(command, cwd=PROJECT_ROOT)
            self.preview_status_var.set("Opened external GPU preview window.")
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
        self.start_var.set(format_gps(payload.get("start_lat", 0.0), payload.get("start_lon", 0.0)))
        self.target_var.set(format_gps(payload.get("target_lat", 0.0), payload.get("target_lon", 0.0)))

        missile_config_name = payload.get("missile_config_name", DEFAULT_CONFIGURATION["name"])
        self.refresh_configurations(select_name=missile_config_name)

        self.waypoints = [
            (float(waypoint["lat"]), float(waypoint["lon"]))
            for waypoint in payload.get("waypoints", [])
        ]
        self._refresh_waypoint_list()
        self.route_points = []
        self._set_summary(payload.get("notes", "Plan loaded."))
        self.refresh_preview()


def main() -> None:
    root = tk.Tk()
    MissionPlannerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
