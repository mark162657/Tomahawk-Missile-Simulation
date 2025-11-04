# Advanced Missile Guidance Framework

Simulates a Tomahawk-class cruise missile guidance system over real-world satellite terrain data (DEM), featuring GPS navigation, TERCOM terrain matching, Kalman filter sensor fusion, and proportional navigation guidance. Modular by design for scalable experimentation and educational use.

---

## Overview

The project focuses on realistic simulation of missile guidance over digital elevation maps. Initial implementation provides foundational features: terrain loading, coordinate conversion, GPS/TERCOM guidance components, and classic sensor fusion. This codebase is structured for easy extension to advanced AI, radar, and complex engagement scenarios in later phases.

---

## Project Structure

```
AdvancedMissileGuidanceFramework/
├── config/               # System configuration files
├── data/
│   ├── dem/              # DEM tiles (raw/merged)
│   ├── missions/         # Mission definitions
│   └── results/          # Simulation outputs
├── src/
│   ├── terrain/
│   │   ├── dem_loader.py         # DEM file loading/query
│   │   ├── dem_merger.py         # Merge SRTM tiles
│   │   ├── database.py           # High-level terrain abstraction
│   │   └── coordinates.py        # Lat/lon <-> XY
│   ├── navigation/
│   │   ├── gps.py                # GPS sensor simulation
│   │   ├── tercom.py             # Terrain contour matching
│   │   └── kalman.py             # Kalman sensor fusion
│   ├── guidance/
│   │   ├── pathfinding.py        # A* route finding
│   │   └── proportional_nav.py   # Guidance law
│   ├── missile/
│   │   ├── profile.py            # Missile specs
│   │   └── state.py              # Missile state
│   ├── simulation/
│   │   └── engine.py             # Main sim loop
│   └── visualization/
│       ├── plotter.py            # Path visualization
│       └── metrics.py            # Performance metrics
├── docs/
│   └── missile-profile.md        # Specs and references
├── tests/
└── README.md
```

---

## Features (Initial Phase)

- Load and merge real DEM (SRTM) tiles over 500-1000 km² areas
- Convert between GPS (lat/lon) and local Cartesian (XY meters)
- Query elevation and compute terrain profiles along paths
- Simulate GPS navigation with realistic sensor noise and dropouts
- Implement TERCOM (Terrain Contour Matching) for position correction
- Foundational sensor fusion via Kalman filter
- A* pathfinding over terrain with elevation constraints
- Proportional navigation guidance toward target

**Tech Stack:** Python 3.10+ | NumPy | rasterio | matplotlib | scipy

---
<!-- 
## Getting Started

### Prerequisites
- Python 3.10 or higher
- Git (for version control)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourname/AdvancedMissileGuidanceFramework.git
   cd AdvancedMissileGuidanceFramework
   ```

2. Create and activate virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### DEM Data Setup

1. Download SRTM tiles (GeoTIFF format) from [USGS EarthExplorer](https://earthexplorer.usgs.gov)
2. Place tiles in `data/dem/raw_tiles/`
3. Merge tiles:
   ```bash
   python src/terrain/dem_merger.py
   ```
4. Test DEM loading:
   ```bash
   python src/terrain/dem_loader.py
   ```
   
   Expected output:
   ```
   DEM loaded: merged_dem.tif
   Shape: (6001, 12001)
   Bounds: BoundingBox(left=90.0, bottom=54.0, right=100.0, top=59.0)
   Elevation at (56.0, 95.0): 842.35m
   ```

### Configuration

Edit `config/config.yaml` to set:
- Missile specifications (cruise speed, turn rate, altitude constraints)
- DEM file path
- Mission start/end GPS coordinates
- Target location
- Sensor noise parameters (GPS, TERCOM)

### Run Simulation

```bash
python src/simulation/engine.py --config config/config.yaml
```

Output plots and metrics will be saved to `data/results/`

--- -->

---

## Key Components

### Terrain System (`src/terrain/`)
- **dem_loader.py:** Low-level DEM file I/O via rasterio
- **dem_merger.py:** Merge adjacent SRTM tiles into single dataset
- **database.py:** High-level TerrainDatabase abstraction for elevation queries
- **coordinates.py:** GPS to local XY conversion and haversine distance

### Navigation System (`src/navigation/`)
- **gps.py:** GPS sensor with Gaussian noise and outage simulation
- **tercom.py:** Terrain contour matching for position fixes
- **kalman.py:** Sensor fusion filter combining GPS and TERCOM measurements

### Guidance System (`src/guidance/`)
- **pathfinding.py:** A* route planning over elevation grid
- **proportional_nav.py:** Classic proportional navigation law

### Missile System (`src/missile/`)
- **profile.py:** Tomahawk specifications and flight constraints
- **state.py:** Current missile position, velocity, and attitude

### Simulation (`src/simulation/`)
- **engine.py:** Main simulation loop, integrates all subsystems

---

## Example Workflow

1. Load DEM and verify terrain bounds
2. Initialize missile state and target location
3. Convert GPS coordinates to local XY frame
4. Plan feasible route via A* pathfinding
5. Simulate GPS and TERCOM sensors
6. Fuse sensor data with Kalman filter
7. Execute guidance and propagate missile dynamics
8. Visualize path and compute accuracy metrics

---

## Future Plans

### Intermediate Extensions
- Multi-missile support with cooperative navigation
- Moving/maneuvering targets and intercept re-planning
- Radar sensor modeling and line-of-sight masking for terrain avoidance
- GPS jamming and TERCOM degradation modes with fallback strategies
- 3D terrain following autopilot for low-altitude flight (50-100m AGL)
- Advanced route planning considering fuel, time, and threat zones

### Advanced and ML Extensions
- Deep learning for terrain feature extraction and correlation
- Reinforcement learning for dynamic path planning and evasion
- Adaptive guidance switching between classical proportional navigation and learned policies
- CNN-based TERCOM matching instead of normalized cross-correlation
- Multi-agent reinforcement learning for missile swarms vs. defensive systems
- Radar/infrared seeker simulation with target tracking
- Scenario randomization and domain randomization for robustness
- Meta-learning for rapid adaptation to new terrains and sensor failures

---

## Documentation

- **docs/missile-profile.md:** Contains source data, Tomahawk specifications, flight constraints, and academic references
- Inline code comments and docstrings explain key algorithms
- Each module is self-contained and can be tested independently

---

## Development Notes

### Architecture Principles
- Separation of concerns: terrain, navigation, guidance, and missile systems are decoupled
- Data-driven configuration via YAML
- Extensible interfaces for future AI/ML additions
- Real-world satellite data over synthetic random terrain

### Testing

```bash
# Run unit tests
python -m pytest tests/

# Quick integration test
python src/terrain/dem_loader.py
python -c "from src.terrain.coordinates import CoordinateSystem; cs = CoordinateSystem(56.0, 95.0); print(cs.latlong_to_xy(56.1, 95.1))"
```

### Common Issues

**DEM not loading:**
- Verify GeoTIFF files exist in `data/dem/raw_tiles/`
- Check file paths in config.yaml
- Use `gdalinfo` to inspect TIFF metadata

**Coordinate errors:**
- Ensure origin point is within DEM bounds
- Check lat/lon order (not lon/lat)
- Verify trigonometric functions use radians

**Import errors:**
- Activate virtual environment
- Run `pip install -r requirements.txt`
- Ensure `__init__.py` exists in all package directories

---

## License

For educational, research, and demonstrative use only. No classified or proprietary military technology included. See LICENSE file.

