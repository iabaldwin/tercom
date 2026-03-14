# tercom

A 3D trajectory visualization and simulation app featuring Kalman filtering, cubic spline interpolation, and waypoint following — built with JavaScript and Three.js.

## Running

Serve the project directory with any static HTTP server:

```bash
npx serve .
```

Then open `http://localhost:3000` in a browser.

## Controls

- **WASD** — move camera
- **Mouse** — look around
- **Click** — place waypoints (click once to lock pointer)
- **Scroll** — zoom

## Tests

```bash
node tests/cubic_spline_test.js
```

## Features

- Figure-8 (lemniscate) trajectory with cubic spline smoothing
- Kalman filter for state estimation with uncertainty visualization
- Interactive waypoint placement via raycasting
- 3D coordinate axes, grid, and terrain
