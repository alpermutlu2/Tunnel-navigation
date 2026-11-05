# Demo script

Live demo for the interview: about 5 minutes, four moments, each answering one question the
reviewer is likely to have. The numbers below are seed 0 on v0.5.16, read from `serve.py`'s
`/api/run`. Other seeds give other numbers; the 300-seed ranges are in SUMMARY.

## Before the call

```bash
pip install -r requirements.txt
python tools/serve.py            # http://localhost:8000, the page shows a LIVE bar
```

- **Warm the cache.** Open every state below once, in order, so each answers instantly during the
  call. Each new run takes ~7 s on a free machine and up to ~25 s on a busy one. Results stay cached
  until the server stops.
- **Keep three fallbacks open in tabs:**
  - the static page, `https://alpermutlu2.github.io/Tunnel-navigation/sim/`: every scenario at
    seed 0, with the pressure-gradient and low-end tabs, the map switch and the k slider, and no
    server;
  - the recording `docs/media/sim_S5_seed0.mp4` (S5, map and driver's view, 10×);
  - the screenshots in `docs/media/`.
- **Do not use the in-browser Pyodide mode** (`browser.html`) in the call. It is experimental.

## Flow

### 1. The baseline: S1, a straight 3 km tunnel (~1 min)

Question it answers: *what does the app show, and how sure is it?*

- **Map panel:** play from ~15 s before the portal at 10×. The true car and the estimate sit side by
  side; the estimate carries its along-track 2σ bar. GNSS fades at the portal.
- **Driver's view:** the phone panel shows "No GPS · estimated ±2σ", the maneuver banner counting
  down, and the spoken callout when ŝ + k·σ_s reaches the callout point.
- **Numbers to point at:** at the exit diverge the error is 10 m with σ 26 m; at the portal 4.6 m with
  σ 16 m. The branch is decided "branch".
- **At the exit:** GNSS returns with multipath and the follower closes the gap with no jump on the
  screen.
- **Say:** "the only unknown is the distance along the road; 22 m is 1 s of callout at 80 km/h."

### 2. A live parameter change: S4 stop-and-go, then the k slider (~1 min)

Question it answers: *what happens when the driving is not ideal, and why does the callout err
early?*

- **Switch to S4.** The banner marks the ZUPTs at the stops. At the diverge the error is −14 m with σ
  58 m, and σ widens through the stops.
- **Drag the k slider from 1 to 0.** The callout moves later, toward the filter's own timing; with k = 0,
  up to 32 % (S4) and 66 % (S5) of callouts per set of 50 came more than 1 s late. Back to 1: early by about σ.
- **Say:** "late can make the driver miss the exit, early costs a second of reaction time; k·σ buys
  'never late' with the filter's own uncertainty, so σ has to be honest, and (e/σ)² is the check."

### 3. The environment fools it: S3 with a 30 Pa/km pressure gradient (~1.5 min)

Question it answers: *where does it break, and what did you do about it?*

- **LIVE bar:** scenario S3, tunnel pressure gradient 30 Pa/km, then hybrid **off**. The ESKF reads
  the pressure gradient as a climb:
  - at the diverge, 182 m off with σ 28 m, so σ does not cover the error;
  - it decides "main", the wrong branch;
  - at the portal it is still 115 m off.
- **Hybrid back to "class default" (on):**
  - at the diverge, 3.5 m off with σ 8 m, and it decides "branch", the right one;
  - at the portal, 0.4 m off.
- **Say:** "in a straight, constant-grade tunnel a pitch error and a pressure gradient give the same
  barometer signal. I did not add a filter, I added a source: the map's grade as a weak pitch
  measurement, with the gradient and the map's grade error as states. If the map has no grade, it
  turns off and the old filter comes back exactly. The gradient itself is an assumption to measure
  in a real tunnel."

### 4. A cheap phone: S4 with the low-end IMU (~1.5 min)

Question it answers: *why three filters, and what does the app actually show?*

- **LIVE bar:** IMU "low-end MPU9250", scenario S4. Turn on all traces:
  - the 16-state ESKF (orange) ends 418 m behind the car at the diverge and 570 m behind at the
    portal: at crawling speed the cheap gyro's tilt leaks gravity into the speed;
  - the app's trace (pink) is the 10-state filter, selected by the detected IMU class: 14 m at the
    diverge and 11 m at the portal.
- **The callout** on this phone uses k = 0.7. It is shown on the k slider, which starts at the class
  value.
- **Say:** "the supervisor runs all three filters on the same data and picks by IMU class. When the
  selected filter disagrees with both others it raises the shown σ. It cannot see a failure in a part
  the filters share."

### Optional, if asked

- **Any tunnel:** the custom-tunnel row (length, grade, speed, one stop) builds a new tab.
- **Every scenario at once:** the Results tab, under the current settings. "Re-run every tab" takes
  ~70 s.
- **Map off:** the map switch shows heading drifting in a straight tunnel (26 m cross-track on S1,
  106 m on 5 km).
- **More switches** (cruise model, NHC turn gate, re-acquisition, stop detector):
  `http://localhost:8000/controls`.
- **A shareable view:** the address keeps the state (`#tab=…&t=…`).

## If something fails

| Symptom | Do |
|---|---|
| No LIVE bar | The server is not running or the page was opened from disk; restart `serve.py` and reload `localhost:8000` |
| A run takes long | It is computing (~7–25 s). Talk through the previous state, it is cached |
| The server crashes | Switch to the static page tab (same scenarios at seed 0, k slider works) |
| No network for the static page | Play `docs/media/sim_S5_seed0.mp4` and use the deck's results slides |
