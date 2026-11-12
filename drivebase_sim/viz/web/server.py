"""Tiny local server for the rover replay viewer (training live feed or evaluation traces).

usage: python viz/web/server.py <feed_dir> [--port 8791]

<feed_dir> is a run's live/ folder or results/r4/traces/ (format: docs/LIVE_FORMAT.md).
Serves index.html and rover.glb from this folder, and the feed as JSON:
  GET /api/state        meta.json, one summary per episode file, progress.csv rows
  GET /api/ep/<name>    one episode's arrays
Episodes are parsed once and cached by modification time. No other process is started.
"""
import argparse
import csv
import json
import math
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCALARS = ("timesteps", "ep_return", "mean_yaw_deg_s", "drift_m")
ARRAYS = ("t", "pos", "quat", "yaw_rate", "yaw_cmd", "current", "error", "bus_v")
_cache = {}


def finite(x):
    return None if x is None or not math.isfinite(x) else x


def load(path):
    key = (path.name, path.stat().st_mtime_ns)
    if key in _cache:
        return _cache[key]
    with np.load(path) as z:
        summary = {k: finite(float(z[k])) if k in z else None for k in SCALARS}
        faults = {}
        if "faults" in z:
            try:
                faults = json.loads(str(z["faults"]))
            except ValueError:
                faults = {"unreadable": str(z["faults"])}
        body = {k: np.round(np.asarray(z[k], float), 5).tolist() for k in ARRAYS if k in z}
        current = np.abs(np.asarray(z["current"], float)) if "current" in z else np.zeros((1, 6))
    summary.update(name=path.stem, faults=faults, seconds=body["t"][-1] if body.get("t") else 0,
                   peak_current_A=float(current.max()))
    body["summary"] = summary
    _cache[key] = (summary, body)
    return summary, body


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, feed, **kw):
        self.feed = feed
        super().__init__(*a, directory=str(HERE), **kw)

    def log_message(self, *args):
        pass

    def send_json(self, obj):
        data = json.dumps(obj, allow_nan=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith("/api/state"):
            episodes = []
            for p in sorted(self.feed.glob("*.npz")):
                try:
                    episodes.append(load(p)[0])
                except Exception:
                    continue
            meta = {}
            try:
                meta = json.loads((self.feed / "meta.json").read_text())
            except (OSError, ValueError):
                pass
            progress = []
            try:
                with (self.feed / "progress.csv").open(newline="") as f:
                    for row in csv.DictReader(f):
                        s, r = row.get("time/total_timesteps"), row.get("rollout/ep_rew_mean")
                        if s and r:
                            progress.append([float(s), float(r)])
            except (OSError, ValueError):
                pass
            return self.send_json({"meta": meta, "episodes": episodes, "progress": progress, "feed": str(self.feed)})
        if self.path.startswith("/api/ep/"):
            name = Path(self.path.split("/api/ep/", 1)[1].split("?")[0]).name
            path = self.feed / (name + ".npz")
            if not path.exists():
                self.send_error(404)
                return
            return self.send_json(load(path)[1])
        return super().do_GET()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("feed_dir")
    ap.add_argument("--port", type=int, default=8791)
    args = ap.parse_args()
    feed = Path(args.feed_dir)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, feed=feed))
    print(f"rover viewer on http://localhost:{args.port}  (feed {feed})", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
