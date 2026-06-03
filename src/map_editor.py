#!/usr/bin/env python3
"""2D obstacle map editor for ackermann_rover.

Draw walls, rectangles, and freeform polygons. Save to JSON that the
lidar_driver and rover_visualizer can load.

Tools:
  Line    – click start, click end
  Rect    – click corner, click opposite corner
  Wall    – click and drag
  Polygon – click vertices, right-click to close

Usage:
  ros2 run ackermann_rover map_editor [map_name]
  python3 map_editor.py
"""
import json
import sys
import os
import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.widgets import Button, RadioButtons


class MapEditor:
    def __init__(self, filename="my_map.json"):
        self.filename = filename
        self.segments = []
        self._temp_verts = []
        self._mode = "line"
        self._drag_active = False
        self._drag_start = None

        self.fig = plt.figure("Map Editor", figsize=(10, 8))
        self.ax = self.fig.add_subplot(111)
        self._build_ui()
        self._connect_events()
        self._redraw()
        self._print_help()

    # ── drawing ──────────────────────────────────────────
    def _draw_grid(self):
        self.ax.clear()
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-8, 8)
        self.ax.set_ylim(-8, 8)
        self.ax.grid(True, alpha=0.25, linestyle="--")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title(f"Map Editor — {self.filename}  [mode: {self._mode}]")
        self.ax.axhline(0, color="gray", alpha=0.4, linewidth=0.5)
        self.ax.axvline(0, color="gray", alpha=0.4, linewidth=0.5)

    def _redraw(self):
        self._draw_grid()
        for (x1, y1), (x2, y2) in self.segments:
            self.ax.plot([x1, x2], [y1, y2], color="#6B4226",
                         linewidth=3, alpha=0.8, solid_capstyle="round")
        if self._temp_verts and len(self._temp_verts) >= 2:
            xs = [v[0] for v in self._temp_verts]
            ys = [v[1] for v in self._temp_verts]
            self.ax.plot(xs, ys, "m--", linewidth=1.5)
            self.ax.scatter(xs, ys, s=30, c="magenta", zorder=20)
        self.fig.canvas.draw_idle()

    # ── tools ────────────────────────────────────────────
    def _commit_segment(self, p1, p2):
        if abs(p1[0] - p2[0]) < 1e-6 and abs(p1[1] - p2[1]) < 1e-6:
            return
        self.segments.append((p1, p2))
        print(f"  segment: ({p1[0]:.2f},{p1[1]:.2f}) -> ({p2[0]:.2f},{p2[1]:.2f})")

    def _add_rect(self, corner_a, corner_b):
        x1, y1 = min(corner_a[0], corner_b[0]), min(corner_a[1], corner_b[1])
        x2, y2 = max(corner_a[0], corner_b[0]), max(corner_a[1], corner_b[1])
        if abs(x2 - x1) < 1e-6 or abs(y2 - y1) < 1e-6:
            return
        self._commit_segment((x1, y1), (x2, y1))
        self._commit_segment((x2, y1), (x2, y2))
        self._commit_segment((x2, y2), (x1, y2))
        self._commit_segment((x1, y2), (x1, y1))
        print(f"  rect: ({x1:.2f},{y1:.2f}) {x2-x1:.2f}x{y2-y1:.2f}")

    def _close_polygon(self):
        if len(self._temp_verts) >= 3:
            for i in range(len(self._temp_verts) - 1):
                self._commit_segment(self._temp_verts[i], self._temp_verts[i + 1])
            self._commit_segment(self._temp_verts[-1], self._temp_verts[0])
            print(f"  polygon: {len(self._temp_verts)} vertices, closed")
        self._temp_verts.clear()

    # ── mouse events ─────────────────────────────────────
    def _connect_events(self):
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_move)

    def _on_press(self, event):
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return

        if self._mode == "line":
            if len(self._temp_verts) == 0:
                self._temp_verts.append((x, y))
            else:
                self._commit_segment(self._temp_verts[0], (x, y))
                self._temp_verts.clear()

        elif self._mode == "rect":
            if len(self._temp_verts) == 0:
                self._temp_verts.append((x, y))
            else:
                self._add_rect(self._temp_verts[0], (x, y))
                self._temp_verts.clear()

        elif self._mode == "wall":
            self._drag_active = True
            self._drag_start = (x, y)
            self._temp_verts = [(x, y), (x, y)]

        elif self._mode == "polygon":
            self._temp_verts.append((x, y))

        self._redraw()

    def _on_release(self, event):
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            x, y = 0.0, 0.0

        if self._mode == "wall" and self._drag_active:
            self._drag_active = False
            if self._drag_start:
                self._commit_segment(self._drag_start, (x, y))
            self._temp_verts.clear()
            self._drag_start = None

        self._redraw()

    def _on_move(self, event):
        if not self._drag_active:
            return
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
        self._temp_verts = [self._drag_start, (x, y)] if self._drag_start else []
        self._redraw()

    def on_key(self, event):
        if event.key == "enter" and self._mode == "polygon" and self._temp_verts:
            self._close_polygon()
            self._redraw()
        elif event.key == "escape":
            self._temp_verts.clear()
            self._redraw()
        elif event.key == "ctrl+z":
            self._undo()
        elif event.key == "l":
            self._mode = "line"
            self._update_mode_button("Line")
            self._redraw()
        elif event.key == "r":
            self._mode = "rect"
            self._update_mode_button("Rect")
            self._redraw()
        elif event.key == "w":
            self._mode = "wall"
            self._update_mode_button("Wall")
            self._redraw()
        elif event.key == "p":
            self._mode = "polygon"
            self._update_mode_button("Polygon")
            self._redraw()
        elif event.key == "ctrl+s":
            self.save()

    # ── button UI ────────────────────────────────────────
    def _build_ui(self):
        self.fig.subplots_adjust(bottom=0.15)
        ax_width = 0.10
        gap = 0.005
        y_pos = 0.04

        def make_button(label, x, callback):
            ax_btn = self.fig.add_axes([x, y_pos, ax_width, 0.055])
            btn = Button(ax_btn, label)
            btn.label.set_fontsize(8)
            btn.on_clicked(callback)
            return btn

        self._mode_buttons = {}
        for i, mode in enumerate(["line", "rect", "wall", "polygon"]):
            x = 0.05 + i * (ax_width + gap)
            self._mode_buttons[mode] = make_button(
                mode.capitalize(), x, lambda evt, m=mode: self._set_mode(m))

        make_button("Undo", 0.05 + 4 * (ax_width + gap),
                     lambda evt: self._undo())
        make_button("Save", 0.05 + 5 * (ax_width + gap),
                     lambda evt: self.save())
        make_button("Clear", 0.05 + 6 * (ax_width + gap),
                     lambda evt: self._clear())

    def _set_mode(self, mode):
        self._mode = mode
        self._temp_verts.clear()
        self._drag_active = False
        self._redraw()

    def _update_mode_button(self, mode):
        self._mode = mode

    def _undo(self):
        if self.segments:
            removed = self.segments.pop()
            print(f"  undo: removed ({removed[0][0]:.2f},{removed[0][1]:.2f})")
            self._redraw()

    def _clear(self):
        self.segments.clear()
        self._temp_verts.clear()
        print("  cleared")
        self._redraw()

    # ── save / load ──────────────────────────────────────
    def save(self, path=None):
        if path is None:
            path = self.filename
        # Always save .segments (primary format for lidar_driver + visualizer)
        seg_path = os.path.splitext(path)[0] + ".segments"
        with open(seg_path, "w") as f:
            f.write(f"# Map: {os.path.splitext(os.path.basename(path))[0]}\n")
            for p1, p2 in self.segments:
                f.write(f"{p1[0]:.4f} {p1[1]:.4f} {p2[0]:.4f} {p2[1]:.4f}\n")
        # Also save JSON for editor re-load (but NOT to same path if .segments)
        json_path = os.path.splitext(path)[0] + ".json"
        if path == json_path:
            json_path = path
        data = {
            "name": os.path.splitext(os.path.basename(path))[0],
            "segments": [
                {"x1": round(p1[0], 4), "y1": round(p1[1], 4),
                 "x2": round(p2[0], 4), "y2": round(p2[1], 4)}
                for p1, p2 in self.segments
            ]
        }
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)
        self.filename = path
        print(f"  saved {len(self.segments)} segments → {seg_path} (+ {json_path})")

    def load(self, path):
        self.segments.clear()
        # Try .segments format first, fall back to JSON
        seg_path = os.path.splitext(path)[0] + ".segments"
        try:
            with open(seg_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = line.split()
                    if len(parts) >= 4:
                        self.segments.append(((float(parts[0]), float(parts[1])),
                                               (float(parts[2]), float(parts[3]))))
            if self.segments:
                self.filename = seg_path
                self._redraw()
                print(f"  loaded {len(self.segments)} segments ← {seg_path}")
                return
        except Exception:
            pass
        # Fall back to JSON
        try:
            with open(path, "r") as f:
                data = json.load(f)
            for seg in data.get("segments", []):
                self.segments.append(((seg["x1"], seg["y1"]), (seg["x2"], seg["y2"])))
            self.filename = path
            self._redraw()
            print(f"  loaded {len(self.segments)} segments ← {path}")
        except Exception as e:
            print(f"  load failed: {e}")

    def _print_help(self):
        print("=" * 58)
        print("  Map Editor — controls")
        print("  ─────────────────────")
        print("  L key       Line mode (click start → click end)")
        print("  R key       Rect mode (corner → opposite corner)")
        print("  W key       Wall mode (click & drag)")
        print("  P key       Polygon mode (click vertices, Enter to close)")
        print("  Esc         Cancel current shape")
        print("  Ctrl+Z      Undo last segment")
        print("  Ctrl+S      Save map")
        print("  Right-click Polygon mode: close shape")
        print("=" * 58)
        print("  Default map: 8x8m grid, origin at center")
        print()


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else "my_map.segments"
    editor = MapEditor(filename)
    editor.fig.canvas.mpl_connect("key_press_event", editor.on_key)
    plt.show()


if __name__ == "__main__":
    main()
