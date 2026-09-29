"""HUD overlays drawn into the video frame (Hydrion Spectra style).

Top-center: heading compass. Center: crosshair + pitch ladder. Left-middle:
depth pillar. Right-middle: altitude pillar. (The sonar map lives in
`ui/sonar.py` as a Qt widget so it stays aligned with the other panels.)
"""

import cv2
import numpy as np

from modules.sensors.sensor_manager import HudState

CYAN = (218, 255, 100)          # BGR of #64FFDA
BLUE = (255, 199, 173)          # BGR of #adc7ff
ON_SURFACE = (255, 227, 214)    # BGR of #d6e3ff
VARIANT = (195, 202, 186)       # BGR of #bacac3
PANEL_BG = (64, 34, 17)         # BGR of #112240


class HudOverlay:
    def __init__(self, config):
        cfg = config.get("hud", {})
        self.enabled = cfg.get("enabled", True)
        self.panel_alpha = cfg.get("panel_alpha", 0.55)
        self.crosshair_enabled = cfg.get("crosshair", {}).get("enabled", True)
        self.compass_enabled = cfg.get("compass", {}).get("enabled", True)
        self.depth_alt_enabled = cfg.get("depth_alt", {}).get("enabled", True)
        self.pitch_ladder_enabled = cfg.get("pitch_ladder", {}).get("enabled", True)
        self.roll_arc_enabled = cfg.get("roll_arc", {}).get("enabled", True)
        #: Degrees of pitch per ladder rung, and pixels of arc per degree of roll.
        self.pitch_step_deg = float(cfg.get("pitch_ladder", {}).get("step_deg", 10.0))
        self.roll_arc_span_deg = float(cfg.get("roll_arc", {}).get("span_deg", 90.0))

        self._state = None
        self._imu_valid = True
        self._depth_valid = True

        colors = cfg.get("colors", {})
        self._c = {
            "cyan": self._bgr(colors.get("primary", [100, 255, 218])),
            "blue": self._bgr(colors.get("secondary", [173, 199, 255])),
            "text": self._bgr(colors.get("on_surface", [214, 227, 255])),
            "variant": self._bgr(colors.get("on_surface_variant", [186, 202, 195])),
            "bg": self._bgr(colors.get("bg", [17, 34, 64])),
            "bg_high": self._bgr(colors.get("bg_high", [39, 53, 76])),
        }

    @staticmethod
    def _bgr(rgb):
        return tuple(int(v) for v in reversed(rgb[:3]))

    def update(self, state):
        """Cache the newest state; pure in-memory, safe to call per frame."""
        if state is None:
            self._state = None
            return
        self._state = state
        self._imu_valid = bool(getattr(state, "imu_valid", True))
        self._depth_valid = bool(getattr(state, "depth_valid", True))

    def _attitude(self, state):
        """(roll, pitch, yaw) in degrees, tolerant of a bare/None state."""
        if state is None:
            return 0.0, 0.0, 0.0
        return (
            float(getattr(state, "roll", 0.0) or 0.0),
            float(getattr(state, "pitch", 0.0) or 0.0),
            float(getattr(state, "yaw_deg", 0.0) or 0.0),
        )

    def _text(self, frame, text, org, color=None, scale=0.45, thickness=1):
        cv2.putText(frame, text, (org[0] + 1, org[1] + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 1,
                    cv2.LINE_AA)
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color or self._c["text"], thickness, cv2.LINE_AA)

    @staticmethod
    def _rounded_mask(h, w, x0, y0, x1, y1, r):
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.rectangle(mask, (x0 + r, y0), (x1 - r, y1), 255, -1)
        cv2.rectangle(mask, (x0, y0 + r), (x1, y1 - r), 255, -1)
        cv2.circle(mask, (x0 + r, y0 + r), r, 255, -1)
        cv2.circle(mask, (x1 - r, y0 + r), r, 255, -1)
        cv2.circle(mask, (x0 + r, y1 - r), r, 255, -1)
        cv2.circle(mask, (x1 - r, y1 - r), r, 255, -1)
        return mask

    def _panel(self, frame, x0, y0, x1, y1, radius=8, alpha=None, border=None):
        h, w = frame.shape[:2]
        x0, x1 = sorted((max(0, x0), min(w, x1)))
        y0, y1 = sorted((max(0, y0), min(h, y1)))
        if x1 - x0 < 4 or y1 - y0 < 4:
            return
        mask = self._rounded_mask(h, w, x0, y0, x1, y1, radius)
        blend = cv2.addWeighted(frame, 1 - (alpha or self.panel_alpha),
                                np.full((h, w, 3), self._c["bg"], dtype=np.uint8),
                                alpha or self.panel_alpha, 0)
        frame[mask > 0] = blend[mask > 0]
        erode = cv2.erode(mask, np.ones((3, 3), np.uint8))
        edge = mask - erode
        frame[edge > 0] = border or self._c["cyan"]

    def render(self, frame, state):
        if not self.enabled or frame is None:
            return frame
        if state is not None:
            self.update(state)
        h, w = frame.shape[:2]
        if h < 300 or w < 400:
            return frame
        roll, pitch, yaw = self._attitude(state)
        if self.compass_enabled:
            self._draw_compass(frame, state, yaw)
        if self.crosshair_enabled:
            self._draw_crosshair(frame, h, w)
        if self.roll_arc_enabled:
            self._draw_roll_arc(frame, roll, h, w)
        if self.pitch_ladder_enabled:
            self._draw_pitch_ladder(frame, pitch, roll, h, w)
        if self.depth_alt_enabled:
            self._draw_depth(frame, state, h)
            self._draw_alt(frame, state, h, w)
        return frame

    def _draw_compass(self, frame, state, yaw):
        cx = frame.shape[1] // 2
        vals = [yaw + d for d in (-20, -10, 0, 10, 20)]
        widths = [46, 42, 46, 42, 46]
        total = sum(widths) + 40
        x0 = cx - total // 2
        y0 = 10
        self._panel(frame, x0, y0, x0 + total, y0 + 30, radius=10)
        x = x0 + 10
        self._text(frame, "HDG", (x, y0 + 19), self._c["variant"], 0.4, 1)
        x += 34
        color = self._c["text"] if self._imu_valid else self._c["variant"]
        for i, v in enumerate(vals):
            hdg = v % 360
            if i == 2:
                self._text(frame, f"{hdg:03.0f}", (x, y0 + 21), color, 0.55, 2)
            else:
                self._text(frame, f"{hdg:03.0f}", (x, y0 + 19),
                           self._c["variant"], 0.4, 1)
            x += widths[i]

    def _draw_roll_arc(self, frame, roll, h, w):
        """Roll pointer: a fixed arc with a tick that swings with vehicle roll."""
        cx, cy = w // 2, h // 2
        radius = 132
        start, end = 200, 340
        cv2.ellipse(frame, (cx, cy), (radius, radius), 0, start, end,
                    self._c["variant"], 1, cv2.LINE_AA)
        span = max(1.0, self.roll_arc_span_deg)
        clamped = max(-span, min(span, roll))
        angle = 270.0 + (clamped / span) * 70.0
        rad = np.deg2rad(angle)
        px, py = int(cx + radius * np.cos(rad)), int(cy + radius * np.sin(rad))
        color = self._c["cyan"] if self._imu_valid else self._c["variant"]
        cv2.line(frame, (cx, cy), (px, py), color, 2, cv2.LINE_AA)
        cv2.circle(frame, (px, py), 4, color, -1, cv2.LINE_AA)
        self._text(frame, f"ROLL {roll:+05.1f}", (cx - 52, cy - radius - 8),
                   color, 0.42, 1)

    def _draw_pitch_ladder(self, frame, pitch, roll, h, w):
        """Pitch ladder: rungs slide vertically and rotate with vehicle roll."""
        if not self._imu_valid:
            return
        cx, cy = w // 2, h // 2
        step = max(1.0, self.pitch_step_deg)
        px_per_deg = 3.2
        half = 96
        base = int(np.clip(-roll, -45.0, 45.0))
        # Draw the rungs around the current pitch so the horizon stays centred.
        for rung in range(-2, 3):
            deg = base + rung * step
            offset = (deg - pitch) * px_per_deg
            y = int(cy + offset)
            if y < cy - h // 3 or y > cy + h // 3:
                continue
            length = half if rung == 0 else half - 26
            color = self._c["cyan"] if rung == 0 else self._c["variant"]
            thickness = 2 if rung == 0 else 1
            if deg >= 0:
                # Positive pitch (nose up) splits the rung and skips the centre.
                for sign in (-1, 1):
                    x_end = cx + sign * length
                    cv2.line(frame, (cx + sign * 22, y), (x_end, y),
                             color, thickness, cv2.LINE_AA)
                    if rung != 0:
                        cv2.line(frame, (x_end, y), (x_end, y + sign * 6),
                                 color, thickness, cv2.LINE_AA)
                label = f"{deg:.0f}"
                self._text(frame, label, (cx - length - 30, y + 4),
                           color, 0.38, 1)
            else:
                cv2.line(frame, (cx - length, y), (cx + length, y),
                         color, thickness, cv2.LINE_AA)
                label = f"{abs(deg):.0f}"
                self._text(frame, label, (cx + length + 6, y + 4),
                           color, 0.38, 1)
        self._text(frame, f"PITCH {pitch:+05.1f}", (cx - 52, cy + h // 2 - 18),
                   self._c["cyan"], 0.42, 1)

    def _draw_crosshair(self, frame, h, w):
        cx, cy = w // 2, h // 2
        r = 44
        cv2.circle(frame, (cx, cy), r, self._c["cyan"], 1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 2, self._c["cyan"], -1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - r + 8), (cx, cy - r + 24), self._c["cyan"], 1)
        cv2.line(frame, (cx, cy + r - 8), (cx, cy + r - 24), self._c["cyan"], 1)
        cv2.line(frame, (cx - r + 8, cy), (cx - r + 24, cy), self._c["cyan"], 1)
        cv2.line(frame, (cx + r - 8, cy), (cx + r - 24, cy), self._c["cyan"], 1)
        for off, label in ((-24, "+10"), (24, "-10")):
            y = cy - off
            cv2.line(frame, (cx - 42, y), (cx - 30, y), self._c["cyan"], 1)
            cv2.line(frame, (cx + 42, y), (cx + 30, y), self._c["cyan"], 1)
            if off < 0:
                cv2.line(frame, (cx - 34, y), (cx - 34, y), self._c["cyan"], 1)
            self._text(frame, label, (cx - 62, y + 5), self._c["cyan"], 0.35, 1)

    def _draw_depth(self, frame, state, h):
        yc = h // 2
        x0 = 10
        pw, ph = 84, 130
        self._panel(frame, x0, yc - ph // 2, x0 + pw, yc + ph // 2, radius=10)
        self._text(frame, "DEPTH", (x0 + 18, yc - 38), self._c["variant"], 0.4, 1)
        # No depth source (e.g. the IMU-only GY-95T): show "--", never a fake 0.0.
        value = f"{state.depth:5.1f}" if self._depth_valid else "  -- "
        color = self._c["cyan"] if self._depth_valid else self._c["variant"]
        self._text(frame, value, (x0 + 20, yc + 4), color, 0.6, 2)
        self._text(frame, "m" if self._depth_valid else "N/A",
                   (x0 + 36, yc + 30), self._c["variant"], 0.4, 1)

    def _draw_alt(self, frame, state, h, w):
        yc = h // 2
        pw, ph = 84, 130
        x1 = w - 10
        y0 = yc - ph // 2
        self._panel(frame, x1 - pw, y0, x1, y0 + ph, radius=10)
        self._text(frame, "ALT", (x1 - pw + 28, y0 + 24), self._c["variant"], 0.4, 1)
        value = f"{state.alt:4.1f}" if self._depth_valid else "  -- "
        color = self._c["blue"] if self._depth_valid else self._c["variant"]
        self._text(frame, value, (x1 - pw + 24, y0 + 52), color, 0.6, 2)
        self._text(frame, "m" if self._depth_valid else "N/A",
                   (x1 - pw + 34, y0 + 78), self._c["variant"], 0.4, 1)

