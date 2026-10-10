"""UAV flight simulator - pygame front end.

Run:  python main.py
Controls: W/S up/down, A/D left/right, TAB flight mode, R reset, M mute, H HUD detail
"""

import csv
import math
import os
import sys

import pygame

from uavsim import Mission, LEVELS, MODES, AUTO, RATE, ASSIST
from uavsim.mission import WORLD_W, CHECKPOINT_R, ZONE_R, HOVER_TIME

WIDTH, HEIGHT = 900, 600
FPS = 60
SCALE = 50                       # px per metre
GROUND_Y = HEIGHT - 50           # screen y of z = 0
BG = (15, 15, 25)

pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=512)
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("UAV Flight Simulator")
clock = pygame.time.Clock()
font = pygame.font.SysFont("monospace", 14)
font_md = pygame.font.SysFont("monospace", 17)
font_lg = pygame.font.SysFont("monospace", 22, bold=True)


def px(x, z):
    return int(x * SCALE), int(GROUND_Y - z * SCALE)


# Audio. Every file is optional; a missing one just stays silent.

SOUNDS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sounds")


def load_sound(name, volume=1.0):
    path = os.path.join(SOUNDS_DIR, name)
    if not os.path.isfile(path):
        return None
    try:
        snd = pygame.mixer.Sound(path)
        snd.set_volume(volume)
        return snd
    except pygame.error as e:
        print(f"audio: could not load {path}: {e}")
        return None


try:
    pygame.mixer.init()
    audio_ok = True
except pygame.error:
    audio_ok = False

snd = {}
if audio_ok:
    snd = {k: load_sound(f"{k}.mp3", v) for k, v in
           [("checkpoint", 0.7), ("complete", 0.8), ("crash", 0.8), ("motor", 1.0), ("wind", 1.0)]}
    motor_ch, wind_ch, crash_ch = (pygame.mixer.Channel(i) for i in range(3))
    for ch, key in ((motor_ch, "motor"), (wind_ch, "wind")):
        if snd.get(key):
            ch.play(snd[key], loops=-1)
            ch.set_volume(0.0)

muted = False
motor_vol = wind_vol = 0.0


def play(key):
    if audio_ok and not muted and snd.get(key):
        (crash_ch.play(snd[key]) if key == "crash" else snd[key].play())


def update_loops(m, active):
    """Motor hum follows mean rotor speed, wind noise follows airspeed."""
    global motor_vol, wind_vol
    if not audio_ok or muted:
        return
    q = m.quad
    mt = wt = 0.0
    if active:
        w = 0.5 * (q.wl + q.wr) / q.v.omega_max_nominal
        mt = min(1.0, 0.05 + 0.85 * w * w)
        wt = min(0.65, 0.08 * math.hypot(m.wind.vx, m.wind.vz))
    motor_vol += (mt - motor_vol) * 0.6
    wind_vol += (wt - wind_vol) * 0.4
    if snd.get("motor"):
        motor_ch.set_volume(motor_vol)
    if snd.get("wind"):
        wind_ch.set_volume(wind_vol)


# Flight logs: reports/flightN/flight_log.csv, one folder per session.
# The folder is only created once there is something to save.

REPORTS_DIR = "reports"
session_dir = None


def save_log(m, pilot):
    global session_dir
    if not m.log:
        return
    if session_dir is None:
        os.makedirs(REPORTS_DIR, exist_ok=True)
        i = 1
        while os.path.exists(os.path.join(REPORTS_DIR, f"flight{i}")):
            i += 1
        session_dir = os.path.join(REPORTS_DIR, f"flight{i}")
        os.makedirs(session_dir)
        with open(os.path.join(session_dir, "pilot.txt"), "w", encoding="utf-8") as f:
            f.write(pilot)
    path = os.path.join(session_dir, "flight_log.csv")
    new = not os.path.isfile(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=m.log[0].keys())
        if new:
            w.writeheader()
        w.writerows(m.log)
    print(f"saved {len(m.log)} samples (level {m.level}) to {path}")


# Drawing

def draw_world(m):
    screen.fill(BG)
    # 1 m grid, faint, so distances on screen mean something
    for i in range(1, int(WORLD_W) + 1):
        pygame.draw.line(screen, (22, 22, 36), (i * SCALE, 0), (i * SCALE, GROUND_Y))
    for j in range(1, 12):
        pygame.draw.line(screen, (22, 22, 36), (0, GROUND_Y - j * SCALE), (WIDTH, GROUND_Y - j * SCALE))
    pygame.draw.rect(screen, (25, 25, 25), (0, GROUND_Y, WIDTH, HEIGHT - GROUND_Y))
    pygame.draw.line(screen, (80, 80, 80), (0, GROUND_Y), (WIDTH, GROUND_Y), 2)
    for x0, z0, x1, z1 in m.walls:
        a, b = px(x0, max(z1, 0)), px(x1, max(z0, 0))
        r = pygame.Rect(a[0], a[1], b[0] - a[0], b[1] - a[1])
        pygame.draw.rect(screen, (60, 20, 20), r)
        pygame.draw.rect(screen, (200, 60, 60), r, 2)


def pulse(t, amp):
    return int(abs(math.sin(t * 3)) * amp)


def draw_progress_arc(center, radius, frac):
    r = pygame.Rect(center[0] - radius, center[1] - radius, 2 * radius, 2 * radius)
    pygame.draw.arc(screen, (255, 255, 0), r, math.pi / 2 - frac * 2 * math.pi, math.pi / 2, 4)


def draw_objectives(m, t):
    if m.level in (1, 2):
        if m.cp_idx >= len(m.cfg["checkpoints"]):
            return
        c = px(*m.cfg["checkpoints"][m.cp_idx])
        r = int(CHECKPOINT_R * SCALE)
        pygame.draw.circle(screen, (20, 20, 60), c, r)
        pygame.draw.circle(screen, (255, 255, 255), c, r, 3)
        pygame.draw.circle(screen, (200, 200, 255), c, r + pulse(t, 10), 1)
        lbl = font.render(str(m.cp_idx + 1), True, (255, 255, 255))
        screen.blit(lbl, (c[0] - lbl.get_width() // 2, c[1] - lbl.get_height() // 2))
        return

    r = int(ZONE_R * SCALE)
    if m.delivered:
        c = px(*m.cfg["dropzone"])
        pygame.draw.circle(screen, (20, 60, 20), c, r)
        pygame.draw.circle(screen, (100, 255, 100), c, r, 3)
        return
    if not m.cargo:
        c, col, dark, text = px(*m.cfg["pickup"]), (60, 220, 80), (10, 40, 10), "PICKUP 0.6 kg"
    else:
        c, col, dark, text = px(*m.cfg["dropzone"]), (255, 200, 50), (40, 30, 10), "DELIVER"
    pygame.draw.circle(screen, dark, c, r)
    pygame.draw.circle(screen, col, c, r, 3)
    pygame.draw.circle(screen, col, c, r + pulse(t, 8), 1)
    lbl = font.render(text, True, col)
    screen.blit(lbl, (c[0] - lbl.get_width() // 2, c[1] + r + 6))
    if m.hover_t > 0:
        draw_progress_arc(c, r + 10, m.hover_t / HOVER_TIME)


def draw_target_pointer(m):
    """Small arrow next to the drone pointing at the current objective."""
    goal = m.objective()
    if goal is None or m.fc.mode == AUTO:
        return
    dx, dz = goal[0] - m.quad.x, goal[1] - m.quad.z
    d = math.hypot(dx, dz)
    if d < 2.0:
        return
    ux, uy = dx / d, -dz / d
    x, y = px(m.quad.x, m.quad.z)
    tip = (x + ux * 84, y + uy * 84)
    base = (x + ux * 70, y + uy * 70)
    left = (base[0] - uy * 7, base[1] + ux * 7)
    right = (base[0] + uy * 7, base[1] - ux * 7)
    pygame.draw.polygon(screen, (255, 220, 80), [tip, left, right])


def draw_autopilot_path(m):
    if m.fc.mode != AUTO or not m._path:
        return
    pts = [px(m.quad.x, m.quad.z)] + [px(*p) for p in m._path]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) // 8))
        for k in range(0, n, 2):
            p0 = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            p1 = (a[0] + (b[0] - a[0]) * (k + 1) / n, a[1] + (b[1] - a[1]) * (k + 1) / n)
            pygame.draw.line(screen, (90, 160, 120), p0, p1, 1)
    for p in pts[1:]:
        pygame.draw.circle(screen, (90, 160, 120), p, 3)


def draw_drone(m):
    """Pseudo-3D rear view: two front rotors and two smaller back rotors.
    Drawn about 2x physical size so it reads at this zoom level."""
    q = m.quad
    x, y = px(q.x, q.z)
    a = q.theta
    ca, sa = math.cos(a), math.sin(a)
    upx, upy = sa, -ca                 # body "up" in screen coordinates
    t_max = q.v.side_thrust_max()
    tl, tr = q.side_thrust(q.wl) / t_max, q.side_thrust(q.wr) / t_max
    front, back, back_off = 38, 26, 10

    fl = (x - front * ca, y - front * sa)
    fr = (x + front * ca, y + front * sa)
    bl = (x - back * ca - back_off * upx, y - back * sa - back_off * upy)
    br = (x + back * ca - back_off * upx, y + back * sa - back_off * upy)

    for p in (bl, br):
        pygame.draw.line(screen, (90, 90, 90), (x, y), p, 2)
        pygame.draw.circle(screen, (int(40 + 140 * tl), 120, 180), p, 7)
        pygame.draw.circle(screen, (150, 150, 150), p, 7, 1)
    for p, t in ((fl, tl), (fr, tr)):
        pygame.draw.line(screen, (170, 170, 170), (x, y), p, 3)
        n = int(t * 40)
        if n > 2:
            pygame.draw.line(screen, (255, 140, 30), p, (p[0] + upx * n, p[1] + upy * n), 2)
    for p, t in ((fl, tl), (fr, tr)):
        pygame.draw.circle(screen, (int(60 + 195 * min(t, 1)), 180, 255), p, 10)
        pygame.draw.circle(screen, (220, 220, 220), p, 10, 1)

    body = (180, 120, 20) if m.cargo else (40, 40, 180)
    pygame.draw.circle(screen, body, (x, y), 10)
    pygame.draw.circle(screen, (100, 100, 255), (x, y), 10, 1)
    pygame.draw.circle(screen, (255, 60, 60), fr, 4)
    if m.cargo:
        # payload hangs 0.12 m below the CG along the body axis
        d = 0.12 * SCALE * 2
        cx, cy = x - upx * d, y - upy * d
        box = pygame.Rect(int(cx) - 6, int(cy) - 5, 12, 10)
        pygame.draw.line(screen, (150, 120, 50), (x, y), (cx, cy), 1)
        pygame.draw.rect(screen, (200, 160, 50), box)
        pygame.draw.rect(screen, (255, 200, 80), box, 1)


def hbar(x, y, w, h, frac, color):
    pygame.draw.rect(screen, (30, 30, 30), (x, y, w, h))
    pygame.draw.rect(screen, color, (x, y, int(w * max(0.0, min(frac, 1.0))), h))
    pygame.draw.rect(screen, (100, 100, 100), (x, y, w, h), 1)


def draw_instruments(m):
    """Throttle, battery and wind live in the ground strip so they never
    cover a checkpoint."""
    q, b, fc = m.quad, m.batt, m.fc
    label = (150, 150, 150)
    x0, y1, y2 = 520, GROUND_Y + 8, GROUND_Y + 27

    if fc.mode in (AUTO, ASSIST):
        thr = (q.side_thrust(q.wl) + q.side_thrust(q.wr)) / (2 * q.v.side_thrust_max())
    else:
        thr = fc.throttle
    screen.blit(font.render("THR", True, label), (x0, y1 - 2))
    hbar(x0 + 36, y1, 120, 10, thr, (80, 200, 255))
    hx = x0 + 36 + int(120 * fc.hover_throttle())
    pygame.draw.line(screen, (255, 220, 80), (hx, y1 - 3), (hx, y1 + 12), 2)
    screen.blit(font.render(f"{thr * 100:3.0f}%", True, label), (x0 + 164, y1 - 2))

    soc = max(b.soc, 0.0)
    col = (60, 220, 80) if soc > 0.3 else (255, 200, 50) if soc > 0.1 else (255, 60, 60)
    screen.blit(font.render("BAT", True, label), (x0, y2 - 2))
    hbar(x0 + 36, y2, 120, 10, soc, col)
    screen.blit(font.render(f"{soc * 100:3.0f}% {b.voltage:4.1f}V", True, col), (x0 + 164, y2 - 2))

    cx, cy = WIDTH - 24, GROUND_Y + 25
    pygame.draw.circle(screen, (40, 40, 60), (cx, cy), 17, 1)
    if not m.wind.enabled:
        screen.blit(font.render("WIND OFF", True, (80, 80, 100)), (cx - 130, y1 - 2))
        return
    wx, wz = m.wind.vx, m.wind.vz
    speed = math.hypot(wx, wz)
    if speed > 0.05:
        ex, ey = cx + wx / speed * 14, cy - wz / speed * 14
        pygame.draw.line(screen, (100, 180, 255), (cx, cy), (ex, ey), 2)
        pygame.draw.circle(screen, (100, 180, 255), (int(ex), int(ey)), 3)
    screen.blit(font.render(f"WIND {speed:3.1f}m/s", True, (100, 180, 255)), (cx - 130, y1 - 2))


def draw_hud(m, detail):
    q, b, fc = m.quad, m.batt, m.fc
    speed = math.hypot(q.vx, q.vz)
    rpm = lambda w: w * 60 / (2 * math.pi)
    grey, blue = (200, 200, 200), (80, 200, 255)
    mode_col = {AUTO: (90, 220, 140), RATE: (255, 120, 120), ASSIST: (80, 200, 255)}.get(fc.mode, (180, 100, 255))
    lines = [
        (f"Mode      {fc.mode}  [TAB]", mode_col),
        (f"Altitude  {q.z:6.2f} m", grey),
        (f"Speed     {speed:6.2f} m/s", grey),
        (f"Tilt      {math.degrees(q.theta):6.1f} deg", grey),
        (f"Motor L   {rpm(q.wl):6.0f} rpm", blue),
        (f"Motor R   {rpm(q.wr):6.0f} rpm", blue),
        (f"Power     {b.power:6.0f} W", (255, 180, 90)),
        (f"Time      {m.t:6.1f} s", (255, 220, 80)),
    ]
    if detail:
        lines[4:4] = [(f"Tilt cmd  {math.degrees(fc.theta_cmd):6.1f} deg", grey)]
        lines += [
            (f"Thrust    {q.side_thrust(q.wl) + q.side_thrust(q.wr):6.2f} N", blue),
            (f"Mass      {q.mass:6.2f} kg", grey),
            (f"Current   {b.current:6.1f} A", (255, 180, 90)),
            (f"Energy    {b.energy_used_wh:6.2f} Wh", (255, 180, 90)),
        ]
    if m.level == 3:
        status = "DELIVERED" if m.delivered else "CARRYING" if m.cargo else "GO TO PICKUP"
        lines.append((f"Cargo     {status}", (80, 255, 120) if m.cargo else grey))
    else:
        n = len(m.cfg["checkpoints"])
        lines.append((f"Target    {min(m.cp_idx + 1, n)} / {n}", (255, 255, 255)))
    lines.append((f"Level     {m.level}", (180, 100, 255)))
    for i, (text, col) in enumerate(lines):
        screen.blit(font.render(text, True, col), (12, 12 + i * 19))

    ctrl = font.render("W/S up/dn  A/D move  TAB mode  R reset  H info ", True, (70, 70, 90))
    ml = font.render("M mute" if not muted else "MUTED", True, (200, 80, 80) if muted else (70, 70, 90))
    screen.blit(ctrl, (10, HEIGHT - 30))
    screen.blit(ml, (10 + ctrl.get_width(), HEIGHT - 30))


def overlay(alpha):
    s = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    s.fill((0, 0, 0, alpha))
    screen.blit(s, (0, 0))


def centered(text, f, color, y):
    s = f.render(text, True, color)
    screen.blit(s, (WIDTH // 2 - s.get_width() // 2, y))
    return f.size("A")[1] + 4


BRIEFINGS = {
    1: [("OBJECTIVE", "h"), ("Fly through the 10 checkpoints in order.", ""),
        ("", ""), ("FLIGHT MODES  (TAB)", "h"),
        ("ASSIST - W/S climb and descend, A/D fly sideways.", ""),
        ("         Let go and it stops and holds position.", ""),
        ("ANGLE  - A/D sets a tilt angle, W/S sets throttle.", ""),
        ("RATE   - A/D sets a tilt rate. No self-levelling. Hard.", ""),
        ("AUTO   - the autopilot flies to the target.", ""),
        ("", ""), ("Arrow keys work too. Hold SHIFT for slow, precise moves.", "n"),
        ("The yellow arrow next to the drone points to the next target.", "n")],
    2: [("OBJECTIVE", "h"), ("Fly through the gates to all 5 checkpoints.", ""),
        ("Touching a wall is a crash.", "w"), ("", ""),
        ("CROSSWIND", "h"), ("5 m/s mean wind with gusts, starts after 2 s.", ""),
        ("Wind acts through drag on airspeed: lean into it.", "")],
    3: [("OBJECTIVE", "h"), ("Hover over the green zone for 1.5 s to load 0.6 kg,", ""),
        ("then hover over the yellow zone to deliver it.", ""), ("", ""),
        ("WHAT CHANGES", "h"),
        ("Payload: +50% mass, more inertia, slower attitude response.", ""),
        ("Battery at 18%: voltage sags under load and cuts thrust.", "w"),
        ("Wind starts rotating once the cargo is aboard.", "n")],
}


def draw_briefing(level):
    overlay(210)
    centered(f"LEVEL {level}: {LEVELS[level]['name'].upper()}", font_lg, (80, 200, 255), 80)
    pygame.draw.line(screen, (50, 50, 80), (100, 125), (WIDTH - 100, 125), 1)
    y = 145
    colors = {"h": (255, 220, 80), "w": (255, 120, 90), "n": (100, 180, 255), "": (200, 200, 200)}
    for text, kind in BRIEFINGS[level]:
        if not text:
            y += 8
            continue
        y += centered(text, font_md if kind == "h" else font, colors[kind], y)
    pygame.draw.line(screen, (50, 50, 80), (100, y + 6), (WIDTH - 100, y + 6), 1)
    p = int(abs(math.sin(pygame.time.get_ticks() / 500)) * 40) + 180
    centered("Press SPACE to start", font_md, (p, p, p), y + 20)


def draw_result(m, times, energies):
    overlay(185)
    y = HEIGHT // 2 - 90
    if m.level < 3:
        centered(f"LEVEL {m.level} COMPLETE", font_lg, (80, 255, 120), y)
        centered(f"Time {m.t:.1f} s    Energy {m.batt.energy_used_wh:.2f} Wh", font_md, (255, 220, 80), y + 50)
        centered("Press SPACE for the next level", font_md, (200, 200, 200), y + 100)
    else:
        centered("MISSION COMPLETE", font_lg, (255, 220, 80), y)
        centered("   ".join(f"L{i + 1} {t:.1f}s" for i, t in enumerate(times)), font_md, (80, 255, 120), y + 50)
        centered(f"Total {sum(times):.1f} s    Energy {sum(energies):.2f} Wh", font_md, (200, 200, 255), y + 90)
        centered(f"Battery left {m.batt.soc * 100:.1f}%", font_md, (200, 200, 255), y + 125)
        centered("Press R to fly again", font, (200, 200, 200), y + 165)


def get_pilot_name():
    name = ""
    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_RETURN and name.strip():
                    return name.strip()
                if e.key == pygame.K_BACKSPACE:
                    name = name[:-1]
                elif e.unicode and e.unicode.isprintable() and len(name) < 20:
                    name += e.unicode
        screen.fill(BG)
        centered("UAV FLIGHT SIMULATOR", font_lg, (80, 200, 255), 140)
        centered("Enter your pilot name:", font_md, (180, 180, 200), 220)
        box = pygame.Rect(WIDTH // 2 - 160, 265, 320, 44)
        pygame.draw.rect(screen, (20, 20, 40), box, border_radius=6)
        pygame.draw.rect(screen, (80, 120, 200), box, 2, border_radius=6)
        cursor = "|" if (pygame.time.get_ticks() // 500) % 2 == 0 else ""
        screen.blit(font_md.render(name + cursor, True, (255, 255, 255)), (box.x + 12, box.y + 10))
        centered("Press ENTER to continue", font, (100, 100, 130), 325)
        pygame.display.flip()
        clock.tick(FPS)


def main():
    global muted
    pilot = get_pilot_name()
    level, mode = 1, MODES[0]
    m = Mission(level)
    briefing, detail, saved = True, False, False
    times, energies = [], []

    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if e.type != pygame.KEYDOWN:
                continue
            if e.key == pygame.K_m:
                muted = not muted
                if muted and audio_ok:
                    motor_ch.set_volume(0.0)
                    wind_ch.set_volume(0.0)
            elif e.key == pygame.K_h:
                detail = not detail
            elif e.key == pygame.K_TAB:
                mode = MODES[(MODES.index(mode) + 1) % len(MODES)]
                m.fc.set_mode(mode)
            elif e.key == pygame.K_SPACE:
                if briefing:
                    briefing = False
                elif m.complete and level < 3:
                    times.append(m.t)
                    energies.append(m.batt.energy_used_wh)
                    level += 1
                    m, briefing, saved = Mission(level), True, False
                    m.fc.set_mode(mode)
            elif e.key == pygame.K_r and not briefing:
                if m.complete and level == 3:
                    level, times, energies = 1, [], []
                m, briefing, saved = Mission(level), True, False
                m.fc.set_mode(mode)

        keys = pygame.key.get_pressed()
        stick = (keys[pygame.K_d] or keys[pygame.K_RIGHT]) - (keys[pygame.K_a] or keys[pygame.K_LEFT])
        throttle = (keys[pygame.K_w] or keys[pygame.K_UP]) - (keys[pygame.K_s] or keys[pygame.K_DOWN])
        if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
            # precision mode: smaller inputs for fine positioning
            stick, throttle = 0.4 * stick, 0.4 * throttle

        if not briefing:
            m.step(1.0 / FPS, stick, throttle)
            for ev in m.pop_events():
                play(ev)
            if m.crashed and not saved:
                play("crash")
                saved = True
            if m.complete and not saved:
                play("complete")
                save_log(m, pilot)
                saved = True
                if level == 3:
                    times.append(m.t)
                    energies.append(m.batt.energy_used_wh)

        update_loops(m, not briefing and not m.crashed and not m.complete)

        t = pygame.time.get_ticks() / 1000
        draw_world(m)
        draw_autopilot_path(m)
        if not briefing:
            draw_target_pointer(m)
        draw_objectives(m, t)
        draw_drone(m)
        if not briefing:
            draw_instruments(m)
            draw_hud(m, detail)
            if m.crashed:
                centered(f"CRASHED: {m.crash_reason}", font_lg, (255, 60, 60), HEIGHT // 2 - 20)
                centered("Press R to reset", font_md, (220, 160, 160), HEIGHT // 2 + 16)
            if m.complete:
                draw_result(m, times, energies)
        else:
            draw_briefing(level)
        pygame.display.flip()
        clock.tick(FPS)


if __name__ == "__main__":
    main()
