"""Local web dashboard for browsing flight reports.

    python dashboard.py      then open http://localhost:5000

Charts are generated on demand with analysis.flight_report.
Meant for local use; it binds to 127.0.0.1 by default.
"""

import glob
import html
import os
import threading
from urllib.parse import quote

from flask import Flask, abort, make_response, redirect, request, send_from_directory, url_for

from analysis import plotstyle
from analysis.flight_report import load_log, by_level, summarize, process_session

app = Flask(__name__)
REPORTS_DIR = os.path.abspath("reports")
_lock = threading.Lock()
esc = html.escape


def session_path(name):
    """Resolve a session folder name, refusing anything outside reports/."""
    path = os.path.abspath(os.path.join(REPORTS_DIR, name))
    if os.path.dirname(path) != REPORTS_DIR or not os.path.isdir(path):
        abort(404)
    return path


def ensure_charts(sd, force=False):
    log = os.path.join(sd, "flight_log.csv")
    if not os.path.isfile(log):
        return
    pngs = glob.glob(os.path.join(sd, "level*_report.png"))
    stale = pngs and min(os.path.getmtime(p) for p in pngs) < os.path.getmtime(log)
    if pngs and not stale and not force:
        return
    with _lock:
        plotstyle.apply()
        process_session(sd)


def sessions():
    out = []
    for sd in sorted(glob.glob(os.path.join(REPORTS_DIR, "*")), reverse=True):
        if not os.path.isdir(sd):
            continue
        pf = os.path.join(sd, "pilot.txt")
        pilot = open(pf, encoding="utf-8").read().strip() if os.path.isfile(pf) else "unknown"
        levels = {}
        try:
            for lvl, rows in by_level(load_log(os.path.join(sd, "flight_log.csv"))).items():
                levels[lvl] = summarize(rows)
        except (OSError, ValueError, KeyError):
            pass
        ensure_charts(sd)
        out.append({"name": os.path.basename(sd), "pilot": pilot, "levels": levels,
                    "pngs": sorted(os.path.basename(p) for p in glob.glob(os.path.join(sd, "level*_report.png")))})
    return out


CSS = """
:root{--bg:#0f0f19;--panel:#1a1a2e;--border:#2a2a4a;--text:#ccd;--accent:#4fc3f7}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:ui-monospace,Consolas,monospace}
a{color:var(--accent);text-decoration:none}a:hover{text-decoration:underline}
header{background:var(--panel);border-bottom:1px solid var(--border);padding:14px 28px;display:flex;gap:24px;align-items:center}
header h1{font-size:1.05rem;color:var(--accent);letter-spacing:2px}
header nav a{margin-right:16px;font-size:.85rem;color:var(--text)}
.wrap{max-width:1100px;margin:0 auto;padding:28px 18px}
h2{font-size:1rem;color:var(--accent);margin-bottom:16px;letter-spacing:1px}
.grid{display:flex;flex-wrap:wrap;gap:14px}
.card{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:14px 18px;margin-bottom:14px}
.card:hover{border-color:var(--accent)}
.meta{font-size:.78rem;color:#889;margin-top:4px}
.stats{display:flex;flex-wrap:wrap;gap:8px 18px;font-size:.8rem;margin:8px 0}
.stats b{color:#fff}
.charts img{width:100%;border-radius:6px;border:1px solid var(--border);margin-top:10px;cursor:zoom-in}
.lb{display:none;position:fixed;inset:0;background:rgba(0,0,0,.92);align-items:center;justify-content:center;cursor:zoom-out}
.lb.open{display:flex}.lb img{max-width:96vw;max-height:94vh}
input,button{font:inherit}
.name{background:var(--panel);border:2px solid var(--border);border-radius:6px;padding:10px 14px;color:#fff;width:280px}
.btn{background:var(--accent);color:#000;border:0;border-radius:6px;padding:10px 22px;font-weight:bold;cursor:pointer}
.center{min-height:70vh;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:14px;text-align:center}
"""


def page(body, pilot=""):
    me = f'<a href="{url_for("my_flights")}">My flights</a><a href="{url_for("logout")}">Change name</a>' if pilot else ""
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>UAV Flight Reports</title>
<style>{CSS}</style></head><body>
<header><h1>UAV FLIGHT REPORTS</h1><nav>{me}<a href="{url_for('all_pilots')}">All pilots</a></nav></header>
<div id="lb" class="lb" onclick="this.classList.remove('open')"><img id="lbi" alt=""></div>
<div class="wrap">{body}</div>
<script>
document.addEventListener('click',e=>{{const i=e.target.closest('.charts img');if(!i)return;
document.getElementById('lbi').src=i.src;document.getElementById('lb').classList.add('open');}});
document.addEventListener('keydown',e=>{{if(e.key==='Escape')document.getElementById('lb').classList.remove('open');}});
</script></body></html>"""


def session_cards(items):
    if not items:
        return "<p class='meta'>No flights yet. Fly the simulator first.</p>"
    out = ""
    for s in items:
        stats = "".join(
            f"<span>L{lvl}: <b>{v['duration_s']:.1f} s</b>, {v['energy_Wh']:.2f} Wh, "
            f"max {v['max_speed_mps']:.1f} m/s</span>" for lvl, v in sorted(s["levels"].items()))
        imgs = "".join(f'<img loading="lazy" src="{url_for("image", session=s["name"], filename=p)}" alt="{esc(p)}">'
                       for p in s["pngs"])
        out += f"""<div class="card"><b>{esc(s['name'])}</b>
<span class="meta">&nbsp; pilot {esc(s['pilot'])} &nbsp;
<a href="{url_for('regen', session=s['name'])}">regenerate charts</a></span>
<div class="stats">{stats or "<span class='meta'>No usable log in this session.</span>"}</div>
<div class="charts">{imgs}</div></div>"""
    return out


@app.route("/", methods=["GET", "POST"])
def landing():
    if request.method == "POST":
        name = request.form.get("name", "").strip()[:30]
        if name:
            resp = make_response(redirect(url_for("my_flights")))
            resp.set_cookie("pilot", name, max_age=30 * 24 * 3600, samesite="Lax")
            return resp
    return page("""<div class="center"><h2>Enter your pilot name</h2>
<form method="post" class="center" style="min-height:0">
<input class="name" name="name" maxlength="30" autofocus placeholder="pilot name">
<button class="btn">View my flights</button></form></div>""")


@app.route("/me")
def my_flights():
    pilot = request.cookies.get("pilot", "")
    if not pilot:
        return redirect(url_for("landing"))
    mine = [s for s in sessions() if s["pilot"] == pilot]
    return page(f"<h2>MY FLIGHTS ({esc(pilot)})</h2>" + session_cards(mine), pilot)


@app.route("/pilot/<name>")
def pilot_page(name):
    items = [s for s in sessions() if s["pilot"] == name]
    return page(f"<h2>PILOT: {esc(name)}</h2>" + session_cards(items), request.cookies.get("pilot", ""))


@app.route("/all")
def all_pilots():
    pilots = {}
    for s in sessions():
        p = pilots.setdefault(s["pilot"], {"n": 0, "lvl": 0, "best": None})
        p["n"] += 1
        if s["levels"]:
            p["lvl"] = max(p["lvl"], max(s["levels"]))
            if len(s["levels"]) == 3:
                total = sum(v["duration_s"] for v in s["levels"].values())
                p["best"] = total if p["best"] is None else min(p["best"], total)
    cards = "".join(
        f'<a class="card" href="{url_for("pilot_page", name=n)}"><b>{esc(n)}</b>'
        f'<div class="meta">{d["n"]} session(s), highest level {d["lvl"]}</div>'
        f'<div class="meta">best full mission: {"%.1f s" % d["best"] if d["best"] else "-"}</div></a>'
        for n, d in pilots.items()) or "<p class='meta'>No sessions yet.</p>"
    return page(f"<h2>ALL PILOTS</h2><div class='grid'>{cards}</div>", request.cookies.get("pilot", ""))


@app.route("/regen/<session>")
def regen(session):
    ensure_charts(session_path(session), force=True)
    return redirect(request.referrer or url_for("all_pilots"))


@app.route("/logout")
def logout():
    resp = make_response(redirect(url_for("landing")))
    resp.delete_cookie("pilot")
    return resp


@app.route("/img/<session>/<filename>")
def image(session, filename):
    return send_from_directory(session_path(session), filename)


if __name__ == "__main__":
    print("UAV flight reports on http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=False)
