#!/usr/bin/env python3
"""
make_campus_world.py - turn a top-down OpenStreetMap export (PDF or PNG) into a Gazebo world for PX4.

What it builds
  * campus.png          the map, used as the ground texture
  * <name>.sdf          a copy of PX4's default.sdf with the map as ground, 3D buildings extruded from the
                        building outlines found in the map (OSM building colour #d9d0c9), and real GPS coordinates
  * layout.json         where each drone spawns, its waypoints (landmarks), altitude, speed and geofence

Usage (Ubuntu, from ~/drone_sim/campus):
  sudo apt install -y poppler-utils          # only needed for PDF input
  python3 make_campus_world.py --map kent_map.pdf --width-m 1170
  export PX4_GZ_WORLD=kent_campus
  cd .. && ./px4_launch_sims.sh 2

IMPORTANT: --width-m is how many metres the map is wide. The default (1170) is an ESTIMATE for the supplied
Kent map. For an exact value use the corner coordinates from OpenStreetMap (Share > Image) or a known distance.
"""
import argparse, glob, json, math, os, re, shutil, subprocess, sys
import numpy as np
import cv2
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
BUILDING_RGB = (217, 208, 201)       # OSM standard building fill

# Landmarks as fractions of the image (x from left, y from top), read off the supplied Kent map.
LANDMARKS = {
    "library":        (0.762, 0.489),
    "keynes_college": (0.559, 0.755),
    "sports_centre":  (0.555, 0.558),
    "rutherford":     (0.881, 0.537),
    "eliot_college":  (0.759, 0.670),
    "cornwallis":     (0.870, 0.346),
    "west_pitches":   (0.146, 0.351),
}
# Open ground to take off from (checked against the building outlines at run time)
SPAWNS = {"drone0": (0.812, 0.777), "drone1": (0.590, 0.883)}
MISSIONS = {"drone0": ["library", "cornwallis"], "drone1": ["sports_centre", "eliot_college", "rutherford"]}


def render_map(path, dpi):
    if path.lower().endswith(".pdf"):
        out = os.path.join(HERE, "_render")
        subprocess.check_call(["pdftoppm", "-r", str(dpi), "-png", "-singlefile", path, out])
        img = Image.open(out + ".png").convert("RGB")
        os.remove(out + ".png")
        return img
    return Image.open(path).convert("RGB")


def find_buildings(img, m_per_px, min_area_m2, tol):
    a = np.array(img).astype(int)
    mask = (np.abs(a - np.array(BUILDING_RGB)).max(axis=2) <= tol).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        area_m2 = cv2.contourArea(c) * m_per_px ** 2
        if area_m2 < min_area_m2 or area_m2 > 60000:
            continue
        ap = cv2.approxPolyDP(c, 1.6, True)[:, 0, :]
        if len(ap) < 3:
            continue
        out.append(ap)
    return out, mask


def px_to_m(p, W, H, m_per_px):
    return ((p[0] - W / 2) * m_per_px, -(p[1] - H / 2) * m_per_px)       # x east, y north, origin = map centre


def inside(poly_px, pt):
    return cv2.pointPolygonTest(poly_px.astype(np.float32).reshape(-1, 1, 2), (float(pt[0]), float(pt[1])), False) >= 0


def build_sdf(default_sdf_text, name, lat, lon, ground_png, W_m, H_m, polys_m, height):
    t = default_sdf_text
    t, n = re.subn(r"(<world\s+name\s*=\s*)(['\"])[^'\"]*\2", r"\1'%s'" % name, t, count=1)
    if n != 1:
        sys.exit("could not find <world name=...> in default.sdf")
    t = re.sub(r"<latitude_deg>[^<]*</latitude_deg>", f"<latitude_deg>{lat:.8f}</latitude_deg>", t)
    t = re.sub(r"<longitude_deg>[^<]*</longitude_deg>", f"<longitude_deg>{lon:.8f}</longitude_deg>", t)
    ground = f"""
    <model name='campus_ground'>
      <static>true</static>
      <pose>0 0 0.02 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{W_m:.2f} {H_m:.2f}</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
            <pbr><metal><albedo_map>{ground_png}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
"""
    links = []
    for i, poly in enumerate(polys_m):
        pts = "".join(f"<point>{x:.2f} {y:.2f}</point>" for x, y in poly)
        geom = f"<geometry><polyline>{pts}<height>{height:.1f}</height></polyline></geometry>"
        links.append(f"""      <link name='b{i}'>
        <collision name='c'>{geom}</collision>
        <visual name='v'>{geom}<material><ambient>0.80 0.76 0.72 1</ambient><diffuse>0.80 0.76 0.72 1</diffuse><specular>0 0 0 1</specular></material></visual>
      </link>""")
    bl = "\n    <model name='campus_buildings'>\n      <static>true</static>\n" + "\n".join(links) + "\n    </model>\n"
    i = t.rfind("</world>")
    if i < 0:
        sys.exit("no </world> in default.sdf")
    return t[:i] + ground + bl + t[i:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=os.path.join(HERE, "kent_map.pdf"))
    ap.add_argument("--width-m", type=float, default=1170.0, help="real width of the map in metres (default: estimate)")
    ap.add_argument("--lat", type=float, default=51.2975, help="latitude of the map centre (approximate default)")
    ap.add_argument("--lon", type=float, default=1.0690, help="longitude of the map centre (approximate default)")
    ap.add_argument("--name", default="kent_campus")
    ap.add_argument("--height", type=float, default=14.0, help="building height in metres (OSM has none; this is a guess)")
    ap.add_argument("--min-area", type=float, default=60.0, help="ignore buildings smaller than this (m2)")
    ap.add_argument("--tol", type=int, default=5, help="colour tolerance when finding buildings")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--default-sdf", help="path to PX4's default.sdf (auto-detected under ~/PX4-Autopilot)")
    ap.add_argument("--no-install", action="store_true", help="write the SDF here instead of into PX4's worlds folder")
    ap.add_argument("--flip-y", action="store_true", help="flip the ground picture vertically (use if it does not match the buildings)")
    ap.add_argument("--flip-x", action="store_true")
    ap.add_argument("--alt", type=float, default=30.0, help="flight altitude in metres (must clear the buildings)")
    ap.add_argument("--speed", type=float, default=8.0)
    a = ap.parse_args()

    img = render_map(a.map, a.dpi)
    W, H = img.size
    m_per_px = a.width_m / W
    W_m, H_m = a.width_m, H * m_per_px
    print(f"map {W}x{H} px = {W_m:.0f} x {H_m:.0f} m  ({m_per_px:.3f} m/px)")

    polys_px, mask = find_buildings(img, m_per_px, a.min_area, a.tol)
    print(f"found {len(polys_px)} buildings (>= {a.min_area:.0f} m2)")
    tex = img
    if a.flip_y:
        tex = tex.transpose(Image.FLIP_TOP_BOTTOM)
    if a.flip_x:
        tex = tex.transpose(Image.FLIP_LEFT_RIGHT)
    ground_png = os.path.join(HERE, "campus.png")
    tex.save(ground_png)

    polys_m = []
    for p in polys_px:
        q = [px_to_m(pt, W, H, m_per_px) for pt in p]
        # counter-clockwise, as Gazebo expects
        area = sum(q[i][0] * q[(i + 1) % len(q)][1] - q[(i + 1) % len(q)][0] * q[i][1] for i in range(len(q)))
        polys_m.append(q if area > 0 else q[::-1])

    # layout: spawn, waypoints (world metres), checked against buildings
    def to_world(frac):
        return px_to_m((frac[0] * W, frac[1] * H), W, H, m_per_px)
    layout = {"map": os.path.basename(a.map), "width_m": W_m, "height_m": H_m, "alt_m": a.alt, "speed_m_s": a.speed,
              "fence_hor_m": 900, "fence_ver_m": a.alt + 40, "timeout_s": 420, "world": a.name, "drones": []}
    for i, key in enumerate(("drone0", "drone1")):
        sp = SPAWNS[key]
        blocked = any(inside(p, (sp[0] * W, sp[1] * H)) for p in polys_px)
        if blocked:
            print(f"WARNING: spawn point of {key} is inside a building outline; pick another in SPAWNS")
        layout["drones"].append({"id": i, "spawn": [round(v, 1) for v in to_world(sp)],
                                 "waypoints": [[round(v, 1) for v in to_world(LANDMARKS[n])] for n in MISSIONS[key]],
                                 "waypoint_names": MISSIONS[key]})
    layout["landmarks"] = {k: [round(v, 1) for v in to_world(f)] for k, f in LANDMARKS.items()}
    with open(os.path.join(HERE, "layout.json"), "w") as fh:
        json.dump(layout, fh, indent=2)

    # preview picture (buildings, spawns, routes drawn over the map)
    prev = np.array(img).copy()
    cv2.polylines(prev, [p.reshape(-1, 1, 2) for p in polys_px], True, (200, 30, 30), 2)
    for d, col in zip(layout["drones"], ((0, 90, 220), (140, 40, 200))):
        pts = [(d["spawn"][0], d["spawn"][1])] + [tuple(w) for w in d["waypoints"]] + [(d["spawn"][0], d["spawn"][1])]
        pp = [(int(x / m_per_px + W / 2), int(-y / m_per_px + H / 2)) for x, y in pts]
        cv2.polylines(prev, [np.array(pp, np.int32).reshape(-1, 1, 2)], False, col, 5)
        cv2.circle(prev, pp[0], 14, col, -1)
    Image.fromarray(prev).resize((W // 2, H // 2)).save(os.path.join(HERE, "preview.png"))

    # world file
    sdf_in = a.default_sdf
    if not sdf_in:
        c = glob.glob(os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf"))
        if not c:
            c = glob.glob(os.path.expanduser("~/PX4-Autopilot/**/worlds/default.sdf"), recursive=True)
        if not c:
            sys.exit("could not find PX4's default.sdf; pass --default-sdf")
        sdf_in = c[0]
    text = open(sdf_in).read()
    out = build_sdf(text, a.name, a.lat, a.lon, ground_png, W_m, H_m, polys_m, a.height)
    dest = os.path.join(HERE, a.name + ".sdf") if a.no_install else os.path.join(os.path.dirname(sdf_in), a.name + ".sdf")
    open(dest, "w").write(out)
    print("world written to", dest)
    print("next:  export PX4_GZ_WORLD=%s   then   cd .. && ./px4_launch_sims.sh 2" % a.name)


if __name__ == "__main__":
    main()
