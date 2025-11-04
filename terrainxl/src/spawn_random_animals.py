#!/usr/bin/env python3
import random, subprocess, time, os

# ---- WORLD BOUNDS (meters) ----
X_MIN, X_MAX = -499.0, -3     
Y_MIN, Y_MAX =  -7975.0, -7495.0

# ---- HEIGHT SETTINGS ----
Z_MAX = 835.0
Z_HIGH = Z_MAX + 10.0               # spawn above the tallest peak
SETTLE_SEC = 2.0                    # time to fall & settle

# ---- MODELS ----
ANIMAL_MODELS = {
    "bear": "model://bear",
    "boar": "model://boar",
}

# ---- SPAWN CONTROL ----
MARGIN_MIN_DIST = 6.0               # min spacing between spawns (m)
WORLD_FILE = None                   # optional: set if you want to launch here
WORLD_NAME = "singletile"           # set to your world name in Gazebo

random.seed()  # change to a number for reproducibility

def sample_xy(existing, tries=100):
    """Sample an (x,y) with min spacing from existing points."""
    for _ in range(tries):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        if all(((x-ex)**2 + (y-ey)**2) ** 0.5 >= MARGIN_MIN_DIST for ex, ey in existing):
            return x, y
    # fallback: no spacing guarantee
    return random.uniform(X_MIN, X_MAX), random.uniform(Y_MIN, Y_MAX)

def spawn_one(name, sdf_path, idx, used_xy):
    x, y = sample_xy(used_xy)
    yaw = random.uniform(0.0, 6.28318)

    print(f"[INFO] Spawning {name}_{idx} at ({x:.2f}, {y:.2f}, {Z_HIGH:.2f}) yaw={yaw:.2f}")
    cmd = [
        "gz", "sim", "-r",
        "--spawn-file", sdf_path,
        "--spawn-name", f"{name}_{idx}",
        "--spawn-pose", f"{x} {y} {Z_HIGH} 0 0 {yaw}"
    ]
    subprocess.run(cmd, check=True)
    used_xy.append((x, y))

def main():
    # Optional: launch world if you want (otherwise launch separately)
    if WORLD_FILE:
        subprocess.Popen(["gz", "sim", "-v", "4", WORLD_FILE])
        time.sleep(4)  # give Gazebo a moment to start

    used_xy = []
    # one of each species
    for name, sdf in ANIMAL_MODELS.items():
        spawn_one(name, sdf, 0, used_xy)
        time.sleep(0.3)

    # let them drop to the ground
    print(f"[INFO] Letting animals settle for {SETTLE_SEC:.1f}s...")
    time.sleep(SETTLE_SEC)

    # (Optional) Freeze them: simplest is to leave as-is.
    # If you really want to freeze, you can set gravity=false on main link via a small helper plugin or service.
    print("[INFO] Done.")

if __name__ == "__main__":
    if "GZ_SIM_RESOURCE_PATH" not in os.environ:
        print("[WARN] GZ_SIM_RESOURCE_PATH not set; textures may not resolve.")
    main()
