#!/usr/bin/env python3
import os, random, subprocess, time, shlex, math

# -------------------------------
# CONFIGURATION
# -------------------------------
WORLD_NAME = "earth"          # world to spawn into
X_MIN, X_MAX = -499.0, -3.0   # map bounds
Y_MIN, Y_MAX = -7975.0, -7495.0
Z_MAX  = 835.0                # tallest point on map
Z_HIGH = Z_MAX + 10.0         # spawn height above terrain
SETTLE_SEC = 2.0
MIN_SPACING = 6.0

# Models available
ANIMALS = {
    "bear": "model://bear",
    "boar": "model://boar",
}

# Number of each species to spawn
COUNT_PER_SPECIES = 6

# -------------------------------
# FUNCTIONS
# -------------------------------
def sample_xy(used, tries=100):
    for _ in range(tries):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        if all(((x-ux)**2 + (y-uy)**2) ** 0.5 >= MIN_SPACING for ux, uy in used):
            return x, y
    return random.uniform(X_MIN, X_MAX), random.uniform(Y_MIN, Y_MAX)

def ign_spawn(name, uri, x, y, z, yaw):
    """Spawn using Ignition /world/<name>/create service."""
    half = yaw * 0.5
    qx, qy = 0.0, 0.0
    qz = math.sin(half)
    qw = math.cos(half)

    req = (
        f'sdf_filename:"{uri}", name:"{name}", '
        f'pose:{{ position:{{x:{x}, y:{y}, z:{z}}}, '
        f'orientation:{{ x:{qx}, y:{qy}, z:{qz}, w:{qw} }} }}'
    )
    cmd = [
        'ign', 'service',
        '-s', f'/world/{WORLD_NAME}/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '300',
        '--req', req
    ]
    print("[INFO]", shlex.join(cmd))
    subprocess.run(cmd, check=True)

def check_model_discovery():
    roots = (os.environ.get('IGN_GAZEBO_RESOURCE_PATH','').split(':') +
             os.environ.get('GZ_SIM_RESOURCE_PATH','').split(':'))
    roots = [r for r in roots if r]
    print("[INFO] Model search roots:")
    for r in roots:
        print("   ", r)
    found = False
    for root in roots:
        test_path = os.path.join(root, 'models', 'bear', 'bear.sdf')
        if os.path.isfile(test_path):
            print(f"[OK] Found bear.sdf at {test_path}")
            found = True
    if not found:
        print("[WARN] Did not find bear.sdf in any search root!")

def main():
    print(f"[INFO] GZ_SIM_RESOURCE_PATH={os.environ.get('GZ_SIM_RESOURCE_PATH','(unset)')}")
    print(f"[INFO] IGN_GAZEBO_RESOURCE_PATH={os.environ.get('IGN_GAZEBO_RESOURCE_PATH','(unset)')}")
    check_model_discovery()

    used = []
    for species, uri in ANIMALS.items():
        for i in range(COUNT_PER_SPECIES):
            x, y = sample_xy(used)
            yaw = random.uniform(0.0, 2 * math.pi)
            print(f"[INFO] Spawning {species}_{i} at ({x:.2f}, {y:.2f}, {Z_HIGH:.2f}) yaw={yaw:.2f}")
            ign_spawn(f"{species}_{i}", uri, x, y, Z_HIGH, yaw)
            used.append((x, y))
            time.sleep(0.2)

    print(f"[INFO] Letting animals settle for {SETTLE_SEC:.1f}s...")
    time.sleep(SETTLE_SEC)
    print("[INFO] Done.")

if __name__ == "__main__":
    random.seed()
    main()
