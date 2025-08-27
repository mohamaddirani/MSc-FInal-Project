# sim_app/main.py
import asyncio
import json
import math
import os
import time

from sim_app.sim_client import get_sim
from sim_app import shared
from sim_app.check_nearest_robot import excute
from sim_app.robot_controller import OmniRobotController
from sim_app.path_viz import live_plotter


ROBOT_IDS = ["Rob0", "Rob1", "Rob2"]
ID_TO_MODEL = {"Rob0": "Omnirob0", "Rob1": "Omnirob1", "Rob2": "Omnirob2"}

GOAL_FILE = "shared_goal.json"
CMD_FILE = "shared_cmd.json"
REPLY_FILE = "shared_reply.json"

# Mirror LLM location map (or move into shared if you prefer)
LOCATION_MAP = {
    "Meetings Table": (-1.500, -7.000),
    "Office Table": (-2.000, 2.000),
    "Cupboard A": (6.800, 1.000),
    "Cupboard B": (3.800, 1.000),
    "Cupboard C": (3.800, -3.000),
    "Cupboard D": (6.800, -3.000),
    "Meetings Rack": (-4.500, -4.000),
    "Office Rack": (-4.500, 0.000),
    "Rack A": (8.000, -5.000),
    "Rack B": (5.000, -5.000),
    "Rack C": (8.000, -9.000),
    "Rack D": (5.000, -9.000),
    "Rack 1": (3.000, -2.000),
    "Rack 2": (3.000, -6.000),
    "Rack 3": (3.000, -10.000),
}


# -----------------------------------------------------------------------------
# Controller & state utilities
# -----------------------------------------------------------------------------

async def init_all_controllers():
    """Initialize all robot controllers and connections."""
    ctrls = {}
    conns = {}  # rid -> (client, sim)
    for rid, model in ID_TO_MODEL.items():
        client, sim = await get_sim()
        ctrl = OmniRobotController()
        await ctrl.init_handles(sim, robot_name=model)
        pos = await ctrl.get_position()

        if shared.robot_start_positions[rid] == (0.0, 0.0) or shared.robot_start_positions[rid] is None:
            shared.robot_start_positions[rid] = (pos[0], pos[1])
        shared.robot_positions[rid] = (pos[0], pos[1])

        ctrls[rid] = ctrl
        conns[rid] = (client, sim)
    return ctrls, conns


async def refresh_all_robot_states(ctrls):
    """Refresh positions and orientations of all robots."""
    for rid, ctrl in ctrls.items():
        try:
            pos = await ctrl.get_position()
            ori = await ctrl.get_orientation()
            shared.robot_positions[rid] = (pos[0], pos[1])
            shared.robot_orientation[rid] = ori
        except Exception as e:
            print(f"⚠️ Failed refresh for {rid}: {e}")


# -----------------------------------------------------------------------------
# File IO helpers
# -----------------------------------------------------------------------------

def try_read_json(path):
    """Read and delete a JSON file if it exists; return dict or None."""
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r") as f:
            data = json.load(f)
        # best effort delete
        try:
            os.remove(path)
        except Exception:
            pass
        return data
    except Exception as e:
        print(f"⚠️ Failed to read {path}: {e}")
        return None


async def write_reply(obj):
    """Write a reply JSON file."""
    with open(REPLY_FILE, "w") as f:
        json.dump(obj, f)


# -----------------------------------------------------------------------------
# Command/goal handlers
# -----------------------------------------------------------------------------

async def handle_goal_file(conns, active_tasks):
    """Dispatch a goal from GOAL_FILE to a specific or nearest idle robot."""
    data = try_read_json(GOAL_FILE)
    if not data:
        return

    (key, goal) = list(data.items())[0]
    if key == "auto":
        candidates = []
        for rid in ROBOT_IDS:
            if shared.robot_status.get(rid) not in (None, "idle"):
                continue
            rx, ry = shared.robot_positions.get(rid, (0.0, 0.0))
            d = math.hypot(goal[0] - rx, goal[1] - ry)
            candidates.append((d, rid))
        if not candidates:
            print("⛔ No idle robots available for auto-dispatch.")
            return
        _, robot_name = min(candidates)
    else:
        robot_name = key

    shared.robot_goal[robot_name] = tuple(goal)
    shared.robot_status[robot_name] = "busy"
    print(f"✅ Goal set for {robot_name}: {goal}")

    if robot_name not in active_tasks:
        start_pos = shared.robot_positions.get(robot_name, (0.0, 0.0))
        start_ori = shared.robot_orientation.get(robot_name, (0.0, 0.0, 0.0))
        _client, sim_r = conns[robot_name]  # use this robot's sim
        task = asyncio.create_task(excute(sim_r, start_pos, start_ori, robot_name, goal))
        active_tasks[robot_name] = task


async def handle_cmd_file(conns, controllers, active_tasks):
    """Process commands from CMD_FILE (status/position/stop/go_home/nearest_to_location)."""
    cmd = try_read_json(CMD_FILE)
    if not cmd:
        return

    ctype = cmd.get("type")
    rid = cmd.get("robot_id")

    if ctype == "status":
        if rid:
            out = {rid: {"status": shared.robot_status.get(rid),
                         "goal": shared.robot_goal.get(rid)}}
        else:
            out = {r: {"status": shared.robot_status.get(r),
                       "goal": shared.robot_goal.get(r)} for r in ROBOT_IDS}
        await write_reply(out)

    elif ctype == "position":
        if rid:
            out = {rid: {"position": shared.robot_positions.get(rid, (0.0, 0.0))}}
        else:
            out = {r: {"position": shared.robot_positions.get(r, (0.0, 0.0))} for r in ROBOT_IDS}
        await write_reply(out)

    elif ctype == "stop" and rid in ROBOT_IDS:
        shared.robot_abort[rid] = True
        await write_reply({rid: "stopping"})

    elif ctype == "go_home" and rid in ROBOT_IDS:
        home = shared.robot_start_positions.get(rid)
        if not home:
            # fallback to current pose rather than (0,0)
            home = shared.robot_positions.get(rid)
        # ask current motion to stop; when it finishes, we’ll launch home as a fresh goal
        shared.robot_abort[rid] = True
        shared.pending_home[rid] = tuple(home)
        await write_reply({rid: {"queued_home_to": home}})

    elif ctype == "nearest_to_location":
        label = cmd.get("location_label")
        if label not in LOCATION_MAP:
            await write_reply({"error": f"unknown location '{label}'"})
            return
        lx, ly = LOCATION_MAP[label]
        best = None
        for r in ROBOT_IDS:
            px, py = shared.robot_positions.get(r, (0.0, 0.0))
            d = math.hypot(lx - px, ly - py)
            if (best is None) or (d < best[0]):
                best = (d, r)
        await write_reply({"label": label, "robot": best[1], "distance": best[0]})


# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------

async def run():
    """Main supervision loop: init, start sim, process goals/commands, schedule tasks."""
    if os.path.exists(REPLY_FILE):
        try:
            os.remove(REPLY_FILE)
        except Exception:
            pass

    shared.load_map()

    # build per-robot controllers and connections
    controllers, conns = await init_all_controllers()

    # start the simulation once on any sim (pick the first)
    any_sim = next(iter(conns.values()))[1]
    await any_sim.startSimulation()
    print("✅ Connected & simulation started")

    # live plots
    for rid in ROBOT_IDS:
        asyncio.create_task(live_plotter(rid, period_s=0.5))

    active_tasks = {}

    try:
        while True:
            await refresh_all_robot_states(controllers)

            # Move orders
            await handle_goal_file(conns, active_tasks)

            # Queries / stop / go_home / nearest
            await handle_cmd_file(conns, controllers, active_tasks)

            # reap finished tasks
            done = []
            for rid, t in active_tasks.items():
                if t.done():
                    try:
                        _ = t.result()  # "DONE" or "FAILED" etc.
                    except Exception:
                        pass
                    shared.robot_status[rid] = "idle"
                    shared.robot_goal[rid] = None
                    done.append(rid)
            for rid in done:
                del active_tasks[rid]

            # if a go_home was queued, launch it now as a brand-new goal
            for rid in ROBOT_IDS:
                home = shared.pending_home.get(rid)
                if home and rid not in active_tasks and shared.robot_status.get(rid) == "idle":
                    shared.pending_home[rid] = None
                    shared.robot_goal[rid] = tuple(home)
                    shared.robot_status[rid] = "busy"
                    start_pos = shared.robot_positions.get(rid, (0.0, 0.0))
                    start_ori = shared.robot_orientation.get(rid, (0.0, 0.0, 0.0))
                    _client, sim_r = conns[rid]
                    active_tasks[rid] = asyncio.create_task(
                        excute(sim_r, start_pos, start_ori, rid, home)
                    )

            await asyncio.sleep(0.05)

    except KeyboardInterrupt:
        print("🛑 Stopped by user.")
        # close all clients cleanly
        for client, _ in conns.values():
            await client.__aexit__(None, None, None)


if __name__ == "__main__":
    import sys
    if sys.platform == "win32" and sys.version_info >= (3, 8):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(run())
