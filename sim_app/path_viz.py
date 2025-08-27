# sim_app/path_viz.py
import os, math, asyncio
from typing import List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt

from sim_app import shared

RUNS_DIR = os.path.join(os.path.dirname(__file__), "runs")
os.makedirs(RUNS_DIR, exist_ok=True)

def _to_np(path: List[Tuple[float, float]]) -> np.ndarray:
    return np.asarray(path, dtype=float) if path else np.zeros((0, 2), dtype=float)

def _nearest_rmse(planned_xy: np.ndarray, executed_xy: np.ndarray) -> float:
    if len(planned_xy) == 0 or len(executed_xy) == 0:
        return float("nan")
    d2_min = []
    for p in executed_xy:
        d2 = np.sum((planned_xy - p)**2, axis=1)
        d2_min.append(np.min(d2))
    return float(np.sqrt(np.mean(d2_min)))

def _extent_from_grid(grid: np.ndarray, res: float):
    h, w = grid.shape
    half_w_m = (w // 2) * res
    half_h_m = (h // 2) * res
    return [-half_w_m, half_w_m, -half_h_m, half_h_m]

def _load_map_masks():
    try:
        occ  = (shared.global_occupancy > 0).astype(np.uint8)
        cost = (shared.global_costmap   > 0).astype(np.uint8)
        if occ.size == 0 or cost.size == 0:
            return None, None
        return occ, cost
    except Exception:
        return None, None

def plot_paths_once(robot: str, save_name: Optional[str] = None) -> dict:
    """
    Overwrite-only save. If save_name is None, saves to {robot}_paths_latest.png
    inside sim_app/runs/.
    """
    # 1) choose planned path: A* (if any) or the waypoint list
    planned_astar = _to_np(shared.latest_astar_path_by_robot.get(robot, []))
    #planned_wp    = _to_np(shared.planned_path_by_robot.get(robot, []))
    planned       = planned_astar #if len(planned_astar) else planned_wp

    # 2) executed (actual)
    executed      = _to_np(shared.executed_path_by_robot.get(robot, []))

    # 3) map backdrop
    occ, cost = _load_map_masks()
    res = shared.MAP_RESOLUTION
    extent = _extent_from_grid(occ, res) if occ is not None else None

    fig = plt.figure(figsize=(6.5, 6.5), facecolor="white")
    ax = fig.add_axes([0.08, 0.06, 0.90, 0.90])
    ax.set_title(f"{robot}: Planned vs Executed", fontsize=12)

    if occ is not None:
        ax.imshow(occ, cmap="binary", vmin=0, vmax=1, interpolation="nearest",
                  extent=extent, origin="lower", alpha=0.25)
    if cost is not None:
        ax.imshow(cost, cmap="binary", vmin=0, vmax=1, interpolation="nearest",
                  extent=extent, origin="lower", alpha=0.10)

    if len(planned):
        ax.plot(planned[:,0], planned[:,1], lw=2, linestyle="--", label="Planned (A*)")
        ax.scatter(planned[0,0], planned[0,1], s=40, marker="o", label="Planned start")
        ax.scatter(planned[-1,0], planned[-1,1], s=90, marker="*", label="Planned goal")

    if len(executed):
        ax.plot(executed[:,0], executed[:,1], lw=2, label="Executed")

    rmse = _nearest_rmse(planned, executed)
    if not math.isnan(rmse):
        ax.text(0.02, 0.98, f"RMSE: {rmse:.3f} m", transform=ax.transAxes, va="top",
                ha="left", bbox=dict(facecolor="white", alpha=0.7, edgecolor="none"))
        if not hasattr(shared, "path_stats"):
            shared.path_stats = {}
        shared.path_stats[robot] = {
            "rmse_m": rmse,
            "planned_pts": int(len(planned)),
            "executed_pts": int(len(executed)),
        }

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.2)
    ax.axis("equal")
    ax.legend(loc="lower right", framealpha=0.85)

    # ---------- SAVE (overwrite-only) ----------
    if save_name is None:
        save_name = f"{robot}_paths_latest.png"
    out_path = os.path.join(RUNS_DIR, save_name)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return {"latest": out_path, "rmse_m": rmse}

async def live_plotter(robot: str, period_s: float = 0.5):
    while True:
        try:
            plot_paths_once(robot)  # overwrite the same file each tick
        except Exception as e:
            print(f"[path_viz:{robot}] plot error: {e}")
        await asyncio.sleep(period_s)
