import numpy as np
import matplotlib.pyplot as plt

def _orient(arr: np.ndarray, flip_ud=False, flip_lr=False, rot_cw=0):
    """Flip/rotate helper (CW rotations)."""
    if flip_ud:
        arr = np.flipud(arr)
    if flip_lr:
        arr = np.fliplr(arr)
    if rot_cw % 4:
        arr = np.rot90(arr, k=-(rot_cw % 4))  # np.rot90 is CCW; use negative for CW
    return arr

def _to_bw(arr: np.ndarray) -> np.ndarray:
    """Binarize: free=0 (white), obstacle/cost>0=1 (black)."""
    return (arr > 0).astype(np.uint8)

def render_map_npz(npz_path: str,
                   out_prefix: str = "map",
                   dpi: int = 180,
                   flip_ud: bool = False,
                   flip_lr: bool = True,
                   rot_cw: int = 0,
                   binarize_costmap: bool = True):
    """
    Load a map .npz (expects 'occupancy', 'costmap', optional 'res'),
    apply orientation, and save JPGs with white background & black obstacles.
    """
    data = np.load(npz_path)
    occ  = _to_bw(data["occupancy"])
    cost_raw = data["costmap"]
    cost = _to_bw(cost_raw) if binarize_costmap else np.clip(cost_raw, 0.0, 1.0)
    res  = float(data["res"]) if "res" in data else 0.36

    outs = {}
    for name, arr in [("occupancy", occ), ("costmap", cost)]:
        arr2 = _orient(arr, flip_ud=flip_ud, flip_lr=flip_lr, rot_cw=rot_cw)

        # Clean, axis-free render on white background
        fig = plt.figure(figsize=(6, 6), facecolor="white")
        ax = fig.add_axes([0, 0, 1, 1])   # fill canvas, no margins
        ax.set_axis_off()
        ax.imshow(arr2, cmap="binary", vmin=0, vmax=1, interpolation="nearest")  # 0=white, 1=black

        out_path = f"{out_prefix}_{name}.jpg"
        fig.savefig(out_path, dpi=dpi, facecolor="white")
        plt.close(fig)
        outs[name] = out_path

    print(f"Saved:\n  {outs['occupancy']}\n  {outs['costmap']}")
    return outs

# Example: mirror horizontally, keep rotation as-is
# render_map_npz("map_memory.npz", out_prefix="map_fixed", flip_lr=True)

render_map_npz("sim_app/map_memory.npz")
