"""
STL'den 12x12 motor grid'ine interpolasyon.
Web_UI/backend/stl_interpolator.py ile ayni mantik (yeniden kullanim).
"""
import numpy as np
from scipy.interpolate import LinearNDInterpolator
import trimesh


def interpolate_stl_to_grid(stl_file, rows=12, cols=12, center_mm=300, max_mm=600):
    """STL dosyasini rows x cols motor grid'ine interpolasyon yapar.
    Donus: (grid_int_1d, mesh) -- grid satir-satir 1D int dizisi (mm)."""
    mesh = trimesh.load(stl_file)
    points = np.asarray(mesh.vertices)

    minX, minY, _ = points.min(axis=0)
    maxX, maxY, maxZ = points.max(axis=0)

    F = LinearNDInterpolator(points[:, :2], points[:, 2], fill_value=maxZ)

    X, Y = np.meshgrid(
        np.linspace(minX, maxX, cols),
        np.linspace(minY, maxY, rows),
    )
    Z = F(X.ravel(), Y.ravel())
    Z[np.isnan(Z)] = maxZ

    # merkez center_mm olacak sekilde offset
    offset = center_mm - float(np.mean(Z))
    Z = np.clip(Z + offset, 0, max_mm)

    return Z.astype(int), mesh


def grid_stats(grid_1d):
    g = np.asarray(grid_1d, dtype=float)
    return {
        "min": int(g.min()),
        "max": int(g.max()),
        "mean": int(g.mean()),
        "std": int(g.std()),
    }
