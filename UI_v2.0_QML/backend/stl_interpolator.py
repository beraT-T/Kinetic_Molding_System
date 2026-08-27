"""
STL'den 12x12 motor grid'ine interpolasyon.
Web_UI/backend/stl_interpolator.py ile ayni mantik (yeniden kullanim).
"""
import numpy as np
from scipy.interpolate import LinearNDInterpolator
import trimesh

# GUVENLIK MARJI (donanimda dogrulandi):
# Aktuator stroku 0-600 mm. Hedef tam sinira (0 veya 600) verilirse aktuator
# fiziksel dayanmaya oturur, encoder durur ve firmware v5.1 stall korumasi
# (1500 ms'de <10 puls) bunu FAULT sanar -> sahte ariza. Bu yuzden interpolasyon
# sonucu tam stroka degil, marjli araliga kirpilir.
STROKE_MIN_MM = 0
STROKE_MAX_MM = 600
SAFE_MARGIN_MM = 10                              # her iki uctan birakilan pay
SAFE_MIN_MM = STROKE_MIN_MM + SAFE_MARGIN_MM     # 10
SAFE_MAX_MM = STROKE_MAX_MM - SAFE_MARGIN_MM     # 590


def interpolate_stl_to_grid(stl_file, rows=12, cols=12, center_mm=300,
                            min_mm=SAFE_MIN_MM, max_mm=SAFE_MAX_MM):
    """STL dosyasini rows x cols motor grid'ine interpolasyon yapar.
    Sonuc guvenlik marjli araliga (varsayilan 10-590 mm) kirpilir.
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

    # merkez center_mm olacak sekilde offset, sonra guvenli araliga kirp
    offset = center_mm - float(np.mean(Z))
    Z = np.clip(Z + offset, min_mm, max_mm)

    return Z.astype(int), mesh


def grid_stats(grid_1d):
    g = np.asarray(grid_1d, dtype=float)
    return {
        "min": int(g.min()),
        "max": int(g.max()),
        "mean": int(g.mean()),
        "std": int(g.std()),
    }
