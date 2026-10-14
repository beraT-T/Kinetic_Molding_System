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
    grid, mesh, _info = interpolate_stl_to_grid_info(stl_file, rows, cols, center_mm,
                                                     min_mm, max_mm)
    return grid, mesh


def interpolate_stl_to_grid_info(stl_file, rows=12, cols=12, center_mm=300,
                                 min_mm=SAFE_MIN_MM, max_mm=SAFE_MAX_MM):
    """interpolate_stl_to_grid ile AYNI algoritma; ek olarak operatore gosterilecek
    donusum bilgisini dondurur (v2.2 seffaflik - algoritma degismedi):
      filled        : model izdusumunun (konveks zarf) disinda kalip maxZ ile
                      DOLDURULAN hucre sayisi
      clipped_low   : 10 mm alt sinira kirpilan hucre sayisi
      clipped_high  : 590 mm ust sinira kirpilan hucre sayisi
      offset_mm     : ortalama center_mm olsun diye eklenen kayma
      max_neighbor  : komsu (yatay/dikey) iki hucre arasindaki en buyuk fark (mm)
                      -- yalniz RAPOR; membran icin izin verilen sinir bilinmiyor (K14)
    Varsayimlar (dogrulanmadi): STL birimi mm, +Z yukari, tek katmanli yuzey
    (ust yuzey degil butun verteksler kullanilir)."""
    mesh = trimesh.load(stl_file)
    points = np.asarray(mesh.vertices)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] < 3:
        raise ValueError("STL içinde yeterli nokta yok (boş veya bozuk dosya)")

    minX, minY, _ = points.min(axis=0)
    maxX, maxY, maxZ = points.max(axis=0)

    # fill_value=nan ile ayni interpolasyon; nan'lar sonra maxZ ile doldurulur
    # (eski fill_value=maxZ + nan->maxZ ile birebir ayni sonuc, sayilabilir)
    F = LinearNDInterpolator(points[:, :2], points[:, 2], fill_value=np.nan)

    X, Y = np.meshgrid(
        np.linspace(minX, maxX, cols),
        np.linspace(minY, maxY, rows),
    )
    Z = F(X.ravel(), Y.ravel())
    outside = np.isnan(Z)
    Z[outside] = maxZ

    # merkez center_mm olacak sekilde offset, sonra guvenli araliga kirp
    offset = center_mm - float(np.mean(Z))
    Zs = Z + offset
    low = int(np.sum(Zs < min_mm))
    high = int(np.sum(Zs > max_mm))
    Z = np.clip(Zs, min_mm, max_mm)
    grid = Z.astype(int)

    g2 = grid.reshape(rows, cols)
    dn = 0
    if rows > 1:
        dn = max(dn, int(np.max(np.abs(np.diff(g2, axis=0)))))
    if cols > 1:
        dn = max(dn, int(np.max(np.abs(np.diff(g2, axis=1)))))
    info = {
        "cells": int(rows * cols),
        "filled": int(np.sum(outside)),
        "clipped_low": low,
        "clipped_high": high,
        "offset_mm": round(offset, 1),
        "max_neighbor": dn,
    }
    return grid, mesh, info


def grid_stats(grid_1d):
    g = np.asarray(grid_1d, dtype=float)
    return {
        "min": int(g.min()),
        "max": int(g.max()),
        "mean": int(g.mean()),
        "std": int(g.std()),
    }
