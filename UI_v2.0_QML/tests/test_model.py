"""Faz 3: atomik model yukleme, interpolasyon seffafligi, hedef != olcum (modul deposu)."""
import os
import unittest

from support import Rig, pump_events, write_box_stl, tmpdir
from stl_interpolator import interpolate_stl_to_grid, interpolate_stl_to_grid_info


class AtomicLoadTest(unittest.TestCase):
    def setUp(self):
        self.d = tmpdir()
        self.a = write_box_stl(os.path.join(self.d, "A.stl"), height=40)
        self.bad = os.path.join(self.d, "B.stl")
        with open(self.bad, "w") as f:
            f.write("bozuk\n")

    def test_success_sets_name_and_grid_together(self):
        r = Rig(modules=(1,))
        r.c.loadStl(self.a)
        self.assertEqual(r.c.modelState, "loading")
        self.assertEqual(r.c.modelName, "")
        self.assertEqual(list(r.c.gridData), [])
        self.assertTrue(pump_events(lambda: r.c.modelState == "ready"))
        self.assertEqual(r.c.modelName, "A.stl")
        self.assertEqual(len(r.c.gridData), 144)

    def test_loading_blocks_motion_and_second_load(self):
        r = Rig(modules=(1,))
        r.setup(grid=None)
        r.c.loadStl(self.a)
        self.assertIn("Model yükleniyor", r.c.productionBlockReason)
        self.assertIn("Model yükleniyor", r.c.sendSelectedBlockReason)
        n = len(r.failures())
        r.c.loadStl(self.a)                    # yeniden giris yok
        self.assertEqual(len(r.failures()), n + 1)
        pump_events(lambda: r.c.modelState == "ready")

    def test_failure_shows_error_and_leaves_no_model(self):
        r = Rig(modules=(1,))
        r.c.loadStl(self.a)
        pump_events(lambda: r.c.modelState == "ready")
        r.c.loadStl(self.bad)
        self.assertTrue(pump_events(lambda: r.c.modelState == "error"))
        self.assertEqual(r.c.modelName, "")
        self.assertEqual(list(r.c.gridData), [])
        self.assertTrue(r.c.modelError)
        self.assertEqual(r.failures()[-1][1], "Model yüklenemedi")

    def test_stale_result_ignored(self):
        r = Rig(modules=(1,))
        r.c._load_seq = 5
        r.c._on_job_done(4, True, {"name": "ESKI.stl", "path": "", "grid": tuple([1] * 144),
                                   "stats": {"min": 1, "max": 1, "mean": 1, "std": 0},
                                   "info": {}})
        self.assertEqual(r.c.modelName, "")

    def test_missing_file(self):
        r = Rig(modules=(1,))
        r.c.loadStl(os.path.join(self.d, "yok.stl"))
        self.assertEqual(r.c.modelState, "error")

    def test_model_change_blocked_during_operation(self):
        r = Rig(modules=(1,))
        r.setup()
        r.c.startHomeAll()
        r.run(200)
        self.assertTrue(r.c.modelBlockReason)
        r.c.loadStl(self.a)
        self.assertEqual(r.c.modelName, "test.stl")


class InterpolationInfoTest(unittest.TestCase):
    def test_same_grid_as_before_and_counts(self):
        import trimesh
        d = tmpdir()
        p = os.path.join(d, "kure.stl")
        trimesh.creation.icosphere(subdivisions=3, radius=50).export(p)
        g1, _ = interpolate_stl_to_grid(p)
        g2, _, info = interpolate_stl_to_grid_info(p)
        self.assertEqual(list(g1), list(g2))
        self.assertGreater(info["filled"], 0)         # kure izdusumu kareyi doldurmaz
        self.assertEqual(info["cells"], 144)
        self.assertGreaterEqual(info["max_neighbor"], 0)

    def test_clip_counts(self):
        import trimesh
        d = tmpdir()
        p = os.path.join(d, "uzun.stl")
        trimesh.creation.box(extents=(100, 100, 2000)).export(p)
        _g, _m, info = interpolate_stl_to_grid_info(p)
        self.assertGreater(info["clipped_low"] + info["clipped_high"], 0)


class TargetVsMeasurementTest(unittest.TestCase):
    def test_other_module_shows_unknown(self):
        r = Rig(modules=(1, 2))
        r.setup()
        r.br.mods[2].u3_down = True
        r.c.currentSlaveId = 1
        i1 = r.c.moduleInfo(1)
        self.assertTrue(all(m["known"] for m in i1["motors"]))
        i3 = r.c.moduleInfo(3)                 # hic cevap vermemis modul
        self.assertTrue(all(not m["known"] and m["mm"] == -1 for m in i3["motors"]))
        self.assertEqual(i3["ageS"], -1)

    def test_stat_does_not_overwrite_target(self):
        r = Rig(modules=(1,), speed=4.0)
        r.setup()
        r.home_all()
        r.c.moveMotor(1, 2, 200)
        r.run_s(2)
        info = r.c.moduleInfo(1)
        self.assertEqual(info["motors"][1]["target"], 200)
        self.assertLess(info["motors"][1]["mm"], 200)    # olcum hedefe henuz ulasmadi
        self.assertEqual(info["motors"][0]["target"], 0)  # home hedefi
        r.run_s(5)
        self.assertEqual(r.c.moduleInfo(1)["motors"][1]["target"], 200)

    def test_uncertain_acceptance_clears_target(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        r.br.reply_filter = lambda cmd, line: None if cmd.startswith("ALL") else line
        r.c.allToValue(1, 250)
        r.run_s(5)
        self.assertTrue(all(m["target"] == -1 for m in r.c.moduleInfo(1)["motors"]))

    def test_module_targets_from_model(self):
        r = Rig(modules=(1,))
        r.set_grid(list(range(144)))
        self.assertEqual(r.c.moduleTargets(1), [0, 1, 2, 12, 13, 14, 24, 25, 26])
        self.assertEqual(r.c.moduleTargets(16), [117, 118, 119, 129, 130, 131, 141, 142, 143])


if __name__ == "__main__":
    unittest.main()
