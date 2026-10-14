"""protocol.py birim testleri: katı STAT şeması, U2ERR eşlemesi, terminal, hedef doğrulama."""
import unittest

import support  # noqa: F401  (import yolu)
import protocol as P


class TokenTest(unittest.TestCase):
    def test_strict_token_schema(self):
        self.assertEqual(P.parse_token("S300"), ("S", 300))
        self.assertEqual(P.parse_token("F0"), ("F", 0))
        for bad in ("S-1", "Sgarbage", "X1", "", "S", "s300", "S 300", "S3.5", "U2ERR"):
            self.assertIsNone(P.parse_token(bad), bad)


class InterpretStatTest(unittest.TestCase):
    def test_nine_valid_tokens_complete(self):
        r = P.interpret_stat(",".join(["S300"] * 9))
        self.assertTrue(r["complete"])
        self.assertEqual(r["motors"][8], ("S", 300))

    def test_one_invalid_token_makes_that_motor_unknown(self):
        toks = ["S300"] * 9
        toks[3] = "Sxx"
        r = P.interpret_stat(",".join(toks))
        self.assertFalse(r["complete"])
        self.assertTrue(r["malformed"])
        self.assertIsNone(r["motors"][3])
        self.assertEqual(r["motors"][2], ("S", 300))

    def test_wrong_count_all_unknown_but_fault_kept(self):
        r = P.interpret_stat("S0,F0,S0")
        self.assertFalse(r["complete"])
        self.assertTrue(all(m is None for m in r["motors"]))
        self.assertEqual(r["unmapped_faults"], 1)

    def test_u2err_without_variant_is_unknown(self):
        # K4: surum yapilandirilmadi -> motor numarasi belirlenemez, F kaybolmaz
        r = P.interpret_stat("F0,S0,S0,S0,U2ERR", None)
        self.assertTrue(r["u2err"])
        self.assertTrue(all(m is None for m in r["motors"]))
        self.assertEqual(r["unmapped_faults"], 1)

    def test_u2err_v5f411_maps_local_1_4(self):
        r = P.interpret_stat("F0,S10,S20,S30,U2ERR", "v5f411")
        self.assertEqual(r["motors"][0], ("F", 0))
        self.assertEqual(r["motors"][3], ("S", 30))
        self.assertTrue(all(m is None for m in r["motors"][4:]))
        self.assertEqual(r["unmapped_faults"], 0)

    def test_u2err_v51_maps_local_6_9(self):
        r = P.interpret_stat("U2ERR,S1,S2,S3,F4", "v5.1")
        self.assertTrue(all(m is None for m in r["motors"][:5]))
        self.assertEqual(r["motors"][5], ("S", 1))
        self.assertEqual(r["motors"][8], ("F", 4))

    def test_u2err_position_conflicting_with_variant(self):
        r = P.interpret_stat("F0,S0,S0,S0,U2ERR", "v5.1")
        self.assertTrue(r["variant_mismatch"])
        self.assertTrue(all(m is None for m in r["motors"]))
        self.assertEqual(r["unmapped_faults"], 1)


class TargetTest(unittest.TestCase):
    def test_check_target(self):
        for bad in ("", " ", "abc", "-5", "0", "9", "591", "600", "700", "3.5", "1e2"):
            self.assertTrue(P.check_target(bad), bad)
        for good in ("10", "300", "590", " 42 "):
            self.assertEqual(P.check_target(good), "", good)

    def test_check_target_value(self):
        self.assertTrue(P.check_target_value(10.5))
        self.assertEqual(P.check_target_value(300), "")
        self.assertTrue(P.check_target_value(None))


class TerminalTest(unittest.TestCase):
    def test_setid_locked(self):
        self.assertIn("SETID kilitli", P.parse_terminal("SETID:01:02").error)

    def test_unknown_verb_rejected(self):
        self.assertIn("Tanınmayan komut", P.parse_terminal("INTERNAL_MOV:1:300").error)
        self.assertIn("Tanınmayan komut", P.parse_terminal("FOO:01").error)

    def test_free_verbs(self):
        for t in ("PING:01", "STAT:16", "GETID:03", "GETPOS:01:09", "stat:01"):
            c = P.parse_terminal(t)
            self.assertEqual(c.error, "", t)

    def test_motion_verb_structure(self):
        self.assertEqual(P.parse_terminal("MOV:01:02:300").values, [300])
        self.assertTrue(P.parse_terminal("MOV:01:10:300").error)
        self.assertTrue(P.parse_terminal("MOV:01:02:0").error)
        self.assertTrue(P.parse_terminal("ALL:01:600").error)
        self.assertTrue(P.parse_terminal("ARR:01:1:2:3").error)
        self.assertEqual(P.parse_terminal("HOME:01").motor, None)
        self.assertEqual(P.parse_terminal("HOME:01:00").error, "")
        self.assertEqual(P.parse_terminal("HOME:01:03").motor, 3)
        self.assertTrue(P.parse_terminal("HOME:17").error)
        self.assertTrue(P.parse_terminal("STAT:1x").error)


class MatchTest(unittest.TestCase):
    def m(self, verb, sid, motor, line):
        return P.request_matches(verb, sid, motor, P.parse_reply(line))

    def test_matching(self):
        self.assertTrue(self.m("STAT", 1, None, "STAT:01:S0,S0"))
        self.assertFalse(self.m("STAT", 2, None, "STAT:01:S0"))
        self.assertTrue(self.m("ARR", 1, None, "ARRERR:FMT"))
        self.assertTrue(self.m("ARR", 1, None, "ARROK:01"))
        self.assertTrue(self.m("HOME", 1, 0, "HOMEOK:01:00"))
        self.assertFalse(self.m("HOME", 1, 0, "HOMEOK:01:03"))
        self.assertTrue(self.m("HOME", 1, 3, "HOMEERR:01:03"))
        self.assertTrue(self.m("MOV", 1, 2, "MOVOK:01:02:300"))
        self.assertFalse(self.m("MOV", 1, 2, "MOVOK:01:03:300"))
        self.assertTrue(self.m("PING", 5, None, "PONG:05"))
        self.assertFalse(self.m("PING", 5, None, "STAT:05:S0"))


if __name__ == "__main__":
    unittest.main()
