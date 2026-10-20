"""Depo guvenligi (statik, dosyalar CALISTIRILMAZ - yalniz AST ile okunur).

1) Test toplayicilarinin (pytest / unittest) bulabilecegi test*.py adli dosyalar, ice
   aktarilinca seri porta yazmamali: modul duzeyinde yalniz import, sabit, fonksiyon/sinif
   tanimi ve `if __name__ == "__main__":` olabilir. (Web_UI/test_mov.py eskiden ice
   aktarilinca COM3'e MOV gonderiyordu.)
2) Donanimsiz test paketi gercek seri cihaz acmaz: connectPort/open yalniz "SIM" veya
   pyserial "loop://" ile cagrilir.
"""
import ast
import os
import re
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SKIP_DIRS = {"venv", ".venv", "node_modules", ".git", ".claude", "__pycache__", ".pio"}


def _repo_test_files():
    out = []
    for root, dirs, files in os.walk(REPO):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for f in files:
            if f.startswith("test") and f.endswith(".py"):
                out.append(os.path.join(root, f))
    return out


def _is_main_guard(node):
    return (isinstance(node, ast.If) and isinstance(node.test, ast.Compare) and
            isinstance(node.test.left, ast.Name) and node.test.left.id == "__name__")


SAFE_TOP = (ast.Import, ast.ImportFrom, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)


class ImportSafetyTest(unittest.TestCase):
    def test_test_named_files_do_nothing_on_import(self):
        files = _repo_test_files()
        self.assertTrue(any(f.endswith(os.path.join("Web_UI", "test_mov.py")) for f in files))
        for path in files:
            with open(path, encoding="utf-8") as fh:
                tree = ast.parse(fh.read(), filename=path)
            for node in tree.body:
                if isinstance(node, SAFE_TOP) or _is_main_guard(node):
                    continue
                if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant):
                    continue                      # docstring
                if isinstance(node, (ast.Assign, ast.AnnAssign)):
                    val = node.value
                    calls = [n for n in ast.walk(val) if isinstance(n, ast.Call)] if val else []
                    bad = [c for c in calls if "Serial" in ast.dump(c.func)]
                    self.assertFalse(bad, f"{path}:{node.lineno} ice aktarmada seri port aciyor")
                    continue
                if isinstance(node, ast.Try) and "serial" in ast.dump(node).lower() \
                        and "Serial" in ast.dump(node):
                    self.fail(f"{path}:{node.lineno} ice aktarmada seri komut calistiriyor")
                if isinstance(node, (ast.Expr, ast.With, ast.For, ast.While, ast.Try)) and \
                        "Serial" in ast.dump(node):
                    self.fail(f"{path}:{node.lineno} modul duzeyinde seri islem")


class HardwareFreeSuiteTest(unittest.TestCase):
    def test_suite_never_opens_real_port(self):
        pat = re.compile(r"""(?:connectPort|\.open)\(\s*["']([^"']*)["']""")
        for f in os.listdir(HERE):
            if not f.endswith(".py"):
                continue
            with open(os.path.join(HERE, f), encoding="utf-8") as fh:
                src = fh.read()
            for dev in pat.findall(src):
                self.assertTrue(dev == "SIM" or dev.startswith("loop://"),
                                f"{f}: donanimsiz testte gercek port: {dev!r}")


if __name__ == "__main__":
    unittest.main()
