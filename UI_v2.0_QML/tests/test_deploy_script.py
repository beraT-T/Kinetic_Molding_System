"""deploy_pi.sh davranis testi - SAHTE ssh ile (gercek Pi'ye baglanilmaz, deploy yapilmaz).

Sahte ssh, uzak komut satirini YEREL bir bash ile gercekten calistirir (boru hatti,
"|| true" gibi kabuk anlamlari korunur). Uzak taraftaki git / venv python3 / sudo / tail
sahtedir: git dal/commit bilgisini ortamdan verir, python3 (QML kontrolu) istenirse
hata verir, sudo yalniz "RESTART" isaretini dosyaya yazar. Asil kontrol: dogrulama
basarisizsa HICBIR restart komutu CALISMAZ.
"""
import os
import stat
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(os.path.dirname(HERE), "deploy_pi.sh")
COMMIT = "0123456789abcdef0123456789abcdef01234567"

FAKES = {
    # uzak komut satirini kaydet ve yerel kabukta calistir
    "ssh": r'''#!/usr/bin/env bash
cmd="${@: -1}"
printf '%s\n' "$cmd" >> "$FAKE_SSH_LOG"
cmd="${cmd//\/home\/berat\/Kinetic_Molding_System/$FAKE_REPO}"
exec bash -c "$cmd"
''',
    "git": r'''#!/usr/bin/env bash
case "$*" in
  *"rev-parse --abbrev-ref HEAD"*) echo "${FAKE_BRANCH:-ui_v2.0}" ;;
  *"rev-parse HEAD"*) echo "${FAKE_HEAD}" ;;
  *"status --porcelain"*) [ -n "${FAKE_DIRTY:-}" ] && echo " M UI_v2.0_QML/main.py" ;;
  *"merge --ff-only"*|*"pull --ff-only"*) [ -n "${FAKE_FAIL_MERGE:-}" ] && exit 1 ;;
  *"log --oneline"*) echo "${FAKE_HEAD:0:7} deploy test" ;;
esac
exit 0
''',
    "timeout": '#!/usr/bin/env bash\nshift\nexec "$@"\n',
    "sudo": '#!/usr/bin/env bash\necho "RESTART $*" >> "$FAKE_RESTART_LOG"\nexit 0\n',
    "tail": '#!/usr/bin/env bash\necho "${FAKE_LOG:-kiosk basladi}"\n',
    "sleep": "#!/usr/bin/env bash\nexit 0\n",
}
FAKE_PY = r'''#!/usr/bin/env bash
if [ -n "${FAKE_FAIL_QML:-}" ]; then
  echo "UYARI: file:///Main.qml:12: TypeError: Cannot read property"
  exit 1
fi
exit 0
'''


def _exe(path, body):
    with open(path, "w") as f:
        f.write(body)
    os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


class DeployScriptTest(unittest.TestCase):
    script = SCRIPT

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="deploytest_")
        self.bin = os.path.join(self.tmp, "bin")
        os.makedirs(self.bin)
        for name, body in FAKES.items():
            _exe(os.path.join(self.bin, name), body)
        self.repo = os.path.join(self.tmp, "remote_repo")
        venv = os.path.join(self.repo, "UI_v2.0_QML", "venv", "bin")
        os.makedirs(venv)
        _exe(os.path.join(venv, "python3"), FAKE_PY)
        self.log = os.path.join(self.tmp, "ssh.log")
        self.rlog = os.path.join(self.tmp, "restart.log")

    def run_deploy(self, **over):
        env = {"PATH": self.bin + os.pathsep + os.environ.get("PATH", ""),
               "HOME": self.tmp, "FAKE_SSH_LOG": self.log, "FAKE_RESTART_LOG": self.rlog,
               "FAKE_REPO": self.repo, "FAKE_HEAD": COMMIT,
               "PI_HOST": "kalip@pi.test", "PI_REPO": self.repo,
               "DEPLOY_BRANCH": "ui_v2.0", "DEPLOY_COMMIT": COMMIT}
        for k, v in over.items():
            if v is None:
                env.pop(k, None)
            else:
                env[k] = v
        for p in (self.log, self.rlog):
            if os.path.exists(p):
                os.remove(p)
        p = subprocess.run(["bash", self.script], env=env, capture_output=True, text=True,
                           timeout=60)
        return p, self._lines(self.log), self._lines(self.rlog)

    @staticmethod
    def _lines(path):
        if not os.path.exists(path):
            return []
        with open(path) as f:
            return f.read().splitlines()

    def test_failed_qml_check_never_restarts(self):
        p, cmds, restarts = self.run_deploy(FAKE_FAIL_QML="1")
        self.assertNotEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertTrue(any("qml_load_check" in c for c in cmds), cmds)
        self.assertEqual(restarts, [], "QML kontrolu basarisizken kiosk yeniden baslatildi")
        self.assertFalse([c for c in cmds if "systemctl restart" in c])
        self.assertIn("YENİDEN BAŞLATILMADI", p.stderr)

    def test_wrong_branch_stops_before_update(self):
        p, cmds, restarts = self.run_deploy(FAKE_BRANCH="ui_v2.2")
        self.assertNotEqual(p.returncode, 0)
        self.assertFalse(any("merge" in c or "qml_load_check" in c for c in cmds), cmds)
        self.assertEqual(restarts, [])

    def test_commit_mismatch_stops(self):
        p, cmds, restarts = self.run_deploy(FAKE_HEAD="f" * 40)
        self.assertNotEqual(p.returncode, 0)
        self.assertFalse(any("qml_load_check" in c for c in cmds))
        self.assertEqual(restarts, [])

    def test_dirty_tree_or_failed_merge_stops(self):
        for kw in ({"FAKE_DIRTY": "1"}, {"FAKE_FAIL_MERGE": "1"}):
            p, cmds, restarts = self.run_deploy(**kw)
            self.assertNotEqual(p.returncode, 0, kw)
            self.assertEqual(restarts, [], kw)
            self.assertFalse(any("qml_load_check" in c for c in cmds), kw)

    def test_missing_or_unsafe_parameters(self):
        for kw in ({"PI_HOST": None}, {"DEPLOY_BRANCH": None},
                   {"DEPLOY_BRANCH": "ui_v2.0;reboot"}, {"DEPLOY_COMMIT": "HEAD~1"}):
            p, cmds, restarts = self.run_deploy(**kw)
            self.assertNotEqual(p.returncode, 0, kw)
            self.assertEqual(cmds, [], f"{kw}: gecersiz parametreyle ssh calisti")

    def test_success_restarts_once_after_checks(self):
        p, cmds, restarts = self.run_deploy()
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertEqual(len(restarts), 1)
        i_qml = next(i for i, c in enumerate(cmds) if "qml_load_check" in c)
        i_rst = next(i for i, c in enumerate(cmds) if "systemctl restart" in c)
        self.assertLess(i_qml, i_rst)

    def test_traceback_in_log_is_failure(self):
        p, _cmds, restarts = self.run_deploy(FAKE_LOG="Traceback (most recent call last):")
        self.assertEqual(p.returncode, 3)
        self.assertEqual(len(restarts), 1)


if __name__ == "__main__":
    unittest.main()
