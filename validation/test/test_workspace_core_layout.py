"""Cold-process regressions for the two cores and shared workspace assets.

These checks deliberately avoid the importing pytest process: another test's
bootstrap must not hide a missing path when a CLI or spawned worker starts.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
SIMULATOR_DIR = REPO_ROOT / "CMC-like Simulator"


class WorkspaceCoreLayoutTests(unittest.TestCase):
    def _run(self, arguments: list[str], cwd: Path) -> subprocess.CompletedProcess:
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        result = subprocess.run(
            [sys.executable, "-B", "-I", *arguments],
            cwd=cwd,
            env=env,
            capture_output=True,
            text=True,
            timeout=90,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def test_cold_bootstrap_keeps_shared_assets_at_workspace_root(self):
        script = textwrap.dedent("""\
            from pathlib import Path
            import sys

            root = Path(sys.argv[1])
            sys.path.insert(0, str(root))
            import workspace_bootstrap as workspace

            heavy = {"numpy", "scipy", "torch", "ray", "opensim"}
            assert not (heavy & sys.modules.keys())
            assert workspace.REPO_ROOT == root
            workspace.ensure_simulator_paths()
            workspace.ensure_simulator_paths()
            assert not (heavy & sys.modules.keys())
            assert sys.path.count(str(workspace.SIMULATOR_DIR)) == 1
            assert sys.path.count(str(root)) == 1

            import config
            import path_resolver
            assert Path(config.__file__).parent == workspace.SIMULATOR_DIR
            assert Path(path_resolver.__file__).parent == workspace.SIMULATOR_DIR
            assert path_resolver.REPO_ROOT == root
            resolved = path_resolver.resolve_simulator_paths(config.SimulatorConfig())
            assert resolved.repo_root == root
            assert resolved.model_path.is_file()
            assert resolved.kinematics_path.is_file()
            assert resolved.model_path.is_relative_to(root / "models")
            assert path_resolver.SHARED_GEOMETRY_DIR == root / "models" / "Geometry"
            assert path_resolver.ONLINE_GRF_DIR == root / "online_GRF"
            assert path_resolver.SEA_PLUGIN_DIR == root / "plugins" / "SEA"
            print("cold workspace bootstrap and asset resolution: OK")
            """)
        with tempfile.TemporaryDirectory() as tmp:
            result = self._run(["-c", script, str(REPO_ROOT)], Path(tmp))
        self.assertIn("asset resolution: OK", result.stdout)

    def test_spawn_preserves_historical_flat_config_pickles(self):
        script = textwrap.dedent("""\
            import multiprocessing
            from pathlib import Path
            import pickle
            import sys

            ROOT = Path(sys.argv[1])

            def worker(connection):
                # Remove paths inherited from the parent's bootstrap. The child
                # must establish the contract itself before reading a pickle.
                hidden = {str(ROOT), str(ROOT / "CMC-like Simulator")}
                sys.path[:] = [entry for entry in sys.path if entry not in hidden]
                assert "config" not in sys.modules
                sys.path.insert(0, str(ROOT))
                import workspace_bootstrap
                workspace_bootstrap.ensure_simulator_paths()
                import config
                import path_resolver

                # Protocol-0 GLOBAL is the pre-move module/class identity.
                legacy_class = pickle.loads(b"cconfig\\nSimulatorConfig\\n.")
                assert legacy_class is config.SimulatorConfig
                value = pickle.loads(connection.recv_bytes())
                assert type(value) is legacy_class
                assert value.output_prefix == "spawn_layout_probe"
                resolved = path_resolver.resolve_simulator_paths(value)
                assert resolved.model_path.is_file()
                connection.send((config.SimulatorConfig.__module__,
                                 str(Path(config.__file__).parent),
                                 str(resolved.repo_root)))
                connection.close()

            if __name__ == "__main__":
                sys.path.insert(0, str(ROOT))
                import workspace_bootstrap
                workspace_bootstrap.ensure_simulator_paths()
                from config import SimulatorConfig

                value = SimulatorConfig(output_prefix="spawn_layout_probe")
                context = multiprocessing.get_context("spawn")
                parent, child = context.Pipe()
                process = context.Process(target=worker, args=(child,))
                process.start()
                child.close()
                try:
                    parent.send_bytes(pickle.dumps(value, protocol=4))
                    assert parent.poll(30), "spawned worker did not reply"
                    result = parent.recv()
                    assert result == ("config", str(ROOT / "CMC-like Simulator"), str(ROOT))
                finally:
                    parent.close()
                    process.join(30)
                    if process.is_alive():
                        process.terminate()
                        process.join()
                assert process.exitcode == 0, process.exitcode
                print("spawn and legacy flat pickle: OK")
            """)
        with tempfile.TemporaryDirectory() as tmp:
            cwd = Path(tmp)
            source = cwd / "spawn_layout_probe.py"
            source.write_text(script, encoding="utf-8")
            result = self._run([str(source), str(REPO_ROOT)], cwd)
        self.assertIn("legacy flat pickle: OK", result.stdout)

    def test_simulator_and_validator_cli_start_outside_workspace(self):
        with tempfile.TemporaryDirectory() as tmp:
            for script in (
                SIMULATOR_DIR / "main.py",
                REPO_ROOT / "validation" / "validate_sim_results.py",
            ):
                with self.subTest(script=script.name):
                    result = self._run([str(script), "--help"], Path(tmp))
                    self.assertIn("--help", result.stdout)


if __name__ == "__main__":
    unittest.main()
