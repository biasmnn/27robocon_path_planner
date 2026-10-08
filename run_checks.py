"""Portable acceptance: python run_checks.py [--with-sim]."""
from pathlib import Path
import argparse
import os
import subprocess
import sys

from robocon_path.core.file_io import atomic_text

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--with-sim", action="store_true", help="also read external OBJ/SDF and inject bad scenes")
    args = parser.parse_args()
    commands = []
    if args.with_sim:
        sim = Path(os.environ.get("ROBOCON_SIM_ROOT", str(ROOT.parent / "27th_gap_gazebo_model/gazebo_models-main"))).expanduser().resolve()
        if not (sim / "robocon_mujoco/meshes/robocon_000.obj").is_file() or not (sim / "robocon_ground/model.sdf").is_file():
            print(f"外部仿真源复核未运行：缺少OBJ/SDF，路径={sim}。设置ROBOCON_SIM_ROOT后重试。")
            return 2
        commands = [["robocon_path/core/sim_bridge.py", "--check"],
                    ["verify_from_sim.py"], ["probe_sim_field.py", "verify"],
                    ["selftest.py", "--verify"], ["selftest.py"], ["integrity.py", "--check"],
                    ["rules_selftest.py"]]
    else:
        print("未运行外部仿真源复核；基础检查仅使用仓库内的已生成地图。", flush=True)
        commands = [["integrity.py", "--check"], ["verify_from_sim.py"],
                    ["selftest.py", "--verify"], ["selftest.py"]]
    commands += [["selftest.py", "--verify", "scenes/nvwa_butian_2027_template.json"],
                 ["selftest.py", "--verify", "scenes/nvwa_butian_2027_from_sim.json"],
                 ["selftest.py", "scenes/nvwa_butian_2027_from_sim.json"],
                 ["ground_selftest.py"], ["draw_field.py", "scenes/nvwa_butian_2027_from_sim.json"],
                 ["uitest.py"], ["integrity.py", "--check"]]
    logs = []
    (ROOT / "out").mkdir(exist_ok=True)
    logfile = ROOT / "out" / ("checks_with_sim.md" if args.with_sim else "checks_basic.md")
    for command in commands:
        print("运行: python " + " ".join(command), flush=True)
        result = subprocess.run([sys.executable, "-u", *command], cwd=ROOT,
                                capture_output=True, text=True, encoding="utf-8", errors="replace")
        output = result.stdout + result.stderr
        print(output, flush=True)
        logs.append("## python " + " ".join(command) + f"\n\nexit={result.returncode}\n\n```text\n{output}\n```\n")
        atomic_text(logfile, "\n".join(logs))
        if result.returncode:
            print(f"FAIL，完整输出：{logfile}")
            return result.returncode
    print(f"{'包含外部源复核的完整' if args.with_sim else '基础'}验收 PASS；完整输出：{logfile}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
