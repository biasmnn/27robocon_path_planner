"""Build and verify a single-file Windows EXE without changing planner code.

First install: python -m pip install -r requirements-build.txt
Then build:    python build_exe.py
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

from robocon_path.core.file_io import atomic_bytes, atomic_text

ROOT = Path(__file__).resolve().parent
NAME = "ROBOCON_TR_Planner"


def main():
    if sys.platform != "win32":
        raise SystemExit("Build on Windows with a 64-bit Python runtime.")
    (ROOT / "out").mkdir(exist_ok=True)
    env = os.environ.copy()
    work = Path(tempfile.mkdtemp(prefix="exe_build_", dir=ROOT / "out"))
    command = [sys.executable, "-m", "PyInstaller", "--onefile", "--windowed",
               "--noconfirm", "--name", NAME, "--distpath", str(work / "dist"),
               "--workpath", str(work / "build"), "--specpath", str(work),
               "--paths", str(ROOT), "--exclude-module", "matplotlib",
               "--exclude-module", "pandas", "--exclude-module", "tkinter",
               "--exclude-module", "torch", "--exclude-module", "IPython"]
    for name in ("nvwa_butian_2027_from_sim.json", "nvwa_butian_2027_template.json"):
        command.extend(["--add-data", str(ROOT / "scenes" / name) + ";scenes"])
    command.append(str(ROOT / "exe_entry.py"))
    print("Build directory:", work, flush=True)
    build = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8", errors="replace")
    atomic_text(work / "build.log", build.stdout + build.stderr)
    print((build.stdout + build.stderr)[-5000:], flush=True)
    if build.returncode:
        raise SystemExit(build.returncode)
    # Test the EXE alone: no sources, scenes, Python PATH or project CWD.
    isolated = work / "独立运行测试"
    isolated.mkdir()
    standalone = isolated / (NAME + ".exe")
    shutil.copy2(work / "dist" / standalone.name, standalone)
    test_env = os.environ.copy()
    for key in ("PYTHONPATH", "PYTHONHOME", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"):
        test_env.pop(key, None)
    test_env["PATH"] = os.pathsep.join([str(Path(os.environ["WINDIR"]) / "System32"), os.environ["WINDIR"]])
    smoke = subprocess.run([str(standalone), "--smoke-test", str(isolated / "结果")], cwd=isolated, env=test_env, timeout=180)
    report_path = isolated / "结果/result.json"
    print((isolated / "结果/output.txt").read_text(encoding="utf-8") if (isolated / "结果/output.txt").exists() else "EXE test produced no report", flush=True)
    if smoke.returncode or not report_path.exists():
        raise SystemExit(smoke.returncode or 1)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "PASS" and report["frozen"]
    destination = ROOT / "dist"
    destination.mkdir(exist_ok=True)
    target = destination / standalone.name
    atomic_bytes(target, standalone.read_bytes())
    digest = hashlib.sha256(target.read_bytes()).hexdigest()
    atomic_text(destination / "使用说明.txt", "ROBOCON TR 离线路径点表生成器\n\n双击 ROBOCON_TR_Planner.exe 即可运行，无需安装 Python。\n默认使用当前仿真场地，仅规划地面路线。\n可单独复制 EXE 到其他目录或另一台兼容的 Windows 64位电脑。\n首次启动需解压内置运行库，可能等待数秒。\n导出 C/CSV 与场景副本时，在保存对话框中选择自己的文件夹。\n启动故障记录：%LOCALAPPDATA%\\ROBOCON_TR_Planner\\startup_error.txt\n")
    atomic_text(destination / "打包验证.json", json.dumps(dict(sha256=digest, size_bytes=target.stat().st_size,
                build_dir=str(work), smoke_test=report), ensure_ascii=False, indent=2))
    print("Delivered:", target, flush=True)
    print("Size:", target.stat().st_size, "bytes; SHA256:", digest, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
