"""文件完整性清单: 生成/校验工程关键文件的 sha256.

背景: 有一次外部 AI 助手在写入 scenes/*.json 时中途失败, 把文件截断成了 `{}`
(3 字节), 而且该目录不在版本控制下, 差点丢掉整份场地模板。

用法:
    python integrity.py --save      # 生成/刷新 integrity.sha256
    python integrity.py --check     # 校验(默认动作), 发现不一致返回码 1
"""

from __future__ import annotations

import hashlib
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "integrity.sha256")

# 需要保护的文件(相对路径); 目录会递归展开
WATCH = [
    "main.py", "selftest.py", "uitest.py", "draw_field.py", "field_lab.py",
    "add_rules_meta.py", "README.md", "AGENT.md",
    "requirements.txt", "requirements_install.bat", "run_app.bat",
    "robocon_path", "scenes",
    "build_template_from_sim.py", "probe_sim_field.py", "verify_from_sim.py", "integrity.py",
    "rules_selftest.py",
    "ground_selftest.py",
    "exe_entry.py", "exe_smoke.py", "build_exe.py",
    "requirements-build.txt", "run_checks.py", ".gitignore", ".gitattributes",
    "docs", "tests/fixtures", ".github",
]
SKIP_DIRS = {"__pycache__", "out", ".git"}


def _iter_files() -> list[str]:
    out: list[str] = []
    for item in WATCH:
        p = os.path.join(HERE, item)
        if os.path.isfile(p):
            out.append(p)
        elif os.path.isdir(p):
            for root, dirs, files in os.walk(p):
                dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
                for fn in sorted(files):
                    if fn.endswith((".pyc", ".exe", ".bak")):
                        continue
                    out.append(os.path.join(root, fn))
    return sorted(set(out))


def _digest(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _rel(path: str) -> str:
    return os.path.relpath(path, HERE).replace("\\", "/")


def save() -> int:
    lines = []
    for p in _iter_files():
        lines.append(f"{_digest(p)}  {_rel(p)}")
    fd, temp_path = tempfile.mkstemp(dir=HERE, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
            fh.write("\n".join(lines) + "\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(temp_path, MANIFEST)
    finally:
        if os.path.exists(temp_path):
            os.unlink(temp_path)
    print(f"已写出 {MANIFEST}  ({len(lines)} 个文件)")
    return 0


def check() -> int:
    if not os.path.isfile(MANIFEST):
        print(f"缺少 {MANIFEST}; 先运行 python integrity.py --save")
        return 1
    expected: dict[str, str] = {}
    with open(MANIFEST, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.rstrip("\n")
            if not line:
                continue
            digest, _, rel = line.partition("  ")
            expected[rel] = digest

    problems: list[str] = []
    now = {_rel(p): p for p in _iter_files()}

    for rel, digest in expected.items():
        if rel not in now:
            problems.append(f"缺失: {rel}")
            continue
        actual = _digest(now[rel])
        if actual != digest:
            size = os.path.getsize(now[rel])
            problems.append(f"被改过: {rel}  (现在 {size} 字节)")
    for rel in now:
        if rel not in expected:
            problems.append(f"新增(未登记): {rel}")

    if problems:
        print("=== 完整性校验 FAIL ===")
        for p in problems:
            print("  -", p)
        print("\n如果确认改动是有意的, 运行 python integrity.py --save 刷新清单。")
        return 1
    print(f"=== 完整性校验 PASS ({len(expected)} 个文件) ===")
    return 0


def main() -> int:
    if "--save" in sys.argv:
        return save()
    return check()


if __name__ == "__main__":
    raise SystemExit(main())
