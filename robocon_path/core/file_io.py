"""Backup existing output, then replace atomically without truncating it."""
import os
from pathlib import Path
import shutil
import tempfile
import io
from datetime import datetime


def atomic_bytes(path, payload):
    path=Path(path)
    backup=None
    if path.exists():
        backup=Path(str(path)+".bak")
        if backup.exists():
            backup=Path(str(path)+"."+datetime.now().strftime("%Y%m%d%H%M%S%f")+".bak")
        shutil.copy2(path,backup)
    fd,tmp=tempfile.mkstemp(dir=path.parent,suffix=".tmp")
    try:
        with os.fdopen(fd,"wb") as f:
            f.write(payload);f.flush();os.fsync(f.fileno())
        if Path(tmp).read_bytes()!=payload: raise IOError("临时文件校验失败")
        os.replace(tmp,path)
        if path.read_bytes()!=payload: raise IOError("落盘校验失败，保留备份")
        if backup: backup.unlink()
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def atomic_text(path,text,encoding="utf-8"):
    atomic_bytes(path,text.encode(encoding))


def atomic_png(path,image):
    buffer=io.BytesIO()
    image.save(buffer,format="PNG")
    atomic_bytes(path,buffer.getvalue())
