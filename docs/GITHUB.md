# 上传 GitHub

本目录已经初始化为独立 Git 仓库，尚未连接远程仓库、提交或推送。

## 上传源码

在 GitHub 创建空仓库，仓库名建议 `27robocon_path_planner`。创建时不勾选自动生成 README、.gitignore 或 LICENSE，本目录已经提供前两项。

也可以用 GitHub Desktop 的 **Add local repository** 选择本目录，再检查 Changes、提交、Publish repository。

使用命令行时，在本目录运行：

```powershell
git status
git add .
git diff --cached --stat
git commit -m "Initial import of TR ground path planner"
git remote add origin <你的GitHub仓库地址>
git push -u origin main
```

若 Git 提示身份未配置，先配置自己的 `user.name` 和 `user.email`。

`.gitignore` 会排除 `dist/`、`out/`、虚拟环境、EXE、临时备份和缓存。通过网页手动上传时，只上传这些源码和文档目录；网页上传不会替你执行 Git 的忽略规则。

## 分享免安装 EXE

在 GitHub 仓库的 **Releases → Draft a new release** 中，把本地 `dist/ROBOCON_TR_Planner.exe` 与 `dist/使用说明.txt` 作为附件上传。

EXE内置地图和运行库，可单独下载使用。它随打包时的源码和地图更新；修改Python源码或JSON后，需重新执行 `python build_exe.py` 才会反映到EXE。

## 自动检查

`.github/workflows/checks.yml` 在push和pull request时运行基础验收。它使用仓库内的场景文件，不需要仿真引擎或外部mesh。外部源几何复核需在有对应仿真资源的机器上单独运行。

未添加许可证；发布者可根据项目与场地资源的授权自行决定许可证。
