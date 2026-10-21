# Codex 项目入口

安装、下载模型、启动仿真、执行验证或排查运行问题时，先阅读 [CODEX_GUIDE.md](CODEX_GUIDE.md)，按对应流程执行并检查完成条件。

修改共享 topic、TF、消息类型或模块边界前，阅读 [docs/INTERFACE.md](docs/INTERFACE.md) 和 [docs/INTERFACE_ALIGNMENT.md](docs/INTERFACE_ALIGNMENT.md)。

## Stage4 project records and experiment evidence

Warehouse Stage4 worktree: `/home/zyd/program/b2w-embodied-navigation-course-stage4`.
Engineering/review/diagnosis/test records belong in this repository, under `docs/reviews/`
and `experiments/g1_g5/evidence/`. Large raw logs, journals and build artifacts belong
under the corresponding evidence directory's `raw/` or `artifacts/` subdirectory.
Keep compact reports/indexes ready for Git review; ignored raw files are local only,
not backed up remotely. Do not modify the original shared checkout or Pipe V0.

`/home/zyd/APRIL_REVIEW_BUNDLES/` is a temporary exchange area for ONE concise APRIL
handoff bundle and checksum at each review gate, not the project archive. Use the
existing `~/bin/april-review-bundle` delivery implementation (Python-only adaptation
when shell auto-exit is prohibited); verify read-back/SHA256 and never overwrite
historical files. Durable report copies remain in this repository. No commit,
push, full Gazebo repeat, motion goal or nonzero authorization before external review.
