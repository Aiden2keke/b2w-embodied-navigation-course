# Codex 项目入口

安装、下载模型、启动仿真、执行验证或排查运行问题时，先阅读 [CODEX_GUIDE.md](CODEX_GUIDE.md)，按对应流程执行并检查完成条件。

修改共享 topic、TF、消息类型或模块边界前，阅读 [docs/INTERFACE.md](docs/INTERFACE.md) 和 [docs/INTERFACE_ALIGNMENT.md](docs/INTERFACE_ALIGNMENT.md)。

## APRIL Review Bundle 固定交付目录

所有未来 APRIL Review Bundle 及对应 `.sha256` 校验文件必须最终交付到
`/home/zyd/APRIL_REVIEW_BUNDLES/`。不得覆盖已有文件，不得删除历史证据。
临时证据、构建产物及 bundle 草稿可以留在 `/tmp`，但临时 bundle 路径绝不能
作为最终交付路径。最终回复必须报告固定目录中的绝对路径和已复核的 SHA256。

使用 `/home/zyd/bin/april-review-bundle` 生成 bundle；该 helper 强制固定目录并
生成、复核 checksum。若使用 compact/adapted workflow 在临时目录生成 bundle，
最终交付必须执行以下命令，成功后才能报告完成（禁止跳过交付步骤）：

```bash
/home/zyd/bin/april-review-bundle --deliver-existing /absolute/path/to/APRIL_REVIEW_BUNDLE_NAME.md EXPECTED_SHA256
```

固定目录若因 sandbox 限制不可写，申请对该目录的定向权限；不得改用 `/tmp`
作为最终交付位置。仅搬移已有 bundle 无需重新生成 bundle 或开启新的审查周期。
