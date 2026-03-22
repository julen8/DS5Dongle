# GitHub Actions CI/CD 工作流说明

## 📋 概述

本项目使用 GitHub Actions 自动编译和发布固件。工作流在以下情况下触发：

- ✅ 推送版本标签（`v*`）到仓库时 → 创建 Release 并发布固件
- ✅ 推送到 `main`/`master` 分支时 → 编译并保存 artifacts
- ✅ Pull Request 时 → 验证编译成功
- ✅ 手动触发（GitHub Actions 界面）

## 🚀 使用方法

### 1. 创建 Release 版本

```bash
# 创建版本标签
git tag v1.0.0
git push origin v1.0.0
```

工作流将自动：
1. 检出代码
2. 安装依赖和 Pico SDK
3. 编译项目
4. 创建 GitHub Release
5. 上传 `.uf2` 和 `.elf` 文件

### 2. 编译日常版本

推送到主分支时，工作流自动编译并存储 artifacts（保留 30 天）：

```bash
git push origin main
```

在 Actions 标签页查看构建结果和下载 artifacts。

### 3. 手动启动编译

在 GitHub Actions 界面点击 "Run workflow" 按钮手动启动。

## 📁 工作流文件

### build.yml（基础版本）
- 自动编译和发布
- 简单配置
- 适合快速开始

### build-release.yml（完整版本）
- 基础功能
- 构建缓存优化
- 策略矩阵支持
- 详细的构建报告
- 自动识别预发布版本
- 完整的闪写说明

## 🔧 自定义配置

### 修改触发条件

编辑 `.github/workflows/build-release.yml` 中的 `on:` 部分：

```yaml
on:
  push:
    tags:
      - 'v*'           # 匹配 v1.0.0, v1.0.1 等
    branches:
      - main
      - master
```

### 修改 Pico SDK 版本

在工作流文件顶部修改：

```yaml
env:
  PICO_SDK_VERSION: '2.2.0'
  TINYUSB_VERSION: '0.20.0'
```

### 修改开发板

在 CMakeLists.txt 中或工作流中设置：

```yaml
-DPICO_BOARD=pico2_w
```

## 📊 监控和调试

### 查看工作流日志

1. 进入 GitHub 仓库 → Actions 标签
2. 选择相应的工作流运行
3. 查看详细日志和构建输出

### 常见问题

**Q: 找不到编译依赖？**
- Actions 会自动安装，确保有网络连接和足够的时间

**Q: 如何使用私有依赖？**
- 在工作流中添加 SSH 密钥或 token

**Q: 构建超时？**
- 增加 runner 超时时间或优化构建配置

## 📝 Release 说明

发布时自动包含：
- 编译信息
- 闪写说明
- SDK 版本号
- 固件文件

## 🔐 权限说明

工作流需要以下权限：
- `contents: write` - 创建 Release 和上传文件
- `packages: read` - 读取包信息

## 📚 相关资源

- [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [GitHub Actions 文档](https://docs.github.com/en/actions)
- [softprops/action-gh-release](https://github.com/softprops/action-gh-release)
