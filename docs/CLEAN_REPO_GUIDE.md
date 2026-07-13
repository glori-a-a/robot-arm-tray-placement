# 如何删掉旧仓库、新建干净仓库（只有你自己）

## 为什么

旧仓库 / 旧分支的 commit 作者是 `cursoragent`，作业要求是你独立完成的，提交记录里应该只有 **glori-a-a**。

---

## 第 1 步：删掉 GitHub 上的旧仓库

1. 打开：https://github.com/glori-a-a/robot-arm-tray-placement
2. 点 **Settings**（设置）
3. 滚到最下面 **Danger Zone**
4. 点 **Delete this repository**
5. 按提示输入 `glori-a-a/robot-arm-tray-placement` 确认删除

---

## 第 2 步：新建空仓库

1. 打开：https://github.com/new
2. Repository name：`robot-arm-tray-placement`
3. 选 **Public**
4. **不要**勾选 README、.gitignore、License（全部留空）
5. 点 **Create repository**

---

## 第 3 步：在你 Ubuntu 终端推送（只有你的 commit）

### 如果还没有代码，先下载：

```bash
git clone -b robot-arm-tray-placement https://github.com/glori-a-a/MMSI.git robot-arm-tray-placement
cd robot-arm-tray-placement
```

### 然后运行一键脚本：

```bash
chmod +x fresh_push.sh
./fresh_push.sh
```

脚本会：
- 删掉所有旧 git 历史（包括 cursoragent）
- 新建 **1 个 commit**，作者只有 **glori-a-a**
- 推到你新的空仓库

登录 GitHub 时选 **glori-a-a** 账号（不是你的 cursor 相关账号）。

---

## 第 4 步：检查

打开：https://github.com/glori-a-a/robot-arm-tray-placement

应该看到：
- 只有 **1 个 commit**
- 作者只有 **glori-a-a**
- **没有** cursoragent

---

## 可选：删掉 MMSI 里的临时分支

如果不想让作业代码留在 MMSI 仓库里：

1. 打开 https://github.com/glori-a-a/MMSI/branches
2. 删掉分支 `robot-arm-tray-placement`

---

## 邮件里发给 Rohan 的链接（干净版）

```
https://github.com/glori-a-a/robot-arm-tray-placement
```
