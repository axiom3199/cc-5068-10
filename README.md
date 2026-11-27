# 声源来向分析台 · Acoustic Direction Lab

多通道时差（TDOA）声源来向估计，加上一个由估计结果驱动的虚拟相机跟拍控制器。
整个项目零运行时依赖，前端是原生 ES modules，服务端是一个 Node 静态服务器。

## 怎样启动界面

```bash
npm start          # 等同 node server.mjs
```

终端会打印：

```
Acoustic direction lab running at http://127.0.0.1:4173/
```

用浏览器打开 `http://127.0.0.1:4173/` 即可。换端口/监听地址：

```bash
node server.mjs --port 8080 --host 0.0.0.0
```

页面内的「怎么启动」按钮也写了同样的步骤。

## 它能做什么

- 阵列几何（预设或拖拽话筒自定义）生成**合成多通道录音**：球面波、精确 FFT 相位延迟、可控信噪比。
- 用 **GCC-PHAT** 逐对估计真实通道时差（含相干度、峰/底比、抛物线插值与 CRLB 不确定性）。
- 在**远场或近场**模型下反解方向（模型用 BIC 选择），给出**候选方向 + 可信范围**：
  - 三维阵列：唯一方向；
  - 平面阵列：前后/上下**镜像歧义**，把两支候选都保留；
  - 共线阵列 / 双话筒：**锥形歧义**，明确判为不可唯一；
  - 证据不足（低信噪、无源、模型不符）：状态为 `insufficient`，拒绝给唯一方向。
- 跟踪器在**世界坐标**里维护假设，歧义时**暂缓转向**并保持上次可信方向；相机以限速云台转向。
- 「探针转头」按几何预测选一个可分离镜像的姿态，转头后用**跨姿态一致性**排除镜像支路。

## 关键约定

| 项目 | 约定 |
| --- | --- |
| 坐标系 | x 向右，y 向上，z 向前；方位角自 +z 转向 +x，仰角向上；方向从阵列指向声源 |
| 时差符号 | `τ_ij = (i 的到达偏移) − (j 的到达偏移)`；`τ_ij` 与预测偏移一致才拟合 |
| 传播速度 | 显式输入（默认 343 m/s），远场方向对速度不敏感、残差会随错误速度上升 |
| 通道顺序 | 通过显式 `channelMap[mic] = 录制通道` 处理；「打乱录制通道」按钮会物理重排声道，由 map 还原 |
| 采样时间 | 显式 `sampleRate`、`startSample`、窗口长度（样本数在诊断面板可见） |
| 声源真值 | 只用于合成录音与图上校验，**从不进入推断**；推断只接收 channels / 几何 / 声速 |

最大幅度通道**不作为**方向答案；这一点有专门的对照测试（`loudestChannelBearing` 仅用于验证基线）。

## 测试

```bash
npm test             # 26 项内核测试：TDOA、方向、模型选择、歧义、跟踪、探针、输入校验
npm run validate     # 同上，带每项明细
npm run test:ui      # 13 项无头浏览器端到端检查（自动起临时服务器）
```

`test:ui` 用 Playwright 启动 Chromium，驱动真实页面并通过 `window.__doa` 验证：
页面能加载并得到真实解算、三维阵列误差 < 6°、平面阵保留 ≥ 2 候选且暂缓、
物理打乱通道后仍解出正确方向、无源时明确 `insufficient`、探针转头后锁定真支路并剪除镜像、
跟拍循环确实把云台转向已确认声源。

### 浏览器端到端检查的前置条件

`npm run test:ui` 需要 Playwright 的 Chromium：

```bash
npm i -D playwright
npx playwright install chromium
```

若宿主机缺少 Chromium 的共享库（常见于精简容器 / 非 root 环境），可把所需库解到一个本地
sysroot 再通过 `LD_LIBRARY_PATH` 传入，例如：

```bash
LD_LIBRARY_PATH=/path/to/sysroot/usr/lib/<triplet> npm run test:ui
```

本次开发即是在无 root、缺库的环境里用这种方式完成浏览器验证的。

## 目录

```
server.mjs             零依赖静态服务器
index.html             三栏界面
src/core/
  synthesize.js        合成录音（唯一持有声源真值的模块）
  tdoa.js              GCC-PHAT 时差估计与不确定度
  solver.js            几何分类、方向拟合、候选与可信范围、歧义判定
  analysis.js          对外唯一推断入口（输入校验、通道映射、汇聚）
  tracker.js           世界坐标假设跟踪、暂缓/转向决策
  disambiguation.js    镜像探针的几何预测
  camera.js            限速虚拟相机与由决策驱动的控制器
  geometry.js          坐标与时差约定、阵列↔世界旋转
  fft.js / wav.js      FFT 与 WAV 读写
src/ui/
  main.js              控制器：场景→录音→分析→跟踪→相机→渲染
  render.js            四个 Canvas 视图
  audio.js             逐通道试听与电平表
tests/                 内核测试、跟踪/探针评估、无头 UI 冒烟
```
