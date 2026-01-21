# 相机内参实时标定（C++/OpenCV）

本项目在 Windows 上构建一个可编译的 C++/CMake 工程，使用 OpenCV 实现实时采集与标定，支持针孔与鱼眼模型。默认棋盘格内角点为 `9x6`，格长 `25mm`。

## 构建

1. 安装 OpenCV（已配置 `OpenCV_DIR` 或加入系统环境变量）。
2. 生成并编译：

```
cmake --build build --config Release
```

## 运行

```
build\Release\camera_calibration.exe --device 0 --cols 9 --rows 6 --square 25 --model pinhole --output calibration.yaml
```

离线输入（视频或图像文件夹）：

```
build\Release\camera_calibration.exe --input "D:\data\calib.mp4" --model pinhole --output calibration.yaml
build\Release\camera_calibration.exe --images "D:\data\images" --model fisheye --output fisheye.yaml
```

视频解码说明（H264/H265 等）：
- 是否能打开视频文件取决于 OpenCV 编译的后端（FFmpeg/GStreamer）。
- Windows 上建议使用带 FFmpeg 的 OpenCV 预编译包，才能直接读取 H264 的 `mp4`/`avi`。

配置文件模式：

```
build\Release\camera_calibration.exe --config config.yaml
```

常用参数：
- `--config`：YAML 配置文件
- `--device`：摄像头索引
- `--input`：视频文件或图像文件夹
- `--images`：图像文件夹（强制图片序列）
- `--input-type`：`auto` / `video` / `images`
- `--cols` / `--rows`：棋盘格内角点数量（默认 9x6）
- `--square`：格长（mm）
- `--model`：`pinhole` 或 `fisheye`
- `--output`：输出 YAML 路径
- `--manual`：关闭自动触发保存，手动按 `s`

键位：
- `q` / `Esc`：退出
- `r`：重置已采样的标定帧
- `s`：当覆盖度达到要求时保存 YAML

## 配置文件示例（YAML）

```
app:
  show_window: 1
  max_seconds: 120
  max_frames: 0
  output_dir: "outputs"
  report_dir: "reports"

cameras:
  - name: "cam0"
    device: 0
    input_path: ""
    input_type: "auto"
    model: "pinhole"
    board_cols: 9
    board_rows: 6
    square_size: 25
    output: "cam0.yaml"
    report: "cam0_report.md"
    auto_calibrate: 1
    frame_selector:
      min_area_ratio: 0.03
      max_area_ratio: 0.60
      min_sharpness: 80.0
      min_corner_quality: 0.25
      min_center_distance_ratio: 0.06
      min_area_diff_ratio: 0.02
      min_tilt_diff: 0.05
      max_per_bucket: 6
      min_total_samples: 20
      min_position_buckets: 6
      min_scale_buckets: 3
      min_tilt_buckets: 3
```

## 采样策略（质量评估）

每帧都会计算以下指标并决定是否采样：
- **尺寸占比**：棋盘在画面中的面积占比，避免过小或过大。
- **位置覆盖**：将画面划分为 `3x3` 区域，要求采样覆盖多个区域。
- **视角变化**：根据棋盘四角透视畸变估计倾斜度，鼓励不同视角。
- **清晰度**：基于 Laplacian 方差过滤模糊帧。
- **角点质量**：角点亚像素收敛位移越小，质量越高。
- **多样性**：与已有样本的中心位置、尺度、倾斜度差异过小时不采样。

采样不会固定张数，而是依据覆盖度与质量阈值动态触发。

## 标定结果判定（建议标准）

以下标准用于判断标定质量是否可信（建议在采集完成后检查）：

1. **重投影误差**
   - 平均重投影误差 < 0.5 px（针孔）或 < 0.8 px（鱼眼）
   - 最大重投影误差不过高（例如 < 2.0 px）
2. **畸变参数合理性**
   - 针孔模型 `k1, k2` 绝对值通常不应过大，`k3` 高阶项更小
   - 鱼眼模型 `k1..k4` 逐阶递减且不应出现明显爆炸
3. **焦距与主点**
   - `fx, fy` 与图像宽高同量级
   - 主点 `(cx, cy)` 接近图像中心，偏差应小于 5% 尺寸
   - `fx / fy` 比例接近 1（除非像素长宽比不一致）
4. **边缘残差**
   - 对棋盘角点重新投影，统计图像边缘区域残差，边缘不应显著偏大
5. **视角覆盖均衡**
   - 视角分桶覆盖尽量完整，避免样本集中在正视或单一方向
6. **交叉验证**
   - 留出部分样本不参与标定，仅用于评估独立重投影误差

## 输出文件

YAML 中包含：
- `model`：`pinhole` / `fisheye`
- `image_width` / `image_height`
- `camera_matrix`
- `distortion_coefficients`
- `rms`
- `mean_reprojection_error`

报告文件（Markdown）包含：
- 会话统计与覆盖度统计
- 采样配置与拒绝原因统计
- 内参与畸变参数矩阵

离线输入模式下，所有通过质量筛选的帧会保存到 `outputs/samples/`，用于复查与复用。
