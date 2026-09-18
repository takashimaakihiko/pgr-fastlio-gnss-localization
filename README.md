# pgr-fastlio-gnss-localization

ラジコン車両 自己位置推定・経路追従実験装置 (Phase 1)

詳細な仕様・フェーズ定義は `pgr-fastlio-gnss-localization_readme.txt` を参照。

## 概要

Livox MID360 の点群+IMUを FAST-LIO でリアルタイムに自己位置推定し、
`x, y, theta` をCSVログして2D軌跡を可視化する。**ROSは使わない**
(最終的に経路生成形レギュレータを中核に据えるため、単一プロセス構成)。

```text
MID360 ──Ethernet──▶ Livox-SDK2 ──▶ FAST-LIO ──▶ pose (x,y,z,quat)
                                        │
                                        ▼
                              trajectory.csv (x,y,theta等)
                                        │
                                        ▼
                              plot_trajectory.py
```

## 構成

```text
├── config/
│   ├── mid360_lidar.json     Livox-SDK2 ネットワーク設定 (host_ip等)
│   └── mid360_fastlio.json   FAST-LIO パラメータ
├── src/
│   ├── app_main.cpp          lio_node メイン
│   ├── mid360/lidar_source.* SDK2→CustomMsg変換・フレーム集約
│   ├── fastlio/              FAST-LIO ROS非依存版 (vendor)
│   │   ├── include/          IKFoM, ikd-Tree, msgs 等
│   │   └── src/              laserMapping, IMU_Processing, preprocess
│   ├── pose/pose_utils.hpp   quaternion→rpy, 角度正規化
│   └── logger/trajectory_logger.hpp  CSVロガー
├── scripts/plot_trajectory.py
├── data/trajectory/
├── third_party/Livox-SDK2    (git submodule)
└── docs/
```

## ビルド

```bash
git clone --recursive <this-repo>
cd pgr-fastlio-gnss-localization
mkdir build && cd build
cmake .. && make -j$(nproc)
```

依存: Eigen3, PCL (>=1.8), OpenMP, python3-dev, boost (Ubuntu 22.04 デフォルトでOK)

## 実行

MID360 との接続:

1. PC のEthernetを固定IP `192.168.1.5/24` に設定
   (MID360 デフォルト: `192.168.1.1xx`、xxはSN下2桁)
2. `config/mid360_lidar.json` の `host_ip` をそのIPに合わせる
3. 実行:

```bash
cd build
./lio_node ../config/mid360_lidar.json ../config/mid360_fastlio.json ../data/trajectory
```

停止は Ctrl-C。`data/trajectory/trajectory.csv` に
`timestamp,x,y,z,roll,pitch,yaw` が記録される。

## 軌跡可視化

```bash
python3 scripts/plot_trajectory.py data/trajectory/trajectory.csv
```

## 車両制御 (Phase 2以降)

CP2112 (USB→I2C) + PCA9685 (PWM) でサーボ/ESCを駆動する構成。
- CP2112 → `/dev/i2c-10` (udev rule: `docs`参照)
- PCA9685 → `0x40`

## クレジット

- FAST-LIO core: [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) を
  [BurhanMuhyiddin/FAST-LIO-NON-ROS](https://github.com/BurhanMuhyiddin/FAST-LIO-NON-ROS) 由来の
  ROS非依存版として `src/fastlio/` にvendor (ライセンス: GPL-2.0)
- LiDAR入力: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) (MIT)
