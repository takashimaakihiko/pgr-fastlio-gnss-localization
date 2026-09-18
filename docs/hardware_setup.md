# ハードウェアセットアップ

## 実験用PC

- Ubuntu 22.04.5 LTS, x86_64, 4コア, 30GB RAM
- ROS 2 Humble インストール済み (本プロジェクトでは未使用)

## MID360 (Livox) — Ethernet

MID360 デフォルト: `192.168.1.1xx` (xx = シリアル下2桁)、DHCP server非搭載。

1. PC のEthernetポートを固定IP `192.168.1.5/24` に設定
   ```bash
   # 例: NetworkManager
   nmcli con add type ethernet ifname <IFACE> ip4 192.168.1.5/24
   ```
2. `config/mid360_lidar.json` の `host_ip` を同じIPにする
3. `ping 192.168.1.1xx` で疎通確認
4. SDK2サンプルで動作確認:
   ```bash
   third_party/Livox-SDK2/build/samples/livox_lidar_quick_start/livox_lidar_quick_start config/mid360_lidar.json
   ```
   (SDK2を単体ビルドした場合。本プロジェクトのビルドでは samples は除外)

データはUDPマルチキャスト `224.1.1.5` で配信される。

## CP2112 (USB→I2Cブリッジ) + PCA9685 (サーボPWM)

配線:

```text
PC ──USB──▶ CP2112 ──I2C(SDA/SCL/GND)──▶ PCA9685 ──PWM──▶ サーボ/ESC
              │                            │
         /dev/i2c-10                  I2C addr 0x40
         (hid_cp2112)                  VCC 3.3V (ロジック)
                                       V+ 5〜6V (サーボ電源)
```

### udevルール (sudoなしでアクセスするため)

`/etc/udev/rules.d/99-cp2112.rules` (設定済み):

```text
SUBSYSTEM=="i2c-dev", ATTRS{name}=="CP2112*", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea90", MODE="0660", GROUP="plugdev"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 確認コマンド

```bash
lsusb | grep 10c4          # CP2112 認識確認
i2cdetect -l               # "CP2112 SMBus Bridge" が出る
i2cdetect -y -r 10         # PCA9685: 0x40 (+ all-call 0x70)
i2cget -y 10 0x40 0x00     # MODE1レジスタ → 0x11 (デフォルト)
```

注意: i2cバス番号はCP2112接続のたびに変わる可能性あり (`i2cdetect -l` で名前確認)。
