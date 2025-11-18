# FAST-LIO ROS2 Robosense LiDAR対応版

このリポジトリは、[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)をベースに、Robosense社製LiDARのサポートを追加したものです。

## サポートするRobosense LiDARモデル

以下のRobosense MEMS LiDARをサポートしています：

- **RS-LiDAR-M1** - 5ラインMEMS LiDAR
- **RS-LiDAR-E1R** - 5ラインMEMS LiDAR（AC1の前モデル）
- **RS-LiDAR-AC1** - 5ラインMEMS LiDAR（最新モデル）
- **RS-LiDAR-Airy** - 32ラインMEMS LiDAR

これらのLiDARは同一の点群フォーマットを使用しているため、同じ実装でサポートされます。

## Robosense対応の実装詳細

### 1. 点群データ構造

Robosense LiDARは以下の点群フォーマットを使用します：

```cpp
namespace robosenseM1_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;              // x, y, z座標
        PCL_ADD_INTENSITY;            // 反射強度
        uint16_t ring = 0;            // レーザーリング番号
        double timestamp = 0;         // タイムスタンプ（秒単位の絶対時刻）
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(robosenseM1_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)
```

**重要な特徴：**
- `timestamp`フィールドは`double`型（他社のLiDARは`float time`や`uint32_t t`を使用）
- タイムスタンプは**絶対時刻**（秒単位）として保存される
- Velodyne、Ousterとは異なるデータ構造のため、専用の処理が必要

### 2. LiDARタイプの定義

`src/preprocess.h`でRobosense用のLiDARタイプを追加：

```cpp
enum LID_TYPE
{
  AVIA = 1,        // Livox Avia
  VELO16,          // Velodyne VLP-16
  OUST64,          // Ouster OS1-64
  MID360,          // Livox MID-360
  RSM1,            // Robosense M1/E1R/AC1/Airy（標準モード）
  RSM1_BREAK       // Robosense M1/E1R/AC1/Airy（サブクラウド分割モード）
};
```

**2つのモード：**
- **RSM1**: 標準モード。点群全体を一度に処理
- **RSM1_BREAK**: サブクラウド分割モード。大きな点群を複数のサブクラウドに分割して処理（高速化・メモリ効率向上）

### 3. タイムスタンプ変換処理

Robosense LiDARの絶対時刻タイムスタンプを、FAST-LIOが使用する相対時刻（スキャン開始からの経過時間）に変換します。

`src/preprocess.cpp`の`robosenseM1_handler()`関数内：

```cpp
void Preprocess::robosenseM1_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                                     int i_sub_cloud, int num_sub_cloud,
                                     double & start_time, double & end_time)
{
    // 点群データを読み込み
    pcl::PointCloud<robosenseM1_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    // 最初の点のタイムスタンプを基準時刻として設定
    start_time = pl_orig.points[pl_orig.points.size() - 1].timestamp;
    end_time = pl_orig.points[0].timestamp;

    // 各点を処理
    for (auto &ori_point : pl_orig.points) {
        // 絶対時刻を相対時刻に変換
        // curvatureフィールドに相対時刻を格納（FAST-LIOの仕様）
        added_pt.curvature = (ori_point.timestamp - start_time) * time_unit_scale;

        // 点を追加
        pl_surf.points.push_back(added_pt);
    }
}
```

**処理の流れ：**
1. 点群の最初の点のタイムスタンプ（`timestamp`）を`start_time`として保存
2. 各点のタイムスタンプから`start_time`を引いて相対時刻を計算
3. 相対時刻を`time_unit_scale`で正規化（秒、ミリ秒、マイクロ秒などの単位変換）
4. 変換した相対時刻を`curvature`フィールドに格納（FAST-LIOが使用）

### 4. サブクラウド分割モード（RSM1_BREAK）

大量の点群を効率的に処理するため、サブクラウド分割機能を実装：

`src/laserMapping.cpp`の`standard_pcl_cbk()`関数内：

```cpp
if (p_pre->lidar_type == RSM1_BREAK) {
    // 点群を num_sub_cloud 個のサブクラウドに分割して処理
    for (int i_sub_cloud = 0; i_sub_cloud < num_sub_cloud; i_sub_cloud++) {
        p_pre->process(msg, ptr, i_sub_cloud, num_sub_cloud, start_time, end_time);
    }
} else {
    // 標準モード：点群全体を一度に処理
    p_pre->process(msg, ptr);
}
```

**サブクラウド分割の利点：**
- メモリ使用量の削減
- リアルタイム処理性能の向上
- 大規模な点群データの安定した処理

### 5. 主な変更ファイル

#### `src/preprocess.h` (111-133行目)
- `robosenseM1_ros::Point`構造体の定義
- `RSM1`、`RSM1_BREAK`のLiDARタイプ追加
- `robosenseM1_handler()`関数の宣言
- サブクラウド対応の`process()`オーバーロード追加

#### `src/preprocess.cpp` (95-140行目、1118-1228行目)
- サブクラウド分割対応の`process()`関数実装
- `robosenseM1_handler()`関数の実装
- タイムスタンプ変換ロジック

#### `src/laserMapping.cpp` (97行目、300-322行目、409-413行目、830行目、867行目)
- `num_sub_cloud`パラメータの追加
- RSM1_BREAKモードの分岐処理
- タイムスタンプ補正処理

#### `config/robosense_ac1.yaml`
- AC1/E1R/M1（5ライン）用の設定ファイル
- `lidar_type: 5` (RSM1)
- `scan_line: 5`

#### `config/robosense_airy.yaml`
- Airy（32ライン）用の設定ファイル
- `lidar_type: 5` (RSM1)
- `scan_line: 32`

#### `launch/mapping_robosense_ac1.launch.py`
- AC1/E1R/M1用のlaunchファイル

#### `launch/mapping_robosense_airy.launch.py`
- Airy用のlaunchファイル

## 使用方法

### 1. ビルド

```bash
cd ~/ros2_ws/src
git clone https://github.com/nkys39/robosense_fast_lio.git fast_lio
cd ~/ros2_ws
git submodule update --init --recursive
colcon build --packages-select fast_lio
source install/setup.bash
```

### 2. 起動

#### AC1/E1R/M1（5ライン）の場合：

```bash
ros2 launch fast_lio mapping_robosense_ac1.launch.py
```

#### Airy（32ライン）の場合：

```bash
ros2 launch fast_lio mapping_robosense_airy.launch.py
```

### 3. パラメータ設定

`config/robosense_ac1.yaml`または`config/robosense_airy.yaml`を編集：

```yaml
common:
    lid_topic: "/rs_lidar/points"  # LiDARの点群トピック
    imu_topic: "/rs_imu"           # IMUデータのトピック

preprocess:
    lidar_type: 5                  # 5 = RSM1（Robosense）
    scan_line: 5                   # 5 (AC1/E1R/M1) or 32 (Airy)
    timestamp_unit: 0              # 0=秒, 1=ミリ秒, 2=マイクロ秒, 3=ナノ秒

mapping:
    num_sub_cloud: 1               # サブクラウド分割数（1=無効、2以上で有効）
                                   # 大量の点群の場合は2-4を推奨
```

**重要な設定：**

- **lidar_type: 5** - Robosense LiDARを使用する場合は必ず`5`に設定
- **scan_line** - LiDARモデルに応じて設定
  - AC1/E1R/M1: `5`
  - Airy: `32`
- **timestamp_unit** - Robosense ROSドライバが出力するタイムスタンプの単位
  - 通常は`0`（秒単位）
- **num_sub_cloud** - サブクラウド分割数
  - `1`: 標準モード（RSM1）
  - `2`以上: サブクラウド分割モード（RSM1_BREAK）
  - 点群が多い場合や処理が重い場合は`2-4`を推奨

### 4. Robosense ROSドライバの設定

Robosense LiDARのROSドライバは以下から入手できます：

- [RoboSense/rslidar_sdk](https://github.com/RoboSense/rslidar_sdk)

ドライバの設定ファイルで、以下のように点群フォーマットを設定してください：

```yaml
msg_source: 1                    # 1: LiDAR, 5: Packet
send_point_cloud_ros: true
ros_send_point_cloud_topic: /rs_lidar/points
timestamp_type: 0                # 0: 秒単位の絶対時刻
```

## トラブルシューティング

### ビルドエラー: ikd-Tree が見つからない

```bash
cd ~/ros2_ws/src/fast_lio
git submodule update --init --recursive
```

### タイムスタンプがおかしい

- `timestamp_unit`パラメータを確認してください
- Robosenseドライバの`timestamp_type`が`0`（秒単位）になっているか確認してください

### 点群が表示されない

- トピック名が正しいか確認：
  ```bash
  ros2 topic list
  ros2 topic echo /rs_lidar/points --field header
  ```
- `lidar_type`が`5`に設定されているか確認
- `scan_line`がLiDARモデルに合っているか確認

### 処理が遅い・メモリ不足

- `num_sub_cloud`を`2`または`4`に設定してサブクラウド分割モードを有効化
- `point_filter_num`を増やして点群をダウンサンプリング

## 技術的な背景

### なぜRobosense専用の実装が必要か？

1. **タイムスタンプフォーマットの違い**
   - Velodyne: `float time`（スキャン開始からの相対時刻）
   - Ouster: `uint32_t t`（ナノ秒単位）
   - Robosense: `double timestamp`（**秒単位の絶対時刻**）

2. **データ型の違い**
   - Robosenseは`double`型で高精度なタイムスタンプを提供
   - 絶対時刻→相対時刻の変換が必要

3. **MEMS LiDARの特性**
   - 回転式LiDARと異なるスキャンパターン
   - 高速スキャンレート（10Hz標準）
   - 点群の時系列順序が重要

### ROS1からROS2への移植時の主な変更点

1. **メッセージ型の変更**
   - `sensor_msgs::PointCloud2::ConstPtr` → `sensor_msgs::msg::PointCloud2::UniquePtr`

2. **名前空間の変更**
   - `ros::Time` → `rclcpp::Time`
   - `ROS_INFO` → `RCLCPP_INFO`

3. **launchファイルの変更**
   - XML形式 → Python形式

## ライセンス

このプロジェクトは元のFAST-LIOのライセンスに従います。

## 参考リンク

- [元のFAST-LIO（ROS2版）](https://github.com/Ericsii/FAST_LIO_ROS2)
- [FAST-LIO（オリジナル・ROS1版）](https://github.com/hku-mars/FAST_LIO)
- [Robosense公式サイト](https://www.robosense.ai/)
- [Robosense ROS SDK](https://github.com/RoboSense/rslidar_sdk)

## 謝辞

- オリジナルのFAST-LIOを開発したHKU-MARSラボ
- ROS2版を移植したEricsii氏
- Robosense社のLiDAR技術
