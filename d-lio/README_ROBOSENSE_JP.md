# D-LIO Robosense LiDAR対応版

このドキュメントでは、D-LIOでRobosense社製LiDARを使用する方法について説明します。

## サポートするRobosense LiDARモデル

以下のRobosense MEMS LiDARをサポートしています：

- **RS-LiDAR-M1** - 5ラインMEMS LiDAR
- **RS-LiDAR-E1R** - 5ラインMEMS LiDAR（AC1の前モデル）
- **RS-LiDAR-AC1** - 5ラインMEMS LiDAR（最新モデル）
- **RS-LiDAR-Airy** - 96ラインMEMS LiDAR

これらのLiDARは同一の点群フォーマット（`double timestamp`フィールド付き）を使用しているため、同じ実装でサポートされます。

## Robosense対応の実装詳細

### 1. 点群データ構造

Robosense LiDARは以下の点群フォーマットを使用します：

```cpp
namespace robosenseM1_ros {
    struct Point {
        PCL_ADD_POINT4D              // x, y, z座標
        PCL_ADD_INTENSITY;           // 反射強度
        uint16_t ring;               // レーザーリング番号
        double timestamp;            // タイムスタンプ（秒単位の絶対時刻）
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}
```

**重要な特徴：**
- `timestamp`フィールドは`double`型（Ouster LiDARは`uint32_t t`を使用）
- タイムスタンプは**絶対時刻**（秒単位）として保存される
- D-LIOのデフォルトOuster形式とは異なるデータ構造のため、専用の処理が必要

### 2. LiDARタイプの選択

D-LIOでは、launchファイルでLiDARタイプを指定できます：

```python
{'lidar_type': 'robosense'},  # 'ouster' または 'robosense'
```

このパラメータに応じて、適切な点群処理関数が自動的に選択されます。

### 3. タイムスタンプ処理

Robosense LiDARの絶対時刻タイムスタンプを使用して、IMUデータとの同期を行います。

`PointCloud2_to_PointXYZ_unwrap_robosense()`関数内（dlo3d_node.cpp 1082-1175行目）：

```cpp
// Robosenseは絶対タイムスタンプを使用（秒単位のdouble型）
double point_time = pt.timestamp;
Closest_Filter_Result closest_Filter_data = findClosestFilterData(filtered_deque, point_time);

// IMUデータから最も近い姿勢情報を取得
tf2::Quaternion closest_Filter_q;
closest_Filter_q.setRPY(closest_Filter_data.Filter_data.roll,
                        closest_Filter_data.Filter_data.pitch,
                        closest_Filter_data.Filter_data.yaw);
```

**処理の流れ：**
1. 各点のタイムスタンプ（`timestamp`）を直接使用
2. IMUフィルタキューから最も近い時刻のIMUデータを検索
3. その時刻のIMU姿勢を使用して点群のdeskewingを実施
4. レンジフィルタリングとダウンサンプリングを適用

### 4. 主な変更ファイル

#### `src/dlo3d_node.cpp`

**点群構造体の追加**（66-85行目）：
```cpp
namespace robosenseM1_ros {
    struct Point {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY;
        uint16_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}
```

**LiDARタイプパラメータの追加**（107行目、271行目）：
```cpp
m_lidarType = this->declare_parameter<std::string>("lidar_type", "ouster");
std::string m_lidarType;  // メンバー変数
```

**点群処理の分岐**（617-694行目）：
```cpp
if (m_lidarType == "robosense") {
    // Robosense LiDAR処理
    pcl::PointCloud<robosenseM1_ros::Point> pcl_cloud_rs;
    pcl::fromROSMsg(*cloud, pcl_cloud_rs);
    // ... Robosense専用処理
} else {
    // Ouster/デフォルトLiDAR処理
    pcl::PointCloud<PointXYZT> pcl_cloud;
    pcl::fromROSMsg(*cloud, pcl_cloud);
    // ... Ouster処理
}
```

**Robosense用アンラップ関数**（1081-1175行目）：
```cpp
bool DLO3DNode::PointCloud2_to_PointXYZ_unwrap_robosense(
    pcl::PointCloud<robosenseM1_ros::Point> &in,
    std::vector<pcl::PointXYZ> &out,
    double scan_end_time)
```

#### `launch/dlo3d_robosense_launch.py`
- Robosense用のlaunchファイル
- `lidar_type: 'robosense'`に設定
- 適切なトピック名（`/rslidar_points`など）に設定

## 使用方法

### 1. ビルド

```bash
cd ~/ros2_ws/src
git clone <このリポジトリのURL> D-LIO
cd ~/ros2_ws
colcon build --packages-select D-LIO
source install/setup.bash
```

### 2. 起動

#### Robosense LiDARの場合：

```bash
ros2 launch D-LIO dlo3d_robosense_launch.py
```

#### Ouster LiDAR（デフォルト）の場合：

```bash
ros2 launch D-LIO dlo3d_launch.py
```

### 3. パラメータ設定

`launch/dlo3d_robosense_launch.py`を編集：

```python
parameters=[
    {'lidar_type': 'robosense'},        # 'ouster' または 'robosense'
    {'in_cloud': '/rslidar_points'},    # LiDARの点群トピック
    {'in_imu': '/imu/data'},            # IMUデータのトピック
    {'hz_cloud': 10.0},                 # LiDAR周波数（Hz）
    {'hz_imu': 100.0},                  # IMU周波数（Hz）
    {'min_range': 1.0},                 # 最小レンジ（m）
    {'max_range': 100.0},               # 最大レンジ（m）
    {'pc_downsampling': 1},             # ダウンサンプリング係数
    # ... その他のパラメータ
]
```

**重要な設定：**

- **lidar_type**: `'robosense'`に設定（Robosense LiDARを使用する場合）
- **in_cloud**: Robosense ROSドライバが発行する点群トピック名
- **in_imu**: IMUデータのトピック名
- **hz_cloud**: LiDARの周波数（通常10Hz）
- **pc_downsampling**: 点群のダウンサンプリング係数（1=全点使用、2=半分、など）

### 4. Robosense ROSドライバの設定

Robosense LiDARのROSドライバは以下から入手できます：

- [RoboSense/rslidar_sdk](https://github.com/RoboSense/rslidar_sdk)

ドライバの設定ファイルで、以下のように点群フォーマットを設定してください：

```yaml
msg_source: 1                       # 1: LiDAR, 5: Packet
send_point_cloud_ros: true
ros_send_point_cloud_topic: /rslidar_points
timestamp_type: 0                   # 0: 秒単位の絶対時刻
```

**重要:** `timestamp_type: 0`に設定して、秒単位の絶対時刻タイムスタンプを使用してください。

## トラブルシューティング

### 点群が表示されない

- トピック名が正しいか確認：
  ```bash
  ros2 topic list
  ros2 topic echo /rslidar_points --field header
  ```
- `lidar_type`が`'robosense'`に設定されているか確認
- TFツリーが正しく設定されているか確認（`rslidar`フレームと`base_link`フレーム間）

### タイムスタンプエラー

- Robosenseドライバの`timestamp_type`が`0`（秒単位）になっているか確認
- IMUとLiDARのタイムスタンプが同期されているか確認
- ログで"IMU queue is empty"や"No data found in the specified time range"が出ていないか確認

### deskewing（補正）が機能しない

- IMUデータが正しく受信されているか確認：
  ```bash
  ros2 topic echo /imu/data
  ```
- IMUとLiDARのタイムスタンプが同じ時刻基準を使用しているか確認
- キャリブレーション時間（`calibration_time`）が適切に設定されているか確認

### 処理が遅い・メモリ不足

- `pc_downsampling`を増やして点群をダウンサンプリング（例：2または4）
- `max_range`を減らして処理する点群の範囲を制限
- `solver_max_threads`を調整してCPU使用率を最適化

## 技術的な背景

### なぜRobosense専用の実装が必要か？

1. **タイムスタンプフォーマットの違い**
   - Ouster: `uint32_t t`（ナノ秒単位の相対時刻）
   - Robosense: `double timestamp`（**秒単位の絶対時刻**）

2. **データ型の違い**
   - Robosenseは`double`型で高精度なタイムスタンプを提供
   - 絶対時刻を直接使用してIMUデータと同期

3. **MEMS LiDARの特性**
   - 回転式LiDARと異なるスキャンパターン
   - 高速スキャンレート（10Hz標準）
   - 点群の時系列順序が重要

### D-LIOとの統合

D-LIOは元々Ouster LiDAR向けに設計されていましたが、Robosense対応により：

1. **柔軟なLiDAR選択**: `lidar_type`パラメータで簡単に切り替え可能
2. **既存機能の維持**: Ouster LiDARの動作は変更なし
3. **統一されたインターフェース**: 同じノード、同じパラメータ構造

## 参考リンク

- [D-LIO（オリジナル）](https://github.com/robotics-upo/D-LIO)
- [Robosense公式サイト](https://www.robosense.ai/)
- [Robosense ROS SDK](https://github.com/RoboSense/rslidar_sdk)

## 謝辞

- オリジナルのD-LIOを開発したrobótics-UPOチーム
- Robosense社のLiDAR技術
- ROS2コミュニティのサポート
