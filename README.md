# RGB-D Grasp Estimator

物体の検出領域とaligned depthから、クラスタ・重心・法線・把持候補姿勢を求めます。
PyLoTのbottle/bowl cluster grasp処理を共通化し、物体検出器とLuna armへの依存を
外しました。物体検出そのものは外部に任せ、bboxまたはPython APIのmaskを入力します。

## Geometry pipeline

1. bbox/mask内の有効深度を、カメラ内部パラメータから3D点へ変換。
2. 空間グリッドとBFSで距離に基づいてクラスタリング。
3. 最大の有効クラスタを選択し、共分散の最小固有ベクトルを法線候補にする。
4. 法線をカメラ側へ向け、指定offsetを加えた位置とapproach姿勢を出力。

点は決定的に間引きます。最大点数、クラスタ距離、点数下限、平面らしさを設定できます。
点不足・無効深度・直線状の退化・平面らしさ不足の場合は候補なしを返します。
最大クラスタが目的物体である保証は無く、背景を含めないROI/maskが重要です。

## Offline use

```bash
rgbd-grasp-estimator estimate depth.npy --unit mm --roi 10 10 100 100 \
  --intrinsics 600 600 320 240 --output candidate.json
```

深度は2次元の `.npy`、bboxは `[x1,y1,x2,y2]` で右下端を含みません。
unitはm/mmを明示します。fx/fy/cx/cyはpixel単位です。出力座標はカメラ光学座標のm、
quaternionはxyzwです。local +Zが法線と逆向きのapproach方向を向きます。
Pythonの `rgbd_grasp.core.estimate` は画像サイズのboolean maskも受け取れます。
CLIの終了コードは候補あり0、候補なし1、入力不正2です。

## ROS interface

aligned depthとCameraInfoを配信し、`/rgbd_grasp/roi` にString JSONを送ります。

```json
{"bbox":[10,10,100,100],"stamp":123.4,"frame_id":"camera_color_optical_frame"}
```

stampは対応する画像のROS時刻です。深度・ROI・CameraInfoは同じ光学フレームと画素系を
必要とします。CameraInfoと深度の寸法一致、深度の鮮度、ROIとの時刻差を確認します。
カメラ内部パラメータは配信されたCameraInfoを使用し、固定の画角で代用しません。
16UC1はmm、32FC1はmとして処理します。

- `/rgbd_grasp/pose` (`PoseStamped`): 候補がある場合のみ出力。headerは深度から保持。
- `/rgbd_grasp/result` (`String` JSON): valid、位置・法線・姿勢・点数、または失敗理由。

入力が不正・古い・未同期のときに前回の姿勢を再配信しません。入力断でposeトピックの
過去メッセージが消えるわけではないため、利用側でもheader.stampを確認してください。

合成平面で単位変換・法線・姿勢・サブサンプリングを確認し、ROS結合テストで
深度→Poseと未同期入力の拒否を確認します。把持の成功、衝突回避、到達性や実アームの
動作は評価しません。このパッケージはアームへの指令を出しません。

## Installation and checks

Python 3.10+。リポジトリのルートで実行します。通常のPython環境ではROSは不要です。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m rgbd_grasp.cli demo
```

ROS 2 Jazzyで使う場合は通常のワークスペースのsrcに配置し、ROS環境でビルドします。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rgbd_grasp
source install/setup.bash
```

`config/params.yaml` を自分の入出力に合わせて変更してください。パラメータは起動時設定です。
`ros2 launch rgbd_grasp rgbd_grasp.launch.py params_file:=/absolute/path/to/params.yaml` で起動できます。
CIはPython 3.10/3.12でコアテストと合成デモを実行します。ROS結合テストはROS環境で
実行し、それ以外ではスキップします。モデル・実センサ・実アームの性能は認証しません。

## License and source

現在はprivateリポジトリです。Apache-2.0。元のコミット・ファイルと変更範囲は
[PROVENANCE.md](PROVENANCE.md) を参照してください。元モノレポへの依存はありません。
