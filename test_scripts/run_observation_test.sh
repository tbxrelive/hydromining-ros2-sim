#!/bin/bash
# 阶段 3 完整测试脚本：观测与配准
# 使用方法：
#   终端 1: ./run_observation_test.sh sim      # 启动仿真
#   终端 2: ./run_observation_test.sh capture  # 启动点云采集
#   终端 3: ./run_observation_test.sh test     # 运行观测测试
#   终端 4: ./run_observation_test.sh verify   # 验证配准（需要先安装 Open3D）

WS_DIR=~/SCI/hydromining_ws
DATA_DIR=$WS_DIR/src/hydromining_robot/test/data

cd $WS_DIR

case "$1" in
  sim)
    echo "=========================================="
    echo "  启动仿真环境"
    echo "=========================================="
    source install/setup.bash
    ros2 launch hydromining_robot robot_simulation.launch.py
    ;;

  capture)
    echo "=========================================="
    echo "  启动点云采集节点"
    echo "=========================================="
    # 确保数据目录存在
    mkdir -p $DATA_DIR
    source install/setup.bash
    ros2 run hydromining_robot pointcloud_capture.py --ros-args \
      -p output_dir:=$DATA_DIR
    ;;

  test)
    echo "=========================================="
    echo "  运行多视角观测测试"
    echo "=========================================="
    source install/setup.bash
    ros2 run hydromining_robot test_observation.py
    ;;

  verify)
    echo "=========================================="
    echo "  验证点云配准质量"
    echo "=========================================="

    # 检查数据文件
    PCD_COUNT=$(find $DATA_DIR -name "*.pcd" 2>/dev/null | wc -l)
    if [ "$PCD_COUNT" -lt 2 ]; then
      echo "❌ 错误：至少需要 2 个点云文件，但只找到 $PCD_COUNT 个"
      echo ""
      echo "请先运行："
      echo "  终端 1: ./run_observation_test.sh sim"
      echo "  终端 2: ./run_observation_test.sh capture"
      echo "  终端 3: ./run_observation_test.sh test"
      exit 1
    fi

    # 检查 Open3D
    if ! python3 -c "import open3d" 2>/dev/null; then
      echo "❌ 错误：需要安装 Open3D"
      echo ""
      echo "安装方法 1（虚拟环境，推荐）："
      echo "  python3 -m venv venv_test"
      echo "  source venv_test/bin/activate"
      echo "  pip install open3d numpy"
      echo "  python3 src/hydromining_robot/test/scripts/verify_registration.py $DATA_DIR"
      echo "  deactivate"
      echo ""
      echo "安装方法 2（系统级，有风险）："
      echo "  pip install open3d --break-system-packages"
      exit 1
    fi

    # 运行验证
    python3 src/hydromining_robot/test/scripts/verify_registration.py $DATA_DIR
    ;;

  clean)
    echo "清理点云数据文件..."
    rm -f $DATA_DIR/*.pcd
    echo "✅ 已清理"
    ;;

  *)
    echo "用法: $0 {sim|capture|test|verify|clean}"
    echo ""
    echo "完整测试流程："
    echo "  1. 终端 1: $0 sim      # 启动仿真"
    echo "  2. 终端 2: $0 capture  # 启动点云采集"
    echo "  3. 终端 3: $0 test     # 运行观测测试"
    echo "  4. 终端 4: $0 verify   # 验证配准"
    echo ""
    echo "其他命令："
    echo "  $0 clean  # 清理点云数据"
    exit 1
    ;;
esac
