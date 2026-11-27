#!/usr/bin/env bash
# 比赛启动脚本（规则 6.1：启动命令为 ./start.sh）。
#
#   ./start.sh            打开左右两个终端：左边 1 号狗，右边 2 号狗（规则 6.2）
#   ./start.sh 1          只在当前终端启动 1 号狗（被抱起后裁判重新启动用）
#   ./start.sh 2 --dry-run   预览 2 号狗决策，不发运动指令
#
# 其余参数原样传给 run_dog.py：--side left|right、--kickoff us|them、--role striker|defender

# ---------------- 现场按需修改 ----------------
RUN_ON="${RUN_ON:-local}"                         # local：本机运行；ssh：登录到狗上运行
DOG1_HOST="${DOG1_HOST:-mi@192.168.1.101}"        # ssh 模式下 1 号狗地址
DOG2_HOST="${DOG2_HOST:-mi@192.168.1.102}"        # ssh 模式下 2 号狗地址
REMOTE_DIR="${REMOTE_DIR:-~/CYBERDOG_FINAL}"      # ssh 模式下代码在狗上的位置
ROS_SETUP="${ROS_SETUP:-/opt/ros2/cyberdog/local_setup.bash}"
PYTHON="${PYTHON:-/usr/bin/python3}"
# ---------------------------------------------

cd "$(dirname "$0")" || exit 1

if [ $# -eq 0 ]; then
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="1号狗" --geometry=95x45+0+0 -- bash -c "./start.sh 1; exec bash"
        gnome-terminal --title="2号狗" --geometry=95x45-0+0 -- bash -c "./start.sh 2; exec bash"
    elif command -v xterm >/dev/null 2>&1; then
        xterm -T "1号狗" -geometry 95x45+0+0 -e bash -c "./start.sh 1; exec bash" &
        xterm -T "2号狗" -geometry 95x45-0+0 -e bash -c "./start.sh 2; exec bash" &
    else
        echo "找不到 gnome-terminal / xterm。请手动开两个终端："
        echo "  左边：./start.sh 1"
        echo "  右边：./start.sh 2"
    fi
    exit 0
fi

DOG="$1"
shift
if [ "$DOG" != "1" ] && [ "$DOG" != "2" ]; then
    echo "用法：./start.sh [1|2] [run_dog.py 参数]"
    exit 1
fi

if [ "$RUN_ON" = "ssh" ]; then
    HOST="$DOG1_HOST"
    [ "$DOG" = "2" ] && HOST="$DOG2_HOST"
    echo "登录 $HOST 启动 $DOG 号狗"
    exec ssh -t "$HOST" "cd $REMOTE_DIR && RUN_ON=local ./start.sh $DOG $*"
fi

if [ -f "$ROS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
else
    echo "警告：找不到 $ROS_SETUP，假定当前终端已加载 ROS2 环境"
fi

echo "==== $DOG 号狗 ===="
exec "$PYTHON" run_dog.py --dog "$DOG" "$@"
