"""K230 H题摆杆控制：OpenCV传统视觉 + UART4任务控制/任务3调参 + 任务6板载按键标定 + 自动回位 + RTSP H.264无线图传。

图像通道分工：
- CAM_CHN_ID_0：800×480灰度图，供OpenCV检测、LCD显示和CanMV IDE预览共用；
- CAM_CHN_ID_1：原彩YUV420SP，只裁取ROI范围，直接绑定VENC编码并通过RTSP发送。

无线图传：
- WIRELESS_STREAM_ENABLE = 0：完全关闭Wi-Fi、VENC和RTSP；
- WIRELESS_STREAM_ENABLE = 1：连接2.4GHz热点并开启H.264 RTSP图传；
- 图传仅包含ROI原彩画面，不包含LCD上绘制的检测框和文字。

本地任务：只修改K230_TEST_MODE = 3/4/5/6即可切换H题对应调试模式。

安全退出：
- 每帧检查os.exitpoint()；
- to_numpy_ref()引用用完立即清空；
- RTSP采用非阻塞轮询，避免无线网络拖住视觉控制；
- 退出时依次停止sensor、VENC、媒体绑定、RTSP、Display和MediaManager。
"""

from media.sensor import *
from media.display import *
from media.media import *
from media.vencoder import *
from machine import UART, FPIOA, Pin

import cv2
from ulab import numpy as np
import time
import os
import gc
import math
import sys
import network
import uctypes
import multimedia as mm


# ============================== 单灰度图像通道 ==============================
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
SENSOR_ID = 2
VISION_CHANNEL = CAM_CHN_ID_0

SENSOR_HMIRROR = False
SENSOR_VFLIP = False

# LCD和CanMV IDE同时显示同一张灰度帧。
ENABLE_IDE_PREVIEW = True
DISPLAY_INTERVAL = 1

# 控制器内部继续使用较小的坐标系，避免原控制参数整体放大2.5倍。
CONTROL_WIDTH = 320
CONTROL_HEIGHT = 192
AI_WIDTH = CONTROL_WIDTH
AI_HEIGHT = CONTROL_HEIGHT
FULL_TO_CONTROL_X = CONTROL_WIDTH / float(DISPLAY_WIDTH)
FULL_TO_CONTROL_Y = CONTROL_HEIGHT / float(DISPLAY_HEIGHT)
CONTROL_TO_FULL_X = DISPLAY_WIDTH / float(CONTROL_WIDTH)
CONTROL_TO_FULL_Y = DISPLAY_HEIGHT / float(CONTROL_HEIGHT)

# 800×480画面中的实测ROI。
ROI_X = 10
ROI_Y = 280
ROI_W = 726
ROI_H = 90
ROD_CENTER_Y = 325
MAX_Y_ERROR = 34
ROI_END_MARGIN = 15

# ============================== RTSP H.264无线图传 ==============================
# 0=关闭；1=开启。关闭时不会初始化Wi-Fi、第二摄像头通道、VENC或RTSP。
WIRELESS_STREAM_ENABLE = 0

# 连接手机热点/路由器，建议只使用2.4GHz。电脑和K230必须处于同一局域网。
WIFI_SSID = "12345"
WIFI_PASSWORD = "12345678"

RTSP_PORT = 8554
RTSP_SESSION = "live"
STREAM_CHANNEL = CAM_CHN_ID_1

# 通道1发送完整800×480原彩画面，不做ROI裁剪。
# 800可以被16整除，480为偶数，满足H.264与YUV420尺寸要求。
STREAM_WIDTH = 800
STREAM_HEIGHT = 480

# H.264质量/流畅度参数。800×480建议从2000Kbit/s、20FPS开始。
VIDEO_BITRATE_KBPS = 2000
VIDEO_OUTPUT_FPS = 20
VIDEO_SOURCE_FPS = 60
VIDEO_GOP = 20
VENC_BUFFER_NUM = 6
RTSP_STATS_PERIOD_MS = 2000

# 非阻塞读取VENC码流。必须保持0，不能使用-1无限等待，否则会拖慢PID循环。
RTSP_GETSTREAM_TIMEOUT_MS = 0

# 三点标定，均为800×480完整画面坐标。
CAL_LEFT_FULL_PX = 33.10
CAL_CENTER_FULL_PX = 353.64
CAL_RIGHT_FULL_PX = 690.92
CAL_LEFT_CM = -12.0
CAL_CENTER_CM = 0.0
CAL_RIGHT_CM = 12.0
DISPLAY_SCALE_MARKS_CM = (-5.0, 0.0, 5.0)

# 传统视觉阈值和形态学。
DARK_THRESHOLD_MODE = "FIXED"
ENABLE_DARK_CONTOUR = True
ENABLE_BRIGHT_CONTOUR = False
DARK_MAX = 92
BRIGHT_MIN = 210
MEDIAN_BLUR_KSIZE = 3
OPEN_KERNEL_SIZE = 3
CLOSE_KERNEL_SIZE = 5
MORPH_OPEN_ITERATIONS = 1
MORPH_CLOSE_ITERATIONS = 1
ADAPTIVE_BLOCK_SIZE = 31
ADAPTIVE_C = 7

# 轮廓筛选。
MIN_BALL_W = 6
MAX_BALL_W = 48
MIN_BALL_H = 6
MAX_BALL_H = 48
MIN_CONTOUR_AREA = 25.0
MAX_CONTOUR_AREA = 1200.0
MIN_BBOX_AREA = 35
MAX_BBOX_AREA = 1800
MIN_ASPECT = 0.45
MAX_ASPECT = 2.20
MIN_FILL_RATIO = 0.22
MAX_FILL_RATIO = 1.00
MIN_CIRCULARITY = 0.18
EXPECTED_DIAMETER_PX = 19.0
MOMENT_CENTER_WEIGHT = 0.55

# 丢球后霍夫圆只间隔调用，避免连续阻塞主循环和停止请求。
ENABLE_HOUGH_REACQUIRE = True
HOUGH_START_AFTER_MISSES = 3
HOUGH_REACQUIRE_INTERVAL = 5
HOUGH_DP = 1.0
HOUGH_MIN_DIST = 20.0
HOUGH_CANNY_HIGH = 75.0
HOUGH_ACC_THRESHOLD = 18.0
HOUGH_MIN_RADIUS = 5
HOUGH_MAX_RADIUS = 28

# 候选评分与显示。
MAX_TRACK_JUMP_PX = 85
MERGE_CENTER_DISTANCE = 16
MIN_CANDIDATE_QUALITY = 55.0
SHOW_ALL_CANDIDATES = False
DRAW_TRACK = True
TRACK_POINT_COUNT = 10

# 传统视觉每帧测量；短时漏检继续使用Kalman预测。
PREDICTION_HOLD_MS = 125
BALL_LOST_RESET_MS = 360
CAMERA_PIPELINE_DELAY_MS = 8
LATENCY_COMPENSATION_GAIN = 1.00
MOTOR_RESPONSE_DELAY_MS = 6.0
CONTROL_LOOKAHEAD_MIN_MS = 5.0
CONTROL_LOOKAHEAD_MAX_MS = 12.0
MAX_PREDICTION_MS = 110.0
BALL_ACCEL_LIMIT_PX_S2 = 1000.0

# Kalman原代码仍按0~1置信度工作；传统视觉质量分会除以100传入。
TRACK_CONFIDENCE_THRESHOLD = 0.10
LOW_CONFIDENCE_FORCE_DETECT = 0.22
KALMAN_MEASUREMENT_STD_PX = 2.20
KALMAN_PROCESS_JERK_STD_PX_S3 = 1850.0
KALMAN_LOW_CONFIDENCE_R_SCALE = 4.0
KALMAN_FAST_MOTION_R_REDUCTION = 0.45
KALMAN_VELOCITY_INNOVATION_GAIN = 0.0

# 调试输出与垃圾回收。
TERMINAL_INTERVAL_MS = 1000
GC_INTERVAL_MS = 8000

# 灰度绘图颜色。
ROI_COLOR = 185
SCALE_COLOR = 205
CENTER_COLOR = 255
TARGET_COLOR = 235
BALL_COLOR = 255
PREDICT_COLOR = 150
TEXT_COLOR = 255

# ============================== 任务6：板载用户按键标定 ==============================
# 立创·庐山派K230-CanMV板载用户按键：GPIO53，下拉输入，按下为高电平。
# 若使用Lite-K230D，应将TASK6_BUTTON_GPIO改为64。
TASK6_BUTTON_GPIO = 53
TASK6_BUTTON_PRESS_LEVEL = 1
TASK6_BUTTON_DEBOUNCE_MS = 30

# 任务6标定位置限制在三点标定量程内，并在两端保留安全余量。
TASK6_SAFE_MARGIN_CM = 0.80
TASK6_MIN_CM = CAL_LEFT_CM + TASK6_SAFE_MARGIN_CM
TASK6_MAX_CM = CAL_RIGHT_CM - TASK6_SAFE_MARGIN_CM

# 任务6专用显示颜色。位置文字贴在钢球检测框上方。
TASK6_WAIT_COLOR = 210
TASK6_LOCK_COLOR = 255
TASK6_ERROR_COLOR = 150
TASK6_LABEL_FONT_SIZE = 18
TASK6_LABEL_HEIGHT = 24

# ============================== 电机：UART2 / Emm FD ==============================
UART2_TX_IO = 11
UART2_RX_IO = 12
UART_BAUDRATE = 115200
MOTOR_ID = 0x01
MOTOR_POSITIVE_DIRECTION = 0

MOTOR_STEP_ANGLE_DEG = 1.8
MOTOR_MICROSTEP = 16
MOTOR_DEG_PER_PULSE = MOTOR_STEP_ANGLE_DEG / MOTOR_MICROSTEP
# ---------- 连杆几何: 把"电机度"翻译成球感受到的加速度 ----------
# 曲柄连杆: 电机转 theta -> 连杆端位移 R*sin(theta) -> 杆倾角 -> 球加速度。
# 小角度下 杆倾度/电机度 = R / L。
# 【为什么必须算这个】下面所有角度上限原来都是直接写"多少电机度", 那个数
# 换个曲柄半径就完全变味。翻译成 cm/s2 才能和球的实际阻力(2~4)、项3 需要
# 的制动量(18)对上号。
CRANK_RADIUS_MM = 47.5          # 电机轴中心 -> 曲柄销
PIVOT_TO_RACK_MM = 280.0        # 摆杆支点 -> 连杆上端铰点
BEAM_DEG_PER_MOTOR_DEG = CRANK_RADIUS_MM / PIVOT_TO_RACK_MM
BALL_ACCEL_PER_BEAM_DEG = (5.0 / 7.0) * 981.0 * math.pi / 180.0  # 12.2


def accel_to_motor_deg(accel_cm_s2):
    """球加速度(cm/s2) -> 需要的电机角(度)"""
    return accel_cm_s2 / BALL_ACCEL_PER_BEAM_DEG / max(
        BEAM_DEG_PER_MOTOR_DEG, 1e-6
    )


MOTOR_ANGLE_LIMIT_DEG = 20.0
MOTOR_MAX_PULSES = int(MOTOR_ANGLE_LIMIT_DEG / MOTOR_DEG_PER_PULSE)
MOTOR_SOFT_LIMIT_DEG = MOTOR_MAX_PULSES * MOTOR_DEG_PER_PULSE

# 柔和模式：保持电机硬限位 ±20°，只降低到位转速、角度建立速度和换向冲击。
# 若动作仍偏猛，优先继续降低 MOTOR_ANGLE_STEP_NORMAL_DEG，
# 不要先降低 ±20°硬限位，否则可能出现远端推不回来的情况。
# FD绝对位置命令速度动态调度：中心附近柔和，普通状态快速，
# 惯性外冲或提前制动时使用更高到位速度。角度仍由软件绝对位置限位。
MOTOR_POSITION_SPEED_NEAR_RPM = 42
MOTOR_POSITION_SPEED_NORMAL_RPM = 72
MOTOR_POSITION_SPEED_BRAKE_RPM = 95
MOTOR_POSITION_SPEED_DISTURBANCE_RPM = 115
MOTOR_ACCELERATION = 20

# 非保持阶段每个控制周期最多改变16个微步，限制目标跳变。
MOTOR_MAX_ANGLE_STEP_DEG = 1.200

# 小角度若不足一个微步会被量化为0；偏离中心时至少给8个微步克服机械静摩擦。
MOTOR_MIN_ACTIVE_PULSES = 10
MOTOR_MIN_ACTIVE_ANGLE_DEG = MOTOR_MIN_ACTIVE_PULSES * MOTOR_DEG_PER_PULSE
MOTOR_ACTIVE_POSITION_ERROR_PX = 0.9
MOTOR_ACTIVE_VELOCITY_ERROR_PX_S = 3.0

# 即使命令脉冲未变化，也周期性重发非零目标，避免单次串口命令丢失后一直不动作。
MOTOR_COMMAND_MIN_INTERVAL_MS = 28
MOTOR_COMMAND_REFRESH_MS = 140
MOTOR_COMMAND_PULSE_HYSTERESIS = 3
MOTOR_COMMAND_NEAR_CENTER_HYSTERESIS = 2
MOTOR_NEAR_CENTER_POSITION_PX = 6.0
CONTROL_SIGN = 1.0


# ============================== UART4：3507 / 任务3调参 ==============================
# UART2已供步进电机使用；UART4固定115200 8N1，必须共地：
#   K230 GPIO48 / UART4_TXD -> 3507 PB3 / UART3_RX
#   K230 GPIO49 / UART4_RXD <- 3507 PB2 / UART3_TX
#
# 调参模式时，GPIO48(TX)->USB串口RX，GPIO49(RX)<-USB串口TX。
# 正式比赛使用3507联机模式：GPIO48/49接3507串口，115200 8N1。
# 任务3调参已结束，保留定型控制参数，不再上电自动运行或占用UART4调参。
UART4_MODE_3507 = 0
UART4_MODE_TASK3_TUNE = 1
UART4_MODE = UART4_MODE_3507
LINK_ENABLE = UART4_MODE == UART4_MODE_3507
TASK3_UART_TUNE_ENABLE = UART4_MODE == UART4_MODE_TASK3_TUNE
LINK_UART_TX_IO = 48
LINK_UART_RX_IO = 49
LINK_BAUDRATE = 115200
TASK3_TUNE_TELEMETRY_MS = 100

# 当前任务结束、手动停止或收到其他任务指令时，先回到本任务开始时的电机位置。
TASK_RETURN_SPEED_RPM = 60
TASK_RETURN_MIN_WAIT_MS = 300
TASK_RETURN_EXTRA_WAIT_MS = 260
TASK_RETURN_MAX_WAIT_MS = 1200

# 上行帧 [±XXXX*]，单位 0.1mm，定义见 3507 的 k230_ball.h。
# 30ms 一帧约 33Hz，远高于 3507 那边 100ms 的链路超时门限，丢两帧也不会误报。
LINK_POSITION_INTERVAL_MS = 30
LINK_LOST_MARK = 9999
LINK_DONE_MARK = 9998          # 项3 序列跑完的哨兵，3507 收到就停表
LINK_DONE_REPEAT = 3           # 连发几次，防止单帧被干扰吃掉
LINK_RX_MAX_PAYLOAD = 48       # 同时容纳 [SET,参数,数值] 调参帧

# 3507起步前馈下行协议：
#   [FC,seq,lead_ms,ramp_ms,gain_x100,damp_x100,max_x10,sign]
#   [FA,seq,ff_x10,reference_accel,measured_accel]
#   [FE,seq]
# ff_x10已经由3507按车辆纵向轮速曲线计算并带好正负号，K230不再读取或
# 猜测车体IMU的X/Y轴，只把它作为摆杆目标角的小幅外部前馈量。
CAR_START_FF_ENABLE = True
CAR_START_FF_SAMPLE_TIMEOUT_MS = 160
CAR_START_FF_ATTACK_DEG_PER_S = 90.0
CAR_START_FF_RELEASE_DEG_PER_S = 45.0
CAR_START_FF_ZERO_EPS_DEG = 0.02
# 3507结束起步前馈时，钢球通常还没有回到任务目标点。直接把附加角归零会
# 让位置环一次性接管剩余误差，容易从“后移”穿过中心变成“前移”。下面的
# 短暂释放段只沿原前馈方向保留一个小角度，并按钢球回中速度提前卸载。
CAR_START_FF_RECOVERY_MS = 700
CAR_START_FF_RECOVERY_KP_DEG_PER_PX = 0.070
CAR_START_FF_RECOVERY_KD_DEG_PER_PX_S = 0.010
CAR_START_FF_RECOVERY_MAX_DEG = 1.00
CAR_START_FF_RECOVERY_RELEASE_PX = 2.5


# ============================== 像素 <-> 厘米 三点标定 ==============================
# 控制器内部使用320×192坐标；这里把800×480实测标定点同步缩放。
BALL_X_SIGN = 1.0
BEAM_VISIBLE_LENGTH_CM = 24.0
BEAM_HALF_RANGE_CM = 12.0

CAL_LEFT_PX = CAL_LEFT_FULL_PX * FULL_TO_CONTROL_X
CAL_CENTER_PX = CAL_CENTER_FULL_PX * FULL_TO_CONTROL_X
CAL_RIGHT_PX = CAL_RIGHT_FULL_PX * FULL_TO_CONTROL_X

# ============================== 项3：摆球序列 ==============================
# 题目要求3：小车静止，钢球从中心 O 出发移动 5cm，折返后到另一侧 5cm 并稳定，
# 限时 5s。3507 只发一条 [T3] 并计时，序列本身整个在这里跑 —— 位置数据本来
# 就在 K230 手里，判到位不必再绕一趟串口。
SEQ3_STROKE_CM = 5.0
SEQ3_FIRST_DIR = 1.0            # 先去 +x 侧；要先去 -x 侧就改成 -1.0
# 第二段存在约0.3cm可重复的静摩擦欠行程。控制器内部目标沿最终运动方向
# 多给这一点距离，但任务判定、串口显示和完成后的保持目标仍是理论 -5cm。
SEQ3_LEG2_TARGET_EXTRA_CM = 0.00

# 中间点只要求"到过"，不要求停住 —— 停一下再折返是白扔时间，5s 很紧。
# 判据用带符号进度，越过目标点同样算到达，不会因为回不到精确值卡在那儿。
# 【0.8 会让第一段少走 0.8cm】球到 4.2cm 判据就说"到过"立刻掉头, 而题目
# 要的是真的走到 5cm。收到 0.15, 剩下那点由切换瞬间的余速带过去。
# 现在收到 0.05: 球读到 4.95cm 以上就算"到过"。主机上 120 组随机条件扫过
# (配合当时的 6 帧确认):
#   确认帧/容差   成功率   一段行程最少/中位
#     4 / 0.15     88%      4.78 / 4.98
#     5 / 0.15     93%      4.79 / 5.02
#     6 / 0.15     98%      4.91 / 5.06
#     6 / 0.05     99%      4.99 / 5.15
#     6 / 0.25     84%      4.79 / 4.97
# 确认帧后来收回 1(见下), 容差保持 0.05 —— 少了确认帧的滤波, 容差就是唯一
# 顶住噪声的东西, 越小越贴 5cm。
SEQ3_REACH_TOL_CM = 0.05

# 【折返前要连续多少帧读到过线】判据吃的是 Kalman 估计位置, 而识别噪声
# 有 ±2~3.5px = ±0.16~0.28cm。球真实只到 4.80cm 时, 估计值偶尔跳到 4.90
# 就会判"到过", 实际差 0.2cm —— 多要几帧确认能把这种误判滤掉。
# 但确认要等的这段时间球一直在往前滚, 滤得越狠冲过量越大, 是个对赌:
# 1 = 估计位置一过线就折返(差一点点也认了); 帧数越多越保证走满 5cm, 代价
# 是等确认的这段时间球一直在往前, 冲过量跟着涨。
#   帧数   一段行程 最少/中位   冲过 中位/最多
#     1      4.63 / 4.80        ~0    / 0.05
#     4      4.78 / 4.98        0.05  / 0.19
#     6      4.97 / 5.12        0.13  / 0.31
# 【取 1】题目第一段只要求"移动 5cm", 差一两毫米不扣分, 而冲过看着更糟;
# 真正计分的是最后稳定在另一侧 5cm 处。要严格走满就改回 6。
SEQ3_REACH_CONFIRM = 1

# 【明显过线就别再等确认了】连续确认是为了防噪声误判, 但球已经实打实
# 越过 5cm 这么多时, 再等 6 帧(120ms)纯粹是让它继续往前冲。超过这个量
# 就立刻折返, 确认帧只用来盯"刚好压线"那个模糊区间。
# 主机上 120 组随机条件扫过(既要走够 5cm, 又别冲太多):
#   门槛      成功率   一段最少/中位   冲过中位/最多
#   关掉       99%     4.99 / 5.15     0.16 / 0.44
#   +0.20cm   100%     4.97 / 5.12     0.12 / 0.31   <-- 取这组
#   +0.10cm    98%     4.81 / 5.05     0.07 / 0.19
#   +0.05cm    92%     4.81 / 5.01     0.05 / 0.19
# 门槛设太小反而更差: 噪声会让球提前被判"确凿越线", 实际还没走到。
SEQ3_REACH_SURE_CM = 0.20

# ==================== 停住即折返 ====================
# 【为什么可以这么干】题目对项3 判的是误差 ≤1cm。球推不动停在 4.2cm 时,
# 离 5cm 还差 0.8cm —— 已经在容差里了。这时候再纠结"到底压没压到线"毫无
# 意义: 它推不动, 再等也不会前进, 等下去只是白扔第二段的时间预算。
# 所以判据从"到没到线"换成"还动不动": 只要离目标已经进了容差, 且确认球
# 不再前进, 立刻折返。
#
# 【难点是分清"停住"和"慢慢在爬"】球以 3cm/s 蠕动时, 短窗口里的位移被
# ±3px(0.24cm) 噪声完全淹没, 看着就像没动。用 Kalman 速度更不行 —— 球
# 静止、框抖 ±2px 时速度估计峰值就有 64px/s(5cm/s), 信噪比还不到 1。
#
# 【做法】取窗口内前半段和后半段的位置【均值】之差, 除以两个半窗中心的
# 时间间隔, 得到一个平均过的速度。各 7~8 帧一平均, 单帧噪声被开方压下去。
# 主机上把球钉死不动、只喂识别噪声, 实测这个估计量的分布:
#   噪声 ±1.5px   σ=0.24   p99=0.56   最大 0.90 cm/s
#   噪声 ±2.5px   σ=0.40   p99=0.92   最大 1.50 cm/s
#   噪声 ±3.5px   σ=0.55   p99=1.28   最大 2.10 cm/s
# 门限取 1.4cm/s: 球真停住时一个窗口(320ms)内就有 99% 的把握读到线下 ——
# 一进容差立刻折返; 而还在以 3cm/s 往前爬的球要被误判需要 2.9σ, 约 0.2%,
# 不会平白丢掉那 0.9cm。
# 【为什么不用位置极差】极差由单帧噪声支配(±3px 的极差本身就有 0.48cm),
# 窗口非拉到 600ms 以上才分得清动没动 —— 这就是原来那两级保险要熬 800ms /
# 1800ms 的原因。改成半窗均值作差后 320ms 就够, 省下 0.5~1.5s 给第二段。
SEQ3_STALL_WIN_MS = 320             # 观察窗长度(约 14 帧 @45fps)
SEQ3_STALL_MIN_SAMPLES = 8          # 窗口里至少要这么多帧才敢下结论
SEQ3_STALL_CREEP_CM_S = 1.4         # 平均速度低于此 = 不再前进
SEQ3_STALL_GAP_CM = 0.9             # 只在离目标已进 ±1cm 容差时才允许放行

# 彻底卡死在更远处的兜底(gap > SEQ3_STALL_GAP_CM)。这种是真出事了 ——
# 摩擦异常大或机械卡住, 少走的量已经超出题目容差, 只能保证整轮不作废。
# 【SPREAD 别按理想静止设】±3px 噪声的位置极差就有 0.48cm, 设 0.3 的话
# 窗口被噪声反复重开, 永远熬不满, 保险等于没有。
SEQ3_REACH_STALL_TOL2_CM = 1.6
SEQ3_REACH_STALL_SPREAD_CM = 0.6
SEQ3_REACH_STALL_MS2 = 1500

# 终点要求"稳定"，按题目的 ±1cm 判，并且必须几乎不动才算数。
# 【别放到题目容差那么宽】原来给 1.0(题目容差)。球卡在 -4.2cm 推不动时,
# 误差 0.8 < 1.0 算 inside, 位置又纹丝不动, 400ms 后直接判"稳稳停住"发完成
# —— 判据把"卡住"当成了"到位"。序列一结束, 独立模式就把目标切回中心, 球被
# 推回 0: 这就是"直接倒冲到 0"的全部机制, 不是控制错误。
SEQ3_STABLE_TOL_CM = 0.35
SEQ3_STABLE_FALLBACK_TOL_CM = 1.0   # 真卡住时的兜底, 但要熬满 INSIDE_DWELL
SEQ3_STABLE_SPREAD_CM = 0.20        # 窗口内位置极差：阻止高速穿过终点时提前完成
SEQ3_STABLE_DWELL_MS = 250
# 只有进入严格位置窗且速度已实际降下来，才从行程控制切入
# 终点保持。切入后再连续验证300ms，不把“路过-5cm”当成完成。
SEQ3_TERMINAL_CAPTURE_TOL_CM = 0.90
SEQ3_TERMINAL_CAPTURE_SPEED_CM_S = 3.5
SEQ3_DONE_MAX_SPEED_CM_S = 1.2

# 兜底判据：球一直没出过容差，就算速度门限偶尔超也认。
# 【为什么要这一条】题目判的是位置，不是速度。球若在终点附近小幅来回但
# 从没出过 ±1cm，评委眼里那就是"稳定在该位置"，可速度门限会一直不成立，
# 严格判据永远等不到 —— 仿真里正是这样卡住的。
SEQ3_INSIDE_DWELL_MS = 2000

# 兜底：超时后不再等稳定判据，但仍然继续把球压在终点上。
# 【不发 DONE】超时说明这一次没做到，发了哨兵会让 3507 显示一个假的完成时间。
# 题目要求总时间小于5s；4.9s仍未完成就判本轮失败，不允许超时后报完成。
SEQ3_TIMEOUT_MS = 4900


# ============================== K230本地调试模式（不接3507） ==============================
# 只修改下面这一行：
#   3：O -> +5cm -> -5cm，最终保持在-5cm；
#   4：持续保持中心O点，模拟A到B行驶扰动；
#   5：持续保持中心O点，模拟整圈行驶扰动；
#   6：先等待钢球在任意位置稳定，再锁定该位置。
K230_TEST_MODE = 3

if K230_TEST_MODE not in (3, 4, 5, 6):
    raise ValueError("K230_TEST_MODE must be 3, 4, 5 or 6")
if TASK3_UART_TUNE_ENABLE and K230_TEST_MODE != 3:
    raise ValueError("UART4 task3 tuning requires K230_TEST_MODE = 3")

# 保留原状态机变量名，内部统一使用模式代号。
STANDALONE_ITEM = K230_TEST_MODE

# 【必须先等球稳在 O 再开跑】题目要求3 的行程是"从中心点 O 出发"，上电那
# 一刻球在哪儿不一定。不等就跑，第一段的 5cm 是从一个随机起点量出来的，
# 看着像做到了，实际终点差多少全凭运气。
STANDALONE_BOOT_MS = 1500        # 开机先空转，等视觉和 Kalman 收敛
STANDALONE_ARM_TOL_CM = 1.0      # 球离 O 多近算就位
STANDALONE_ARM_SPREAD_CM = 0.25  # 不用噪声较大的瞬时速度，以小窗口位置极差判定真正停稳
STANDALONE_ARM_MS = 1000         # 计时前准备可以等稳，不占用任务3的5s时限

# 模式6：当前位置连续稳定达到以下条件后，才锁定为目标。
MODE6_ARM_SPREAD_CM = 0.60
MODE6_ARM_MS = 800

# 【为什么判"位置波动"而不是"速度小"】Kalman 的速度估计里有一项
# KALMAN_VELOCITY_INNOVATION_GAIN * innovation / dt，YOLO 框中心抖 1 个
# 像素就会放大成 50px/s 的速度尖峰 —— 球纹丝不动，估计出来的速度却在乱跳。
# 拿它当门限要连续几十帧不出尖峰，实测框抖 ±2px 就再也凑不满，永远不开跑
# （控制器自己的 center_hold 门限是 6px/s，同样的原因在噪声下几乎不成立）。
# 位置是直接测量，噪声只有零点几毫米，看它的极差既抗噪又直接反映球动没动。

# 跑完之后要不要自动回中心再来一遍。
# False = 只跑一次, 之后目标就钉在终点上, 球一直被稳在那儿(题目要的
#         "最后稳定在该位置"就是这个状态), 不会再动。
# True  = 循环重跑, 调参时省事, 但每轮结束都会把球拽回中心。
STANDALONE_REPEAT = False
STANDALONE_RERUN_MS = 3000       # 在终点停留多久再回中心


# ============================== 串级控制 ==============================
MODE_CASCADE = 0
MODE_VELOCITY_TUNE = 1
CONTROL_MODE = MODE_CASCADE
VELOCITY_TUNE_SETPOINT_PX_S = 100.0

# 目标点为检测区域几何中心。机械安装有固定偏差时，只微调此值。
TARGET_POSITION_OFFSET_PX = 0.0

# 外环：位置误差 -> 速度给定。
# 不再使用固定增益：远离中心或小球向外冲时提高位置增益；
# 接近中心且速度较高时提高阻尼，并根据预计刹停距离提前制动。
POSITION_KP_NEAR = 1.15
POSITION_KP_NORMAL = 1.80
POSITION_KP_FAR = 2.35
POSITION_KP_DISTURBANCE_BOOST = 0.45
POSITION_KD_NEAR = 0.78
POSITION_KD_FAR = 0.30
# 外环按 sqrt(2*这个值*剩余距离) 决定还能跑多快, 也就是"多早开始减速"。
# 原值 235px/s2(18.8cm/s2) 偏乐观: 剩 5cm 时还允许 13.7cm/s, 而实际制动
# 能力只有 33cm/s2 —— 减速段留得太短。降到 120 后剩 5cm 只允许 9.8cm/s,
# 球有更长的距离把速度交出来。
POSITION_BRAKE_ACCEL_NEAR_PX_S2 = 80.0
POSITION_BRAKE_ACCEL_FAR_PX_S2 = 120.0
POSITION_BRAKE_MARGIN = 0.82
POSITION_BRAKE_MIN_CLOSING_SPEED_PX_S = 22.0
POSITION_REVERSE_BRAKE_MAX_PX_S = 34.0
POSITION_AWAY_VELOCITY_BOOST = 0.72
POSITION_AWAY_BOOST_LIMIT_PX_S = 90.0
# 【进场速度才是过冲的主因, 不是推力】原值 235px/s = 18.8cm/s, 而按制动
# 能力 33cm/s2 算, 要在 1cm 内刹住进场速度最多 8.1cm/s(102px/s) —— 18.8
# 的制动距离是 5.3cm, 五倍于容差, 再大的制动角也刹不住。
# 主机上扫过(推力保持 36cm/s2 不变, 只改限速):
#   235/70 (18.8cm/s)  过冲 2.88~3.05cm
#   150/60 (12.0cm/s)  过冲 1.08~1.12cm
#   110/55 ( 8.8cm/s)  过冲 0.83~0.91cm   <-- 取这组, 耗时也最短
#    75/45 ( 6.0cm/s)  过冲 0.87~0.89cm   更慢但没更准
# 推力管"推不推得动", 限速管"冲不冲过头", 这是两件事, 别用一个参数去凑。
MAX_TARGET_VELOCITY_NEAR_PX_S = 55.0
MAX_TARGET_VELOCITY_FAR_PX_S = 110.0

# 第二段(±5 -> 干5, 行程 10cm)单独限速: 它是第一段的两倍长, 球有更多距离
# 加速, 进终点时动能大得多。0 = 跟第一段用同一个值。
# 实机两次重启时的起步齿隙侧不同，80px/s设定在最差工况下
# 会因执行器换向滞后冲到16.6cm/s；降至60px/s让制动提前建立。
SEG_LEG1_SPEED_CAP_PX_S = 50.0
SEG_LEG2_SPEED_CAP_PX_S = 45.0
# 任务3第二段不再直接由位置PD生成时快时慢的速度给定，而是用
# 独立梯形速度规划：换向后平滑加速，中段连续运行，按剩余距离提前减速。
# 规划减速度故意小于实际制动能力，让减速点提前，却不因害怕越线而中途反复刹停。
SEG_LEG2_PROFILE_MAX_CM_S = 7.0
SEG_LEG2_PROFILE_ACCEL_CM_S2 = 12.0
SEG_LEG2_PROFILE_DECEL_CM_S2 = 10.0
SEG_LEG2_PROFILE_STOP_MARGIN_CM = 0.12
SEG_LEG2_OVERSHOOT_RETURN_MAX_CM_S = 2.0
# 第二段换向时电机/连杆约有数百毫秒回差。只限制速度给定不够：机构尚未
# 响应时速度环仍会给出远端满推力，回差一释放便产生十几 cm/s 的峰值。
# 因此第二段另设物理加速度（最终换算为电机角）的硬上限。
SEG_LEG1_DRIVE_ACCEL_CM_S2 = 20.0
SEG_LEG2_DRIVE_ACCEL_CM_S2 = 20.0

# 小车突然运动时，钢球会因惯性向外冲。没有UART3/IMU时，程序通过
# 摄像头估计的“向外速度、向外加速度和误差增长速度”识别冲击。
INERTIA_SPEED_TRIGGER_PX_S = 38.0
INERTIA_ACCEL_TRIGGER_PX_S2 = 300.0
INERTIA_ERROR_GROWTH_TRIGGER_PX_S = 28.0
INERTIA_HOLD_MS = 190
INERTIA_MIN_POSITION_ERROR_PX = 0.45

# 位置偏差快速倾角助推：只在钢球尚未达到外环要求速度时生效。
# 它不是替代速度环，而是解决“小偏差->小速度给定->小倾角->脉冲为0”的死区。
POSITION_ANGLE_ASSIST_KP = 0.20
POSITION_ANGLE_ASSIST_LIMIT_DEG = 10.0

# 中心捕获与保持。进入条件严格、退出条件宽松，形成滞回，防止频繁切换。
CENTER_ENTER_POSITION_PX = 1.0
CENTER_EXIT_POSITION_PX = 2.0
CENTER_ENTER_SPEED_PX_S = 6.0
CENTER_EXIT_SPEED_PX_S = 15.0
CENTER_CAPTURE_DWELL_MS = 120

# 保持状态只做轻微修正，不使用积分，不允许输出大倾角。
CENTER_HOLD_POSITION_KP = 2.40
CENTER_HOLD_VELOCITY_KD = 0.65
CENTER_HOLD_MAX_VELOCITY_PX_S = 32.0
# 保持状态限制的是“相对学习到的平衡角”的修正量，而不是总角度。
CENTER_HOLD_ANGLE_LIMIT_DEG = 4.0
CENTER_HOLD_ANGLE_STEP_DEG = 0.350
CENTER_SETTLED_POSITION_PX = 0.35
CENTER_SETTLED_SPEED_PX_S = 1.8
CENTER_LEVEL_BIAS_DEG = 0.0

# 任务3完成后的终点保持。不能简单把电机角度拉回0，也不能永久固定在某个角度：
# 本机存在明显机械零偏和静摩擦，实测固定-4deg会让球从-5cm滑到+3cm后卡住。
# 因此以任务启动前O点静止时的实际角度为平衡基准，在其附近做低速位置-速度PD。
TERMINAL_HOLD_KP_DEG_PER_CM = 0.90
TERMINAL_HOLD_KD_DEG_PER_CM_S = 1.80
# 任务3第二段从 +5cm 向 -5cm 到达终点后，传动间隙已经压在负向侧。
# 上电O点虽然也能静止，但可能停在正向回差侧（本机实测可达 +3.3deg）；
# 该角度不能复用作终点水平基准，否则完成后会把球从 -5cm 推回 -2.6cm。
TERMINAL_HOLD_PRELOAD_DEG = 8.80
TERMINAL_HOLD_CORRECTION_LIMIT_DEG = 5.50
TERMINAL_HOLD_TOTAL_LIMIT_DEG = 14.50
TERMINAL_HOLD_ANGLE_STEP_DEG = 0.55
TERMINAL_HOLD_TARGET_ALPHA = 0.50
TERMINAL_HOLD_MOTOR_SPEED_RPM = 95
# 终点PD推不动时的防卡脉冲。实机确认负向静止时即使-14deg也可能因
# 传动回差不启动；因此先反向小幅卸载，再负向推进，球一动立即释放给PD刹车。
TERMINAL_HOLD_STALL_ERROR_CM = 0.35
TERMINAL_HOLD_STALL_SPEED_CM_S = 0.35
TERMINAL_HOLD_STALL_DWELL_MS = 220
TERMINAL_HOLD_STALL_KICK_DEG = 13.50
TERMINAL_HOLD_STALL_RELEASE_SPEED_CM_S = 0.35
TERMINAL_HOLD_STALL_RELEASE_ERROR_CM = 0.18
TERMINAL_HOLD_STALL_UNLOAD_DEG = 2.00
TERMINAL_HOLD_STALL_UNLOAD_MS = 200
TERMINAL_HOLD_STALL_DRIVE_MS = 900
TERMINAL_HOLD_STALL_ANGLE_STEP_DEG = 0.45

# 中点附近不再连续追逐每一帧噪声，而是采用“误差持续确认 + 最小有效倾角”。
# 小球确实偏离且保持一段时间后，给一次足以克服静摩擦的稳定倾角；
# 一旦小球开始向中点运动，就立即撤掉额外倾角，避免电机来回空转。
CENTER_FINE_TRIGGER_PX = 0.45
CENTER_FINE_RELEASE_PX = 0.18
CENTER_FINE_DWELL_MS = 60
CENTER_FINE_MAX_SPEED_PX_S = 12.0
CENTER_FINE_STOP_CLOSING_SPEED_PX_S = 4.0
CENTER_FINE_MIN_ANGLE_DEG = 1.20
CENTER_FINE_KP_DEG_PER_PX = 0.45
CENTER_FINE_MAX_ANGLE_DEG = 4.00
CENTER_ERROR_LPF_ALPHA = 0.36

# 持续静差恢复：钢球在中点外静止时，说明普通串级输出不足以克服
# 平台传动间隙、轨道静摩擦或连杆减速比。程序先确认误差持续存在，
# 再沿正确方向平滑增加额外倾角，直到钢球真正开始回中。
# 该补偿随位置误差减小自动收缩，越过中点时立即清零，不会无限积分。
STATIC_RECOVERY_ENABLE = True
STATIC_RECOVERY_TRIGGER_PX = 0.55
STATIC_RECOVERY_RESET_PX = 0.16
# 【别按理想静止设】这是"球算不算停住了"的门槛, 而 Kalman 速度在识别噪声
# 下本来就有几十 px/s 的抖动。设 9 的话判据时断时续, 静差恢复的 70ms 驻留
# 永远凑不满 —— 球卡在离目标几毫米处推不动, 而专治这个的机制根本没启动。
STATIC_RECOVERY_MAX_STILL_SPEED_PX_S = 25.0
STATIC_RECOVERY_DWELL_MS = 120
STATIC_RECOVERY_MIN_ANGLE_DEG = 2.40
STATIC_RECOVERY_RAMP_DEG_S = 10.0
STATIC_RECOVERY_DECAY_DEG_S = 1.0
STATIC_RECOVERY_FAST_DECAY_DEG_S = 28.0
STATIC_RECOVERY_ERROR_TO_LIMIT_DEG_PER_PX = 0.55
STATIC_RECOVERY_MAX_ANGLE_DEG = 12.0
STATIC_RECOVERY_CLOSING_SPEED_PX_S = 100.0
STATIC_RECOVERY_SIGN_HYSTERESIS_PX = 0.45
# 恢复倾角一旦建立，不会在钢球刚开始移动时立即撤掉；只有进入中心附近、
# 明显高速接近中心或误差反向后才快速释放，避免钢球总停在同一偏置位置。
STATIC_RECOVERY_HOLD_UNTIL_PX = 0.80
STATIC_RECOVERY_RELEASE_SPEED_PX_S = 60.0
STATIC_RECOVERY_MIN_HOLD_RATIO = 0.88
STATIC_RECOVERY_KICK_INTERVAL_MS = 420
STATIC_RECOVERY_KICK_STEP_DEG = 0.90

# 最后阶段持续推进：钢球进入中点附近但尚未达到中心时，普通PD输出可能因
# 静摩擦和传动间隙再次变小。这里不额外叠加大角度，只保证控制角不低于
# 一个随剩余误差变化的最小值，并在钢球已经快速回中时自动退出。
FINAL_APPROACH_ENABLE = True
FINAL_APPROACH_ZONE_PX = 10.0
FINAL_APPROACH_RELEASE_PX = 0.28
FINAL_APPROACH_MAX_CLOSING_SPEED_PX_S = 28.0
FINAL_APPROACH_MIN_ANGLE_DEG = 1.20
FINAL_APPROACH_KP_DEG_PER_PX = 0.28
FINAL_APPROACH_MAX_ANGLE_DEG = 5.00

# 角度硬限位仍按位置距离分级开放；低通系数和角度步长由动态调度实时决定。
ANGLE_LIMIT_FAR_DEG = 18.0
ANGLE_LIMIT_EMERGENCY_POSITION_PX = 95.0

# 自动学习维持钢球在中点所需的平衡倾角。机械零位不完全水平时，
# 不再在SETTLED状态强制回0°，而是保留并缓慢修正这个平衡角。
CENTER_BIAS_LEARN_ENABLE = True
CENTER_BIAS_LEARN_KI_DEG_PER_PX_S = 0.012
CENTER_BIAS_ACTIVE_POSITION_PX = 1.0
CENTER_BIAS_ACTIVE_SPEED_PX_S = 3.5
CENTER_BIAS_LIMIT_DEG = 5.0

# 根据离中心的距离分级开放角度。软件总限位为±20°，但中心附近不会直接使用大角度。
ANGLE_LIMIT_NEAR_POSITION_PX = 5.0
ANGLE_LIMIT_MID_POSITION_PX = 22.0
ANGLE_LIMIT_FAR_POSITION_PX = 65.0
ANGLE_LIMIT_NEAR_DEG = 5.0
ANGLE_LIMIT_MID_DEG = 12.0

# 内环：速度误差 -> 倾角。使用动态增益调度：
# 中心附近降低比例并增加阻尼；惯性冲击时临时提高比例，积分暂停，快速压住外冲。
VELOCITY_KP_NEAR = 0.0260
VELOCITY_KP_NORMAL = 0.0360
VELOCITY_KP_DISTURBANCE = 0.0600
VELOCITY_KI_NEAR = 0.00035
VELOCITY_KI_NORMAL = 0.00100
VELOCITY_KD_NEAR = 0.00014
VELOCITY_KD_NORMAL = 0.00008
VELOCITY_KD_DISTURBANCE = 0.00006
VELOCITY_D_FILTER_TAU_S = 0.055
VELOCITY_I_ACTIVE_ERROR_PX_S = 95.0
VELOCITY_INTEGRAL_LEAK_NORMAL = 0.997
VELOCITY_INTEGRAL_LEAK_DISTURBANCE = 0.90
VELOCITY_INTEGRAL_LEAK_HOLD = 0.82

# 速度给定前馈使电机在外环刚给出目标速度时立即产生适量倾角；
# 加速度阻尼抵消摄像头检测到的突然惯性加速，二者均有限幅。
VELOCITY_FEEDFORWARD_K_DEG_PER_PX_S = 0.012
VELOCITY_FEEDFORWARD_LIMIT_DEG = 2.5
BALL_ACCEL_DAMP_K_NORMAL = 0.0012
BALL_ACCEL_DAMP_K_DISTURBANCE = 0.0048
BALL_ACCEL_DAMP_LIMIT_DEG = 4.0

# 动态电机目标整形：紧急状态响应快，正常状态平滑，中心附近最柔和。
ANGLE_TARGET_ALPHA_SETTLED = 0.12
ANGLE_TARGET_ALPHA_NEAR = 0.20
ANGLE_TARGET_ALPHA_NORMAL = 0.30
ANGLE_TARGET_ALPHA_DISTURBANCE = 0.45
MOTOR_ANGLE_STEP_NEAR_DEG = 0.45
MOTOR_ANGLE_STEP_NORMAL_DEG = 1.20
MOTOR_ANGLE_STEP_DISTURBANCE_DEG = 2.00
MOTOR_ANGLE_STEP_REVERSAL_DEG = 0.70
MOTOR_ANGLE_STEP_BRAKE_REVERSAL_DEG = 1.20
ANGLE_REVERSAL_GUARD_DEG = 0.80

# ---------- 仅针对项3第二段（实物上的 -5cm -> +5cm）柔和角度限制 ----------
# 任务状态机进入第二段时强制启用；同时保留“负侧设置正侧目标”的自动识别。
# 第一段、中心保持和其他测试项不受影响。
SEG_SOFT_ENABLE = True

# 【两段都要限】原来只在第二段启用, 第一段(0 -> +5)用的是 20 度总限位 =
# 41cm/s2, 是需要值的两倍多, 冲过 +5 就是这么来的。
#
# 参数全部改按【球加速度】定义, 再换算成电机角 —— 这样换曲柄半径不用重调:
#   球的滚动阻力      2~4 cm/s2
#   推得动的门槛      5~8 cm/s2  (含齿隙和装配余量)
#   项3 制动 6cm/s 进 1cm 内   18 cm/s2
# 【远处别收力】起步要克服的是静摩擦 + 连杆/铰点的摩擦, 实机总阻力比理论
# 滚动阻力大得多。把远处推力从原来的 41cm/s2 压到 20 之后, 球直接推不动了
# (现象: 0->+5 阶段球一直待在 0)。远处保持接近原值, 只在接近目标时收 ——
# 冲过头是"进终点时速度太大"造成的, 跟起步给多大劲没关系。
SEG_DRIVE_ACCEL_FAR = 36.0      # 远离目标时的推力上限(原始行为约 41)
SEG_DRIVE_ACCEL_NEAR = 17.0     # 接近目标时收到这里, 防冲过
SEG_BRAKE_ACCEL = 33.0          # 反向制动留大一些, 端点才刹得住
SEG_NEAR_ZONE_CM = 2.5          # 离目标多近开始收推力(第二段用)

# 【第一段不收力】两段的目标根本不同:
#   第一段 要的是"走到/走过 5cm", 判据就是越过即算到达 —— 冲过去无害,
#          立刻就折返了; 反而是差那么一两毫米推不动会直接扣行程。
#   第二段 要的是"精确停在 5cm 并稳住", 冲过才是问题。
# 之前两段共用一套收力参数, 等于拿第二段的顾虑去管第一段, 结果第一段总
# 差一点点碰不到线。第一段把收力区设 0 就完全不收, 全程给足推力。
SEG_LEG1_NEAR_ZONE_CM = 0.0

# 【球停下来就别收力了】收力是为了防止高速冲过终点。球一旦被制动到停住,
# 静摩擦比动摩擦大, 这时还按"离得近"去限推力, 它就再也起不来 —— 现象就是
# "还没到就停在半路", 而且静差恢复想加力也会被这个 cap 一起砍掉。
# 球速低于这个值时推力上限放回远处的值。
SEG_COAST_SPEED_PX_S = 30.0     # 2.4cm/s 以下算"基本停了"

# 【但离目标很近时不算卡住】球正常减速进终点时速度也会掉到这个值以下,
# 那时候放开推力等于在最不该加力的地方加力, 直接冲过去。只有"停住了 而且
# 离目标还有一段"才是真卡住, 才需要把推力放回去。
# 【别设太大】球停在离目标 0.4~0.8cm 的地方是最常见的卡死点: 按"离目标近"
# 收力就推不动, 按"还没到"又够不着防卡保险的容差 —— 两头都漏, 球就永远
# 停在那儿等超时。0.4 让这一档也能重新拿到全推力。
SEG_STALL_MIN_CM = 0.4
# 只在目标前最后一小段解除自适应角度限幅造成的“推不动”。
# 加上区域约束后，起步静止不会触发，不会因此增大第一段起步冲击。
SEG_STALL_RECOVERY_ZONE_CM = 2.0
SEG_STALL_RECOVERY_ANGLE_DEG = 13.5
SEG_STALL_RECOVERY_DWELL_MS = 180

# 第一段(0 -> +5)要不要也限幅。设 False 就完全回到原来的行为(只限第二段)。
# 【别轻易关】关掉的话第一段用 20 度总限位一路推到底, 那正是"冲过 +5"的成因。
SEG_SOFT_FIRST_LEG = True

# 【原值有多小】DRIVE_NEAR 4.0 电机度 = 0.68 杆度 = 8.3cm/s2, 正好卡在
# "推得动"的门槛上 —— 球进入终点 3cm 内就被限到这个值, 稍有阻力就停住,
# 这就是"到 -5 前基本停止"的原因。
SEG_DRIVE_MAX_ANGLE_DEG = accel_to_motor_deg(SEG_DRIVE_ACCEL_FAR)
SEG_DRIVE_NEAR_ANGLE_DEG = accel_to_motor_deg(SEG_DRIVE_ACCEL_NEAR)
SEG_BRAKE_MAX_ANGLE_DEG = accel_to_motor_deg(SEG_BRAKE_ACCEL)

# 推动方向的角度变化放缓, 免得把球一下子推飞。
SEG_MAX_ANGLE_STEP_DEG = 0.65
SEG_TARGET_ALPHA = 0.22

# 【制动方向要快一些, 但别过头】同一个步长时从 +8 度(推)翻到 -16 度(制动)
# 要 37 拍 = 740ms, 第二段球速高就来不及。但放太快也不行 —— 主机上扫过:
#   0.65/0.22 (同推动)  过冲 0.22cm  耗时 3.7s
#   1.5 /0.40           过冲 0.04~0.19cm  耗时 3.9s   <-- 取这组
#   3.0 /0.65           过冲 0.00 但球只到 +4.86, 耗时 5.4s 超时
#   5.0 /0.80           更差, 直接 DNF
# 制动太猛会在球还没到位时就把它拽住, 反而走不到 5cm。
SEG_BRAKE_ANGLE_STEP_DEG = 1.5
SEG_BRAKE_TARGET_ALPHA = 0.40

# 三状态恒加速度Kalman：[位置、速度、加速度]。低置信度测量自动降低权重。
# 【1.10 太乐观】YOLO 框中心实测抖 ±2~3px, 按 1.10 算等于告诉滤波器"测量
# 很准", 它就紧跟每一帧噪声, 速度估计跟着乱跳。
KALMAN_MEASUREMENT_STD_PX = 2.20
KALMAN_PROCESS_JERK_STD_PX_S3 = 1850.0
KALMAN_LOW_CONFIDENCE_R_SCALE = 4.0
KALMAN_FAST_MOTION_R_REDUCTION = 0.45
# 【关掉】这一项把 innovation/dt 直接加进速度, dt 只有 22ms, 等于把测量
# 噪声放大 45 倍灌进去。它宣称买到的低滞后只值 2ms(0.5Hz 工作频段实测),
# 代价却是近一半的速度噪声。
KALMAN_VELOCITY_INNOVATION_GAIN = 0.0

# LCD OSD使用时间调度，避免检测周期与帧取模重合后永远不刷新。
# 普通状态约8Hz，快速运动约6Hz；若连续YOLO导致没有空闲帧，
# 最多等待OSD_FORCE_MAX_GAP_MS后强制刷新一次。
OSD_INTERVAL_MS = 125
OSD_FAST_INTERVAL_MS = 170
OSD_FORCE_MAX_GAP_MS = 400
TERMINAL_INTERVAL_MS = 1000
GC_INTERVAL_MS = 20000


# 检测区域和标定点换算到控制坐标。
AI_GUIDE_X1 = CAL_LEFT_PX
AI_GUIDE_X2 = CAL_RIGHT_PX
AI_GUIDE_Y1 = ROI_Y * FULL_TO_CONTROL_Y
AI_GUIDE_Y2 = (ROI_Y + ROI_H) * FULL_TO_CONTROL_Y
AI_GUIDE_CENTER_X = CAL_CENTER_PX
TARGET_POSITION_PX = CAL_CENTER_PX
CENTER_TARGET_PX = CAL_CENTER_PX

# 仅用于打印和少量速度门限换算；位置换算本身使用三点分段标定。
PIXELS_PER_CM_LEFT = (
    CAL_CENTER_PX - CAL_LEFT_PX
) / (CAL_CENTER_CM - CAL_LEFT_CM)
PIXELS_PER_CM_RIGHT = (
    CAL_RIGHT_PX - CAL_CENTER_PX
) / (CAL_RIGHT_CM - CAL_CENTER_CM)
PIXELS_PER_CM = 0.5 * (PIXELS_PER_CM_LEFT + PIXELS_PER_CM_RIGHT)

# UART4任务3调参表：别名，实际全局变量，下限，上限，是否取整。
# PC下发 [SET,VFAR,100] 即可在不重启K230的情况下改参。
TASK3_TUNE_PARAM_SPECS = (
    ("VFAR", "MAX_TARGET_VELOCITY_FAR_PX_S", 30.0, 180.0, False),
    ("VNEAR", "MAX_TARGET_VELOCITY_NEAR_PX_S", 15.0, 120.0, False),
    ("VLEG1", "SEG_LEG1_SPEED_CAP_PX_S", 20.0, 160.0, False),
    ("VLEG2", "SEG_LEG2_SPEED_CAP_PX_S", 20.0, 160.0, False),
    ("PVMAX", "SEG_LEG2_PROFILE_MAX_CM_S", 3.0, 12.0, False),
    ("PVACC", "SEG_LEG2_PROFILE_ACCEL_CM_S2", 4.0, 30.0, False),
    ("PVDEC", "SEG_LEG2_PROFILE_DECEL_CM_S2", 4.0, 30.0, False),
    ("PSTOP", "SEG_LEG2_PROFILE_STOP_MARGIN_CM", 0.0, 0.80, False),
    ("PVRET", "SEG_LEG2_OVERSHOOT_RETURN_MAX_CM_S", 0.5, 4.0, False),
    ("ALEG1", "SEG_LEG1_DRIVE_ACCEL_CM_S2", 10.0, 35.0, False),
    ("ALEG2", "SEG_LEG2_DRIVE_ACCEL_CM_S2", 10.0, 35.0, False),
    ("XLEG2", "SEQ3_LEG2_TARGET_EXTRA_CM", 0.0, 1.0, False),
    ("AFAR", "SEG_DRIVE_ACCEL_FAR", 8.0, 60.0, False),
    ("ANEAR", "SEG_DRIVE_ACCEL_NEAR", 5.0, 45.0, False),
    ("ABRAKE", "SEG_BRAKE_ACCEL", 8.0, 60.0, False),
    ("ZONE", "SEG_NEAR_ZONE_CM", 0.0, 5.0, False),
    ("L1ZONE", "SEG_LEG1_NEAR_ZONE_CM", 0.0, 5.0, False),
    ("COASTV", "SEG_COAST_SPEED_PX_S", 5.0, 80.0, False),
    ("STALLCM", "SEG_STALL_MIN_CM", 0.10, 2.00, False),
    ("SRZONE", "SEG_STALL_RECOVERY_ZONE_CM", 0.50, 4.00, False),
    ("SRANGLE", "SEG_STALL_RECOVERY_ANGLE_DEG", 5.00, 18.00, False),
    ("SRDWELL", "SEG_STALL_RECOVERY_DWELL_MS", 50.0, 800.0, True),
    ("DSTEP", "SEG_MAX_ANGLE_STEP_DEG", 0.10, 3.00, False),
    ("DALPHA", "SEG_TARGET_ALPHA", 0.05, 0.90, False),
    ("BSTEP", "SEG_BRAKE_ANGLE_STEP_DEG", 0.10, 5.00, False),
    ("BALPHA", "SEG_BRAKE_TARGET_ALPHA", 0.05, 0.95, False),
    ("PKPNEAR", "POSITION_KP_NEAR", 0.10, 5.00, False),
    ("PKPFAR", "POSITION_KP_FAR", 0.10, 6.00, False),
    ("PKDNEAR", "POSITION_KD_NEAR", 0.00, 3.00, False),
    ("PKDFAR", "POSITION_KD_FAR", 0.00, 3.00, False),
    ("VKP", "VELOCITY_KP_NORMAL", 0.005, 0.150, False),
    ("VKI", "VELOCITY_KI_NORMAL", 0.000, 0.010, False),
    ("VKD", "VELOCITY_KD_NORMAL", 0.000, 0.005, False),
    ("STOL", "SEQ3_STABLE_TOL_CM", 0.10, 1.00, False),
    ("SDWELL", "SEQ3_STABLE_DWELL_MS", 100.0, 2000.0, True),
    ("CAPTOL", "SEQ3_TERMINAL_CAPTURE_TOL_CM", 0.30, 1.00, False),
    ("CAPSPD", "SEQ3_TERMINAL_CAPTURE_SPEED_CM_S", 0.30, 5.00, False),
    ("DONESPD", "SEQ3_DONE_MAX_SPEED_CM_S", 0.30, 3.00, False),
    ("TIMEOUT", "SEQ3_TIMEOUT_MS", 3000.0, 15000.0, True),
    ("THKP", "TERMINAL_HOLD_KP_DEG_PER_CM", 0.10, 3.00, False),
    ("THKD", "TERMINAL_HOLD_KD_DEG_PER_CM_S", 0.00, 3.00, False),
    ("THLIM", "TERMINAL_HOLD_CORRECTION_LIMIT_DEG", 2.00, 12.00, False),
    ("THTOTAL", "TERMINAL_HOLD_TOTAL_LIMIT_DEG", 4.00, 16.00, False),
    ("THSTEP", "TERMINAL_HOLD_ANGLE_STEP_DEG", 0.05, 1.00, False),
    ("THALPHA", "TERMINAL_HOLD_TARGET_ALPHA", 0.05, 0.90, False),
    ("THRPM", "TERMINAL_HOLD_MOTOR_SPEED_RPM", 30.0, 130.0, True),
    ("THSERR", "TERMINAL_HOLD_STALL_ERROR_CM", 0.20, 2.00, False),
    ("THSSPD", "TERMINAL_HOLD_STALL_SPEED_CM_S", 0.05, 1.50, False),
    ("THSDW", "TERMINAL_HOLD_STALL_DWELL_MS", 80.0, 1000.0, True),
    ("THSKICK", "TERMINAL_HOLD_STALL_KICK_DEG", 5.00, 14.00, False),
    ("THSREL", "TERMINAL_HOLD_STALL_RELEASE_SPEED_CM_S", 0.20, 3.00, False),
    ("THSRERR", "TERMINAL_HOLD_STALL_RELEASE_ERROR_CM", 0.10, 1.00, False),
    ("THSUNLD", "TERMINAL_HOLD_STALL_UNLOAD_DEG", 0.50, 6.00, False),
    ("THSUNMS", "TERMINAL_HOLD_STALL_UNLOAD_MS", 80.0, 800.0, True),
    ("THSDRV", "TERMINAL_HOLD_STALL_DRIVE_MS", 200.0, 2000.0, True),
    ("THSSTEP", "TERMINAL_HOLD_STALL_ANGLE_STEP_DEG", 0.10, 1.50, False),
)


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def task3_tune_find_spec(alias):
    alias = str(alias).strip().upper()
    for spec in TASK3_TUNE_PARAM_SPECS:
        if spec[0] == alias:
            return spec
    return None


def task3_tune_refresh_derived(controller=None):
    """调参后立即重算由加速度派生的电机角度上限。"""
    global SEG_DRIVE_MAX_ANGLE_DEG
    global SEG_DRIVE_NEAR_ANGLE_DEG
    global SEG_BRAKE_MAX_ANGLE_DEG

    SEG_DRIVE_MAX_ANGLE_DEG = accel_to_motor_deg(SEG_DRIVE_ACCEL_FAR)
    SEG_DRIVE_NEAR_ANGLE_DEG = accel_to_motor_deg(SEG_DRIVE_ACCEL_NEAR)
    SEG_BRAKE_MAX_ANGLE_DEG = accel_to_motor_deg(SEG_BRAKE_ACCEL)

    if controller is not None:
        controller.velocity_pid.kp = VELOCITY_KP_NORMAL
        controller.velocity_pid.ki = VELOCITY_KI_NORMAL
        controller.velocity_pid.kd = VELOCITY_KD_NORMAL
        controller.velocity_pid.reset_integral()


def task3_tune_set_parameter(alias, raw_value, controller=None):
    spec = task3_tune_find_spec(alias)
    if spec is None:
        return False, None

    try:
        value = float(raw_value)
    except Exception:
        return False, None
    if value != value:  # NaN
        return False, None

    value = clamp(value, spec[2], spec[3])
    if spec[4]:
        value = int(value + 0.5)
    globals()[spec[1]] = value
    task3_tune_refresh_derived(controller)
    return True, value


def ticks_shift(ticks_ms, delta_ms):
    try:
        return time.ticks_add(ticks_ms, int(delta_ms))
    except Exception:
        return ticks_ms + int(delta_ms)


def full_to_control_x(x_full):
    return float(x_full) * FULL_TO_CONTROL_X


def full_to_control_y(y_full):
    return float(y_full) * FULL_TO_CONTROL_Y


def control_to_full_x(x_control):
    return float(x_control) * CONTROL_TO_FULL_X


def control_to_full_y(y_control):
    return float(y_control) * CONTROL_TO_FULL_Y


def px_to_cm(position_px):
    """控制坐标像素 -> 以O点为原点的厘米，三点分段线性。"""
    x_px = float(position_px)
    if x_px <= CAL_CENTER_PX:
        denominator = CAL_CENTER_PX - CAL_LEFT_PX
        if abs(denominator) < 1e-6:
            return 0.0
        ratio = (x_px - CAL_LEFT_PX) / denominator
        value = CAL_LEFT_CM + ratio * (CAL_CENTER_CM - CAL_LEFT_CM)
    else:
        denominator = CAL_RIGHT_PX - CAL_CENTER_PX
        if abs(denominator) < 1e-6:
            return 0.0
        ratio = (x_px - CAL_CENTER_PX) / denominator
        value = CAL_CENTER_CM + ratio * (CAL_RIGHT_CM - CAL_CENTER_CM)
    return value * BALL_X_SIGN


def cm_to_px(position_cm):
    """厘米 -> 控制坐标像素，px_to_cm的分段反函数。"""
    x_cm = float(position_cm) * BALL_X_SIGN
    if x_cm <= CAL_CENTER_CM:
        denominator = CAL_CENTER_CM - CAL_LEFT_CM
        if abs(denominator) < 1e-6:
            return CAL_CENTER_PX
        ratio = (x_cm - CAL_LEFT_CM) / denominator
        return CAL_LEFT_PX + ratio * (CAL_CENTER_PX - CAL_LEFT_PX)
    denominator = CAL_RIGHT_CM - CAL_CENTER_CM
    if abs(denominator) < 1e-6:
        return CAL_CENTER_PX
    ratio = (x_cm - CAL_CENTER_CM) / denominator
    return CAL_CENTER_PX + ratio * (CAL_RIGHT_PX - CAL_CENTER_PX)


def px_s_to_cm_s(velocity_px_s):
    """控制坐标速度 -> cm/s；中心附近取左右标定比例平均值。"""
    return float(velocity_px_s) / max(PIXELS_PER_CM, 1e-6) * BALL_X_SIGN


def cm_s_to_px_s(velocity_cm_s):
    """cm/s -> 控制坐标速度，与px_s_to_cm_s互为反变换。"""
    return float(velocity_cm_s) * PIXELS_PER_CM * BALL_X_SIGN


def inside_rod_roi(cx, cy):
    """判断钢球候选中心是否位于800×480完整画面的摆杆ROI内。

    OpenCV轮廓和霍夫圆产生的cx/cy均为完整画面坐标，
    因此这里必须使用ROI_X/ROI_Y等完整分辨率参数判断。
    """
    return (
        cx >= ROI_X + ROI_END_MARGIN
        and cx <= ROI_X + ROI_W - ROI_END_MARGIN
        and cy >= ROI_Y
        and cy <= ROI_Y + ROI_H
    )


# ============================== OpenCV传统视觉检测器 ==============================
# ============================================================

class OpenCVSteelBallDetector:
    def __init__(self):
        self.open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE)
        )

        self.close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE)
        )

        self.last_dark_threshold = float(DARK_MAX)

    def _find_contours_compatible(self, binary):
        result = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(result) == 2:
            contours, hierarchy = result
        else:
            _, contours, hierarchy = result

        return contours, hierarchy

    def _make_dark_mask(self, blurred):
        mode = DARK_THRESHOLD_MODE.upper()

        if mode == "OTSU":
            actual_threshold, mask = cv2.threshold(
                blurred,
                0,
                255,
                cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
            )
            self.last_dark_threshold = float(actual_threshold)
            return mask

        if mode == "ADAPTIVE":
            mask = cv2.adaptiveThreshold(
                blurred,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                ADAPTIVE_BLOCK_SIZE,
                ADAPTIVE_C
            )
            self.last_dark_threshold = -1.0
            return mask

        actual_threshold, mask = cv2.threshold(
            blurred,
            DARK_MAX,
            255,
            cv2.THRESH_BINARY_INV
        )
        self.last_dark_threshold = float(actual_threshold)
        return mask

    def _make_bright_mask(self, blurred):
        _, mask = cv2.threshold(
            blurred,
            BRIGHT_MIN,
            255,
            cv2.THRESH_BINARY
        )
        return mask

    def _clean_mask(self, mask):
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self.open_kernel,
            iterations=MORPH_OPEN_ITERATIONS
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self.close_kernel,
            iterations=MORPH_CLOSE_ITERATIONS
        )

        return mask

    def _contours_to_candidates(self, contours, source_name):
        candidates = []

        for contour in contours:
            try:
                contour_area = float(cv2.contourArea(contour))
            except Exception:
                continue

            if (
                contour_area < MIN_CONTOUR_AREA or
                contour_area > MAX_CONTOUR_AREA
            ):
                continue

            try:
                x_local, y_local, w, h = cv2.boundingRect(contour)
            except Exception:
                continue

            x_local = int(x_local)
            y_local = int(y_local)
            w = int(w)
            h = int(h)

            if w < MIN_BALL_W or w > MAX_BALL_W:
                continue
            if h < MIN_BALL_H or h > MAX_BALL_H:
                continue

            bbox_area = w * h
            if (
                bbox_area < MIN_BBOX_AREA or
                bbox_area > MAX_BBOX_AREA
            ):
                continue

            aspect = w / float(h)
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                continue

            fill_ratio = contour_area / float(bbox_area)
            if (
                fill_ratio < MIN_FILL_RATIO or
                fill_ratio > MAX_FILL_RATIO
            ):
                continue

            try:
                perimeter = float(
                    cv2.arcLength(contour, True)
                )
            except Exception:
                continue

            if perimeter <= 1e-6:
                continue

            circularity = (
                4.0 * math.pi * contour_area /
                (perimeter * perimeter)
            )

            if circularity < MIN_CIRCULARITY:
                continue

            bbox_cx_local = x_local + w * 0.5
            bbox_cy_local = y_local + h * 0.5

            moment_cx_local = bbox_cx_local
            moment_cy_local = bbox_cy_local

            try:
                moments = cv2.moments(contour)
                m00 = float(moments["m00"])

                if abs(m00) > 1e-6:
                    moment_cx_local = (
                        float(moments["m10"]) / m00
                    )
                    moment_cy_local = (
                        float(moments["m01"]) / m00
                    )
            except Exception:
                pass

            center_weight = MOMENT_CENTER_WEIGHT

            cx_local = (
                center_weight * moment_cx_local +
                (1.0 - center_weight) * bbox_cx_local
            )

            cy_local = (
                center_weight * moment_cy_local +
                (1.0 - center_weight) * bbox_cy_local
            )

            radius = (w + h) * 0.25

            try:
                circle_center, circle_radius = \
                    cv2.minEnclosingCircle(contour)

                circle_radius = float(circle_radius)

                # 外接圆只用于尺寸辅助，不直接完全替代质心。
                if circle_radius > 0:
                    radius = circle_radius
            except Exception:
                pass

            cx = cx_local + ROI_X
            cy = cy_local + ROI_Y

            if not inside_rod_roi(cx, cy):
                continue

            y_error = abs(cy - ROD_CENTER_Y)
            if y_error > MAX_Y_ERROR:
                continue

            candidates.append({
                "x": x_local + ROI_X,
                "y": y_local + ROI_Y,
                "w": w,
                "h": h,
                "cx": cx,
                "cy": cy,
                "radius": radius,
                "area": contour_area,
                "fill_ratio": fill_ratio,
                "circularity": circularity,
                "source": source_name,
                "quality": 0.0,
                "merge_count": 1,
            })

        return candidates

    def detect_contours(self, gray_np):
        # 生成连续内存的ROI副本，避免OpenCV处理步长切片失败。
        roi_view = gray_np[
            ROI_Y:ROI_Y + ROI_H,
            ROI_X:ROI_X + ROI_W
        ]

        roi = np.array(roi_view, dtype=np.uint8)

        blurred = cv2.medianBlur(
            roi,
            MEDIAN_BLUR_KSIZE
        )

        candidates = []

        if ENABLE_DARK_CONTOUR:
            dark_mask = self._make_dark_mask(blurred)
            dark_mask = self._clean_mask(dark_mask)

            dark_contours, _ = \
                self._find_contours_compatible(dark_mask)

            candidates.extend(
                self._contours_to_candidates(
                    dark_contours,
                    "dark"
                )
            )

        if ENABLE_BRIGHT_CONTOUR:
            bright_mask = self._make_bright_mask(blurred)
            bright_mask = self._clean_mask(bright_mask)

            bright_contours, _ = \
                self._find_contours_compatible(bright_mask)

            candidates.extend(
                self._contours_to_candidates(
                    bright_contours,
                    "bright"
                )
            )

        return candidates, blurred

    def hough_reacquire(self, blurred_roi):
        try:
            circles = cv2.HoughCircles(
                blurred_roi,
                getattr(cv2, "HOUGH_GRADIENT", 3),
                HOUGH_DP,
                HOUGH_MIN_DIST,
                param1=HOUGH_CANNY_HIGH,
                param2=HOUGH_ACC_THRESHOLD,
                minRadius=HOUGH_MIN_RADIUS,
                maxRadius=HOUGH_MAX_RADIUS
            )
        except Exception as error:
            print("HoughCircles warning:", repr(error))
            return []

        if circles is None:
            return []

        candidates = []

        try:
            shape = circles.shape
        except Exception:
            return candidates

        rows = []

        # 官方K230文档为Nx3；兼容标准OpenCV常见的1xNx3。
        try:
            if len(shape) == 2:
                for index in range(shape[0]):
                    rows.append(circles[index])
            elif len(shape) == 3:
                for index in range(shape[1]):
                    rows.append(circles[0][index])
        except Exception:
            return candidates

        for row in rows:
            try:
                cx_local = float(row[0])
                cy_local = float(row[1])
                radius = float(row[2])
            except Exception:
                continue

            cx = cx_local + ROI_X
            cy = cy_local + ROI_Y

            if not inside_rod_roi(cx, cy):
                continue

            if abs(cy - ROD_CENTER_Y) > MAX_Y_ERROR:
                continue

            diameter = radius * 2.0

            if diameter < MIN_BALL_W:
                continue
            if diameter > MAX_BALL_W:
                continue

            candidates.append({
                "x": int(cx - radius),
                "y": int(cy - radius),
                "w": int(round(diameter)),
                "h": int(round(diameter)),
                "cx": cx,
                "cy": cy,
                "radius": radius,
                "area": math.pi * radius * radius,
                "fill_ratio": 0.78,
                "circularity": 1.0,
                "source": "circle",
                "quality": 0.0,
                "merge_count": 1,
            })

        return candidates


# ============================================================
# ============================== 候选合并与评分 ==============================
# ============================================================

def merge_close_candidates(candidates):
    merged = []

    for candidate in candidates:
        matched = None

        for group in merged:
            dx = candidate["cx"] - group["cx"]
            dy = candidate["cy"] - group["cy"]

            distance = math.sqrt(
                dx * dx + dy * dy
            )

            if distance <= MERGE_CENTER_DISTANCE:
                matched = group
                break

        if matched is None:
            merged.append(dict(candidate))
            continue

        count = matched.get("merge_count", 1)

        matched["cx"] = (
            matched["cx"] * count +
            candidate["cx"]
        ) / (count + 1)

        matched["cy"] = (
            matched["cy"] * count +
            candidate["cy"]
        ) / (count + 1)

        if (
            candidate["w"] * candidate["h"] >
            matched["w"] * matched["h"]
        ):
            matched["x"] = candidate["x"]
            matched["y"] = candidate["y"]
            matched["w"] = candidate["w"]
            matched["h"] = candidate["h"]
            matched["radius"] = candidate["radius"]
            matched["area"] = candidate["area"]
            matched["fill_ratio"] = \
                candidate["fill_ratio"]
            matched["circularity"] = \
                candidate["circularity"]

        matched["source"] = (
            matched["source"] + "+" +
            candidate["source"]
        )

        matched["merge_count"] = count + 1

    return merged


def score_candidate(
    candidate,
    predicted_x,
    tracking_valid
):
    diameter = (
        candidate["w"] +
        candidate["h"]
    ) * 0.5

    size_error = abs(
        diameter - EXPECTED_DIAMETER_PX
    )

    aspect_error = abs(
        candidate["w"] -
        candidate["h"]
    )

    y_error = abs(
        candidate["cy"] -
        ROD_CENTER_Y
    )

    circularity_penalty = (
        1.0 -
        clamp(candidate.get("circularity", 0.0), 0.0, 1.0)
    ) * 12.0

    fill_penalty = abs(
        candidate.get("fill_ratio", 0.5) -
        0.65
    ) * 8.0

    score = (
        y_error * 1.8 +
        size_error * 1.20 +
        aspect_error * 0.60 +
        circularity_penalty +
        fill_penalty
    )

    if tracking_valid:
        x_error = abs(
            candidate["cx"] -
            predicted_x
        )

        if x_error > MAX_TRACK_JUMP_PX:
            score += 1000.0
        else:
            score += x_error * 0.80

    if candidate["source"].startswith("dark"):
        score -= 3.0

    if "circle" in candidate["source"]:
        score -= 2.0

    if candidate.get("merge_count", 1) >= 2:
        score -= 4.0

    candidate["quality"] = clamp(
        100.0 - score,
        0.0,
        100.0
    )

    return score


def select_best_candidate(
    candidates,
    predicted_x,
    tracking_valid
):
    if not candidates:
        return None

    best = None
    best_score = 1e9

    for candidate in candidates:
        score = score_candidate(
            candidate,
            predicted_x,
            tracking_valid
        )

        if score < best_score:
            best_score = score
            best = candidate

    if best_score >= 900.0:
        return None

    if best.get("quality", 0.0) < MIN_CANDIDATE_QUALITY:
        return None

    return best


# ============================================================


def candidate_to_detection(candidate):
    """完整画面候选框 -> 原控制器兼容的检测元组。"""
    confidence = clamp(candidate.get("quality", 0.0) / 100.0, 0.0, 1.0)
    x1 = full_to_control_x(candidate["x"])
    y1 = full_to_control_y(candidate["y"])
    x2 = full_to_control_x(candidate["x"] + candidate["w"])
    y2 = full_to_control_y(candidate["y"] + candidate["h"])
    return (confidence, x1, y1, x2, y2)


def ball_center_px(detection):
    _, x1, y1, x2, y2 = detection
    return (x1 + x2) * 0.5, (y1 + y2) * 0.5


# ============================== Kalman / PID / 串级控制 ==============================
class KalmanPositionVelocity:
    """三状态恒加速度Kalman：状态为x、v、a，测量只有摄像头x。"""

    def __init__(self):
        self.base_r = KALMAN_MEASUREMENT_STD_PX * KALMAN_MEASUREMENT_STD_PX
        self.q = KALMAN_PROCESS_JERK_STD_PX_S3 * KALMAN_PROCESS_JERK_STD_PX_S3
        self.reset()

    def reset(self, position=None, timestamp_ms=None):
        self.position = 0.0 if position is None else float(position)
        self.velocity = 0.0
        self.acceleration = 0.0
        self.p00 = 16.0
        self.p01 = 0.0
        self.p02 = 0.0
        self.p11 = 3600.0
        self.p12 = 0.0
        self.p22 = 160000.0
        self.timestamp_ms = timestamp_ms
        self.ready = position is not None

    def _predict_inplace(self, dt):
        dt = clamp(dt, 0.0, 0.180)
        if dt <= 0.0:
            return

        dt2 = dt * dt
        half_dt2 = 0.5 * dt2
        self.position += self.velocity * dt + half_dt2 * self.acceleration
        self.velocity += self.acceleration * dt

        # P = F P F^T + Q，F为恒加速度模型。
        a00 = self.p00 + dt * self.p01 + half_dt2 * self.p02
        a01 = self.p01 + dt * self.p11 + half_dt2 * self.p12
        a02 = self.p02 + dt * self.p12 + half_dt2 * self.p22
        a11 = self.p11 + dt * self.p12
        a12 = self.p12 + dt * self.p22

        p00 = a00 + dt * a01 + half_dt2 * a02
        p01 = a01 + dt * a02
        p02 = a02
        p11 = a11 + dt * a12
        p12 = a12
        p22 = self.p22

        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        dt5 = dt4 * dt
        q = self.q
        self.p00 = p00 + q * dt5 / 20.0
        self.p01 = p01 + q * dt4 / 8.0
        self.p02 = p02 + q * dt3 / 6.0
        self.p11 = p11 + q * dt3 / 3.0
        self.p12 = p12 + q * dt2 / 2.0
        self.p22 = p22 + q * dt

    def correct(self, measurement, measurement_ms, confidence=1.0):
        if not self.ready:
            self.reset(measurement, measurement_ms)
            return self.position, self.velocity, self.acceleration

        dt = clamp(
            time.ticks_diff(measurement_ms, self.timestamp_ms) / 1000.0,
            0.003,
            0.180,
        )
        self._predict_inplace(dt)

        quality = clamp(
            (float(confidence) - TRACK_CONFIDENCE_THRESHOLD)
            / max(0.01, 1.0 - TRACK_CONFIDENCE_THRESHOLD),
            0.0,
            1.0,
        )
        # 高速运动时适当提高对YOLO位置测量的信任，减少滤波相位滞后；
        # 低置信度时仍自动增大R，防止运动模糊误框把轨迹拉偏。
        speed_factor = clamp(abs(self.velocity) / 120.0, 0.0, 1.0)
        motion_r_scale = 1.0 - KALMAN_FAST_MOTION_R_REDUCTION * speed_factor
        r = self.base_r * (
            1.0 + (1.0 - quality) * KALMAN_LOW_CONFIDENCE_R_SCALE
        ) * motion_r_scale

        innovation = measurement - self.position
        innovation_var = self.p00 + r
        k0 = self.p00 / innovation_var
        k1 = self.p01 / innovation_var
        k2 = self.p02 / innovation_var

        p00 = self.p00
        p01 = self.p01
        p02 = self.p02
        self.position += k0 * innovation
        self.velocity += k1 * innovation

        # 两次YOLO之间隔着若干预测帧。用少量“创新/时间”直接修正速度，
        # 可明显减轻运动钢球方向变化时Kalman速度跟随滞后。
        if confidence >= LOW_CONFIDENCE_FORCE_DETECT:
            self.velocity += (
                KALMAN_VELOCITY_INNOVATION_GAIN
                * innovation / max(dt, 0.012)
            )

        self.acceleration += k2 * innovation
        self.acceleration = clamp(
            self.acceleration,
            -BALL_ACCEL_LIMIT_PX_S2,
            BALL_ACCEL_LIMIT_PX_S2,
        )

        self.p00 = (1.0 - k0) * p00
        self.p01 = (1.0 - k0) * p01
        self.p02 = (1.0 - k0) * p02
        self.p11 = self.p11 - k1 * p01
        self.p12 = self.p12 - k1 * p02
        self.p22 = self.p22 - k2 * p02
        self.timestamp_ms = measurement_ms
        return self.position, self.velocity, self.acceleration

    def predict(self, query_ms, extra_delay_ms=0.0):
        if not self.ready:
            return None

        elapsed_ms = max(0, time.ticks_diff(query_ms, self.timestamp_ms))
        prediction_ms = clamp(
            elapsed_ms + LATENCY_COMPENSATION_GAIN * extra_delay_ms,
            0.0,
            MAX_PREDICTION_MS,
        )
        dt = prediction_ms / 1000.0
        position = (
            self.position
            + self.velocity * dt
            + 0.5 * self.acceleration * dt * dt
        )
        velocity = self.velocity + self.acceleration * dt
        return position, velocity, prediction_ms


class PID:
    def __init__(self, kp, ki, kd, output_limit, derivative_tau):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = output_limit
        self.derivative_tau = derivative_tau
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.derivative = 0.0
        self.ready = False

    def reset_integral(self):
        self.integral = 0.0

    def update(
        self, error, dt, integrate=True, integral_leak=1.0,
        kp=None, ki=None, kd=None,
    ):
        dt = clamp(dt, 0.005, 0.100)
        raw_d = (error - self.last_error) / dt if self.ready else 0.0
        alpha = dt / (self.derivative_tau + dt)
        self.derivative += alpha * (raw_d - self.derivative)
        self.ready = True

        use_kp = self.kp if kp is None else kp
        use_ki = self.ki if ki is None else ki
        use_kd = self.kd if kd is None else kd

        self.integral *= clamp(integral_leak, 0.0, 1.0)
        new_integral = self.integral + error * dt if integrate else self.integral
        raw = (
            use_kp * error
            + use_ki * new_integral
            + use_kd * self.derivative
        )
        output = clamp(raw, -self.limit, self.limit)

        if integrate and (
            raw == output
            or (raw > self.limit and error < 0)
            or (raw < -self.limit and error > 0)
        ):
            self.integral = new_integral

        self.last_error = error
        return output


class CascadeController:
    def __init__(self):
        # 目标点是可搬动的：项3 的序列要把它挪到 ±5cm，项6 要就地锁定。
        # 【不放进 reset()】丢球超时会调 reset，那时把目标悄悄弹回中心，
        # 球一重新出现就往回跑，整个序列白做。目标只由任务层显式改。
        self.target_px = TARGET_POSITION_PX
        self.kalman = KalmanPositionVelocity()
        self.last_measurement_px = None
        self.last_measurement_ms = None
        self.velocity_pid = PID(
            VELOCITY_KP_NORMAL,
            VELOCITY_KI_NORMAL,
            VELOCITY_KD_NORMAL,
            MOTOR_SOFT_LIMIT_DEG,
            VELOCITY_D_FILTER_TAU_S,
        )
        self.last_control_ms = None
        self.angle_command_deg = 0.0
        # 车辆起步加速度补偿。它不进入球位置PID，只在最终电机角度输出处
        # 与闭环命令相加，避免改变已经调好的位置/速度控制器状态。
        self.external_feedforward_deg = 0.0
        self.center_hold = False
        # 任务3完成后的专用终点保持：锁住理论 -5cm 目标，不再因普通
        # center_hold 的小范围退出门限重新落回大角度行程控制。
        self.terminal_hold_active = False
        self.terminal_hold_bias_deg = CENTER_LEVEL_BIAS_DEG
        self.terminal_stall_candidate_ms = None
        self.terminal_stall_active = False
        self.terminal_stall_sign = 0.0
        self.terminal_stall_phase = 0
        self.terminal_stall_phase_started_ms = None
        self.center_candidate_ms = None
        self.center_bias_deg = CENTER_LEVEL_BIAS_DEG
        self.settled_state = False
        self.filtered_position_error_px = 0.0
        self.error_filter_ready = False
        self.fine_candidate_ms = None
        self.fine_active = False
        self.static_recovery_candidate_ms = None
        self.static_recovery_angle_deg = 0.0
        self.static_recovery_sign = 0.0
        self.static_recovery_last_kick_ms = None
        self.filtered_target_angle_deg = 0.0
        self.disturbance_until_ms = None
        self.last_abs_position_error_px = None
        self.last_motion_context_ms = None
        self.dynamic_urgency = 0.0
        self.dynamic_position_kp = POSITION_KP_NORMAL
        self.dynamic_velocity_kp = VELOCITY_KP_NORMAL
        self.dynamic_brake_active = False
        self.dynamic_disturbance_active = False
        self.dynamic_closing_velocity_px_s = 0.0
        self.dynamic_angle_step_deg = MOTOR_ANGLE_STEP_NORMAL_DEG
        self.dynamic_motor_speed_rpm = MOTOR_POSITION_SPEED_NORMAL_RPM
        # 设置“负侧 -> 正侧目标”时锁存，目标再次改变后重新判断。
        self.neg_to_pos_soft_active = False
        # 本段的行进方向(物理 +x 为正)。0 = 没在跑分段行程。
        # 【不能每帧用 target-current 现算】球一旦冲过目标, 那个符号就翻转,
        # 于是"制动"被误判成"推动", 角度上限从制动档掉回推动档 —— 制动能力
        # 恰好在最需要的时候被砍掉一半。
        self.seg_drive_dir = 0.0
        # >0 时给本段的速度给定再加一道上限(第二段行程长, 要更慢)
        self.seg_speed_cap_px_s = 0.0
        # >0 时给本段推进方向单独设置物理加速度硬上限。
        self.seg_drive_accel_limit_cm_s2 = 0.0
        # 本段的收力区(cm)。<0 表示用全局 SEG_NEAR_ZONE_CM。
        self.seg_near_zone_cm = -1.0
        self.seg_stall_recovery_active = False
        self.seg_remaining_cm = 0.0
        self.seg_velocity_profile_active = False
        self.seg_profile_speed_cm_s = 0.0
        self.seg_stall_candidate_ms = None

    def reset(self):
        self.seg_drive_dir = 0.0
        # >0 时给本段的速度给定再加一道上限(第二段行程长, 要更慢)
        self.seg_speed_cap_px_s = 0.0
        self.seg_drive_accel_limit_cm_s2 = 0.0
        # 本段的收力区(cm)。<0 表示用全局 SEG_NEAR_ZONE_CM。
        self.seg_near_zone_cm = -1.0
        self.seg_stall_recovery_active = False
        self.seg_remaining_cm = 0.0
        self.seg_velocity_profile_active = False
        self.seg_profile_speed_cm_s = 0.0
        self.seg_stall_candidate_ms = None
        self.kalman.reset()
        self.last_measurement_px = None
        self.last_measurement_ms = None
        self.velocity_pid.reset()
        self.last_control_ms = None
        self.angle_command_deg = 0.0
        self.external_feedforward_deg = 0.0
        self.center_hold = False
        self.terminal_hold_active = False
        self.terminal_hold_bias_deg = CENTER_LEVEL_BIAS_DEG
        self.terminal_stall_candidate_ms = None
        self.terminal_stall_active = False
        self.terminal_stall_sign = 0.0
        self.terminal_stall_phase = 0
        self.terminal_stall_phase_started_ms = None
        self.center_candidate_ms = None
        self.center_bias_deg = CENTER_LEVEL_BIAS_DEG
        self.settled_state = False
        self.filtered_position_error_px = 0.0
        self.error_filter_ready = False
        self.fine_candidate_ms = None
        self.fine_active = False
        self.static_recovery_candidate_ms = None
        self.static_recovery_angle_deg = 0.0
        self.static_recovery_sign = 0.0
        self.static_recovery_last_kick_ms = None
        self.filtered_target_angle_deg = 0.0
        self.disturbance_until_ms = None
        self.last_abs_position_error_px = None
        self.last_motion_context_ms = None
        self.dynamic_urgency = 0.0
        self.dynamic_position_kp = POSITION_KP_NORMAL
        self.dynamic_velocity_kp = VELOCITY_KP_NORMAL
        self.dynamic_brake_active = False
        self.dynamic_disturbance_active = False
        self.dynamic_closing_velocity_px_s = 0.0
        self.dynamic_angle_step_deg = MOTOR_ANGLE_STEP_NORMAL_DEG
        self.dynamic_motor_speed_rpm = MOTOR_POSITION_SPEED_NORMAL_RPM
        # 设置“负侧 -> 正侧目标”时锁存，目标再次改变后重新判断。
        self.neg_to_pos_soft_active = False
        # 本段的行进方向(物理 +x 为正)。0 = 没在跑分段行程。
        # 【不能每帧用 target-current 现算】球一旦冲过目标, 那个符号就翻转,
        # 于是"制动"被误判成"推动", 角度上限从制动档掉回推动档 —— 制动能力
        # 恰好在最需要的时候被砍掉一半。
        self.seg_drive_dir = 0.0
        # >0 时给本段的速度给定再加一道上限(第二段行程长, 要更慢)
        self.seg_speed_cap_px_s = 0.0
        self.seg_drive_accel_limit_cm_s2 = 0.0
        # 本段的收力区(cm)。<0 表示用全局 SEG_NEAR_ZONE_CM。
        self.seg_near_zone_cm = -1.0

    def set_external_feedforward_deg(self, angle_deg):
        """设置车辆纵向加速度对应的外部前馈角，不改变球位置PID状态。"""
        self.external_feedforward_deg = clamp(
            float(angle_deg),
            -MOTOR_SOFT_LIMIT_DEG,
            MOTOR_SOFT_LIMIT_DEG,
        )

    def set_target_px(self, target_px):
        """搬动目标点。目标一变，围绕旧目标建立的所有锁存状态必须一起清掉。

        center_hold / fine / static_recovery 判的都是"相对目标"的误差，
        但它们是带驻留时间的锁存量：换目标时若不清，控制器会带着"我已经
        锁在中心了"的状态去追一个 30 像素外的新目标，输出被
        CENTER_HOLD_ANGLE_LIMIT_DEG 死死压在 ±8°，球根本走不动。

        center_bias_deg 保留 —— 它补偿的是机械零位不水平，与目标点无关。
        """
        target_px = clamp(target_px, AI_GUIDE_X1, AI_GUIDE_X2)

        # 根据设置目标这一刻的小球实际位置判断是否为 -5cm -> +5cm。
        # 全部使用物理厘米，因此不受 BALL_X_SIGN 的取值影响。
        target_cm = px_to_cm(target_px)
        current_cm = (
            px_to_cm(self.kalman.position)
            if self.kalman.ready
            else 0.0
        )
        # 分段限幅由任务层显式开关(见 _enter_seq3), 这里不再自动识别方向 ——
        # 自动识别只认"负侧->正侧", 第二段(正侧->负侧)根本盖不到。
        neg_to_pos_soft = self.neg_to_pos_soft_active

        if abs(target_px - self.target_px) <= 0.5:
            self.target_px = target_px
            return

        self.target_px = target_px
        self.neg_to_pos_soft_active = neg_to_pos_soft
        self.terminal_hold_active = False
        self.terminal_stall_candidate_ms = None
        self.terminal_stall_active = False
        self.terminal_stall_sign = 0.0
        self.terminal_stall_phase = 0
        self.terminal_stall_phase_started_ms = None
        self.center_hold = False
        self.center_candidate_ms = None
        self.fine_active = False
        self.fine_candidate_ms = None
        self.static_recovery_candidate_ms = None
        self.static_recovery_last_kick_ms = None
        self.static_recovery_angle_deg = 0.0
        self.static_recovery_sign = 0.0
        self.error_filter_ready = False
        self.velocity_pid.reset_integral()
        # 误差增长速度的历史基准也失效了，留着会在换目标那一拍算出一个
        # 巨大的 error_growth，被误判成惯性冲击。
        self.last_abs_position_error_px = None
        self.last_motion_context_ms = None

    def sync_motor_output_pulses(self, pulses):
        """任务回位完成后同步控制器内部角度，避免下一拍PID从旧角度跳变。"""
        angle_deg = int(pulses) * MOTOR_DEG_PER_PULSE
        self.angle_command_deg = angle_deg
        self.filtered_target_angle_deg = angle_deg
        self.last_control_ms = None
        self.velocity_pid.reset_integral()

    def correct_measurement(self, measured_position_px, measurement_ms, confidence):
        self.last_measurement_px = float(measured_position_px)
        self.last_measurement_ms = measurement_ms
        return self.kalman.correct(
            measured_position_px,
            measurement_ms,
            confidence,
        )

    def predicted_state(self, query_ms, include_motor_delay=False):
        extra_ms = 0.0
        if include_motor_delay:
            # 低速只做最小提前量，速度越快越接近最大提前量。
            motion_factor = clamp(abs(self.kalman.velocity) / 110.0, 0.0, 1.0)
            lookahead_ms = (
                CONTROL_LOOKAHEAD_MIN_MS
                + (CONTROL_LOOKAHEAD_MAX_MS - CONTROL_LOOKAHEAD_MIN_MS)
                * motion_factor
            )
            extra_ms = MOTOR_RESPONSE_DELAY_MS + lookahead_ms
        return self.kalman.predict(query_ms, extra_ms)

    def _update_center_state(self, position_error_px, velocity_px_s, now_ms):
        abs_error = abs(position_error_px)
        abs_velocity = abs(velocity_px_s)

        if self.center_hold:
            if self.terminal_hold_active:
                # 任务3已经按稳定判据完成，之后只允许终点小角度保持。
                # 即使误差短时超过普通2px退出门限，也不能重启大角度控制。
                return
            if (
                abs_error >= CENTER_EXIT_POSITION_PX
                or abs_velocity >= CENTER_EXIT_SPEED_PX_S
            ):
                self.center_hold = False
                self.center_candidate_ms = None
                self.velocity_pid.reset_integral()
            return

        if (
            abs_error <= CENTER_ENTER_POSITION_PX
            and abs_velocity <= CENTER_ENTER_SPEED_PX_S
        ):
            if self.center_candidate_ms is None:
                self.center_candidate_ms = now_ms
            elif (
                time.ticks_diff(now_ms, self.center_candidate_ms)
                >= CENTER_CAPTURE_DWELL_MS
            ):
                self.center_hold = True
                self.center_candidate_ms = None
                self.velocity_pid.reset_integral()
        else:
            self.center_candidate_ms = None

    def _update_center_bias(self, position_error_px, velocity_px_s, dt):
        """慢速学习机械平衡角，消除平台零位误差和静摩擦造成的中心静差。"""
        if not CENTER_BIAS_LEARN_ENABLE:
            return
        if (
            self.center_hold
            and not self.terminal_hold_active
            and abs(position_error_px) <= CENTER_BIAS_ACTIVE_POSITION_PX
            and abs(velocity_px_s) <= CENTER_BIAS_ACTIVE_SPEED_PX_S
        ):
            self.center_bias_deg += (
                CENTER_BIAS_LEARN_KI_DEG_PER_PX_S
                * position_error_px
                * dt
                * CONTROL_SIGN
            )
            self.center_bias_deg = clamp(
                self.center_bias_deg,
                -CENTER_BIAS_LIMIT_DEG,
                CENTER_BIAS_LIMIT_DEG,
            )

    def _adaptive_angle_limit(self, abs_position_error_px):
        """保留±20°硬限位，并按偏差分级开放，避免远端突然抽搐。"""
        e = abs_position_error_px
        if e <= ANGLE_LIMIT_NEAR_POSITION_PX:
            return min(ANGLE_LIMIT_NEAR_DEG, MOTOR_SOFT_LIMIT_DEG)
        if e <= ANGLE_LIMIT_MID_POSITION_PX:
            ratio = (
                (e - ANGLE_LIMIT_NEAR_POSITION_PX)
                / (ANGLE_LIMIT_MID_POSITION_PX - ANGLE_LIMIT_NEAR_POSITION_PX)
            )
            return min(
                ANGLE_LIMIT_NEAR_DEG
                + ratio * (ANGLE_LIMIT_MID_DEG - ANGLE_LIMIT_NEAR_DEG),
                MOTOR_SOFT_LIMIT_DEG,
            )
        if e <= ANGLE_LIMIT_FAR_POSITION_PX:
            ratio = (
                (e - ANGLE_LIMIT_MID_POSITION_PX)
                / (ANGLE_LIMIT_FAR_POSITION_PX - ANGLE_LIMIT_MID_POSITION_PX)
            )
            return min(
                ANGLE_LIMIT_MID_DEG
                + ratio * (ANGLE_LIMIT_FAR_DEG - ANGLE_LIMIT_MID_DEG),
                MOTOR_SOFT_LIMIT_DEG,
            )
        if e <= ANGLE_LIMIT_EMERGENCY_POSITION_PX:
            ratio = (
                (e - ANGLE_LIMIT_FAR_POSITION_PX)
                / (ANGLE_LIMIT_EMERGENCY_POSITION_PX - ANGLE_LIMIT_FAR_POSITION_PX)
            )
            return min(
                ANGLE_LIMIT_FAR_DEG
                + ratio * (MOTOR_SOFT_LIMIT_DEG - ANGLE_LIMIT_FAR_DEG),
                MOTOR_SOFT_LIMIT_DEG,
            )
        return MOTOR_SOFT_LIMIT_DEG

    def _update_fine_state(self, position_error_px, velocity_px_s, now_ms):
        """中点附近确认持续误差，避免对单帧噪声反复换向。"""
        abs_error = abs(position_error_px)
        abs_velocity = abs(velocity_px_s)

        if not self.center_hold:
            self.fine_active = False
            self.fine_candidate_ms = None
            return

        if self.fine_active:
            desired_sign = 1.0 if position_error_px >= 0.0 else -1.0
            closing_velocity = desired_sign * velocity_px_s
            if (
                abs_error <= CENTER_FINE_RELEASE_PX
                or abs_velocity > CENTER_FINE_MAX_SPEED_PX_S
                or closing_velocity >= CENTER_FINE_STOP_CLOSING_SPEED_PX_S
            ):
                self.fine_active = False
                self.fine_candidate_ms = None
            return

        if (
            abs_error >= CENTER_FINE_TRIGGER_PX
            and abs_velocity <= CENTER_FINE_MAX_SPEED_PX_S
        ):
            if self.fine_candidate_ms is None:
                self.fine_candidate_ms = now_ms
            elif (
                time.ticks_diff(now_ms, self.fine_candidate_ms)
                >= CENTER_FINE_DWELL_MS
            ):
                self.fine_active = True
                self.fine_candidate_ms = None
        else:
            self.fine_candidate_ms = None

    def _update_static_recovery(
        self, position_error_px, velocity_px_s, dt, now_ms,
    ):
        """锁存式静差恢复。

        普通PD/速度环在静摩擦、连杆间隙或大减速比下可能形成固定静差。
        本函数在误差持续且钢球低速时逐步增加倾角；倾角建立后保持，
        不会因为钢球刚开始移动就立即撤销，直到真正接近中点才释放。
        """
        if not STATIC_RECOVERY_ENABLE or self.center_hold:
            self.static_recovery_candidate_ms = None
            self.static_recovery_last_kick_ms = None
            decay = STATIC_RECOVERY_FAST_DECAY_DEG_S * dt
            if self.static_recovery_angle_deg > 0.0:
                self.static_recovery_angle_deg = max(
                    0.0, self.static_recovery_angle_deg - decay
                )
            elif self.static_recovery_angle_deg < 0.0:
                self.static_recovery_angle_deg = min(
                    0.0, self.static_recovery_angle_deg + decay
                )
            if abs(self.static_recovery_angle_deg) < 0.01:
                self.static_recovery_angle_deg = 0.0
                self.static_recovery_sign = 0.0
            return self.static_recovery_angle_deg

        abs_error = abs(position_error_px)
        if abs_error <= STATIC_RECOVERY_RESET_PX:
            self.static_recovery_candidate_ms = None
            self.static_recovery_last_kick_ms = None
            self.static_recovery_angle_deg = 0.0
            self.static_recovery_sign = 0.0
            return 0.0

        desired_sign = 1.0 if position_error_px >= 0.0 else -1.0

        # 穿过目标中点后，清除旧方向补偿，防止反向残留继续推球。
        if (
            self.static_recovery_sign != 0.0
            and desired_sign != self.static_recovery_sign
            and abs_error >= STATIC_RECOVERY_SIGN_HYSTERESIS_PX
        ):
            self.static_recovery_angle_deg = 0.0
            self.static_recovery_candidate_ms = None
            self.static_recovery_last_kick_ms = None

        self.static_recovery_sign = desired_sign
        closing_velocity = desired_sign * velocity_px_s

        # 误差越大，允许的持续补偿越大。图片所示约几十像素静差时，
        # 上限可开放到几十度，而不是被旧版约十几度上限卡住。
        error_limit = clamp(
            STATIC_RECOVERY_MIN_ANGLE_DEG
            + STATIC_RECOVERY_ERROR_TO_LIMIT_DEG_PER_PX
            * max(0.0, abs_error - STATIC_RECOVERY_TRIGGER_PX),
            STATIC_RECOVERY_MIN_ANGLE_DEG,
            min(STATIC_RECOVERY_MAX_ANGLE_DEG, MOTOR_SOFT_LIMIT_DEG),
        )

        persistent_error = abs_error >= STATIC_RECOVERY_TRIGGER_PX
        nearly_still = abs(velocity_px_s) <= STATIC_RECOVERY_MAX_STILL_SPEED_PX_S

        if persistent_error and nearly_still:
            if self.static_recovery_candidate_ms is None:
                self.static_recovery_candidate_ms = now_ms
                self.static_recovery_last_kick_ms = now_ms
            elif (
                time.ticks_diff(now_ms, self.static_recovery_candidate_ms)
                >= STATIC_RECOVERY_DWELL_MS
            ):
                magnitude = abs(self.static_recovery_angle_deg)
                if magnitude < STATIC_RECOVERY_MIN_ANGLE_DEG:
                    magnitude = STATIC_RECOVERY_MIN_ANGLE_DEG
                else:
                    magnitude += STATIC_RECOVERY_RAMP_DEG_S * dt

                # 若持续静止，每隔一段时间增加一个小台阶，帮助跨过传动死区。
                if (
                    self.static_recovery_last_kick_ms is None
                    or time.ticks_diff(now_ms, self.static_recovery_last_kick_ms)
                    >= STATIC_RECOVERY_KICK_INTERVAL_MS
                ):
                    magnitude += STATIC_RECOVERY_KICK_STEP_DEG
                    self.static_recovery_last_kick_ms = now_ms

                self.static_recovery_angle_deg = (
                    desired_sign * min(magnitude, error_limit)
                )
        else:
            self.static_recovery_candidate_ms = None

            current_magnitude = abs(self.static_recovery_angle_deg)
            if abs_error <= STATIC_RECOVERY_HOLD_UNTIL_PX:
                # 已进入中点附近，快速释放，交给精细保持控制。
                decay = STATIC_RECOVERY_FAST_DECAY_DEG_S * dt
                current_magnitude = max(0.0, current_magnitude - decay)
            elif closing_velocity >= STATIC_RECOVERY_RELEASE_SPEED_PX_S:
                # 高速接近中心时适度释放，避免大惯性冲过中点。
                decay = STATIC_RECOVERY_FAST_DECAY_DEG_S * dt
                current_magnitude = max(0.0, current_magnitude - decay)
            elif persistent_error and closing_velocity > 0.0:
                # 已经开始朝中心移动，但仍离中心较远：至少保留大部分倾角，
                # 防止一移动就撤力，最终又停回图片中的固定偏置位置。
                hold_floor = min(
                    error_limit,
                    max(
                        STATIC_RECOVERY_MIN_ANGLE_DEG,
                        abs(self.static_recovery_angle_deg)
                        * STATIC_RECOVERY_MIN_HOLD_RATIO,
                    ),
                )
                current_magnitude = max(
                    hold_floor,
                    current_magnitude - STATIC_RECOVERY_DECAY_DEG_S * dt,
                )
            elif not persistent_error:
                current_magnitude = max(
                    0.0,
                    current_magnitude - STATIC_RECOVERY_FAST_DECAY_DEG_S * dt,
                )

            current_magnitude = min(current_magnitude, error_limit)
            self.static_recovery_angle_deg = desired_sign * current_magnitude

        return self.static_recovery_angle_deg

    def _update_motion_context(
        self, position_error_px, velocity_px_s, acceleration_px_s2, now_ms,
    ):
        """根据位置、速度、加速度识别惯性外冲，并生成0~1紧迫度。"""
        abs_error = abs(position_error_px)
        error_sign = 1.0 if position_error_px >= 0.0 else -1.0
        closing_velocity = error_sign * velocity_px_s
        closing_acceleration = error_sign * acceleration_px_s2

        if self.last_motion_context_ms is None:
            error_growth_speed = 0.0
        else:
            context_dt = clamp(
                time.ticks_diff(now_ms, self.last_motion_context_ms) / 1000.0,
                0.005,
                0.100,
            )
            previous_error = (
                abs_error
                if self.last_abs_position_error_px is None
                else self.last_abs_position_error_px
            )
            error_growth_speed = (abs_error - previous_error) / context_dt

        self.last_abs_position_error_px = abs_error
        self.last_motion_context_ms = now_ms

        away_speed_factor = clamp(
            max(0.0, -closing_velocity) / INERTIA_SPEED_TRIGGER_PX_S,
            0.0,
            1.6,
        )
        away_accel_factor = clamp(
            max(0.0, -closing_acceleration) / INERTIA_ACCEL_TRIGGER_PX_S2,
            0.0,
            1.6,
        )
        growth_factor = clamp(
            max(0.0, error_growth_speed)
            / INERTIA_ERROR_GROWTH_TRIGGER_PX_S,
            0.0,
            1.6,
        )
        impact_strength = max(
            away_speed_factor, away_accel_factor, growth_factor
        )

        if (
            abs_error >= INERTIA_MIN_POSITION_ERROR_PX
            and impact_strength >= 1.0
        ):
            self.disturbance_until_ms = ticks_shift(now_ms, INERTIA_HOLD_MS)

        disturbance_active = (
            self.disturbance_until_ms is not None
            and time.ticks_diff(self.disturbance_until_ms, now_ms) > 0
        )

        distance_factor = clamp(abs_error / 42.0, 0.0, 1.0)
        speed_factor = clamp(abs(velocity_px_s) / 150.0, 0.0, 1.0)
        impact_factor = clamp(impact_strength, 0.0, 1.0)
        urgency = clamp(
            max(
                0.45 * distance_factor + 0.35 * speed_factor,
                impact_factor,
                0.72 if disturbance_active else 0.0,
            ),
            0.0,
            1.0,
        )

        self.dynamic_urgency = urgency
        self.dynamic_disturbance_active = disturbance_active
        self.dynamic_closing_velocity_px_s = closing_velocity
        return (
            error_sign,
            closing_velocity,
            closing_acceleration,
            distance_factor,
            speed_factor,
            impact_factor,
            disturbance_active,
            urgency,
        )

    def _outer_velocity_reference(
        self, position_error_px, velocity_px_s, motion_context,
    ):
        if CONTROL_MODE == MODE_VELOCITY_TUNE:
            self.dynamic_position_kp = POSITION_KP_NORMAL
            self.dynamic_brake_active = False
            return VELOCITY_TUNE_SETPOINT_PX_S

        if self.center_hold:
            self.dynamic_position_kp = CENTER_HOLD_POSITION_KP
            self.dynamic_brake_active = False
            velocity_reference = (
                CENTER_HOLD_POSITION_KP * position_error_px
                - CENTER_HOLD_VELOCITY_KD * velocity_px_s
            )
            return clamp(
                velocity_reference,
                -CENTER_HOLD_MAX_VELOCITY_PX_S,
                CENTER_HOLD_MAX_VELOCITY_PX_S,
            )

        (
            error_sign, closing_velocity, _closing_acceleration,
            distance_factor, _speed_factor, impact_factor,
            disturbance_active, _urgency,
        ) = motion_context
        distance = abs(position_error_px)

        # 远距离提高P；中心附近降低P并提高速度阻尼。
        kp = (
            POSITION_KP_NEAR
            + (POSITION_KP_FAR - POSITION_KP_NEAR) * distance_factor
        )
        if disturbance_active:
            kp *= 1.0 + POSITION_KP_DISTURBANCE_BOOST * max(0.45, impact_factor)
        kd = (
            POSITION_KD_NEAR
            + (POSITION_KD_FAR - POSITION_KD_NEAR) * distance_factor
        )
        self.dynamic_position_kp = kp

        raw_reference = kp * position_error_px - kd * velocity_px_s

        # 小球正在向远离中点的方向运动时，临时提高回中速度给定。
        away_speed = max(0.0, -closing_velocity)
        if away_speed > 0.0:
            raw_reference += error_sign * min(
                POSITION_AWAY_BOOST_LIMIT_PX_S,
                POSITION_AWAY_VELOCITY_BOOST * away_speed,
            )

        brake_accel = (
            POSITION_BRAKE_ACCEL_NEAR_PX_S2
            + (POSITION_BRAKE_ACCEL_FAR_PX_S2
               - POSITION_BRAKE_ACCEL_NEAR_PX_S2) * distance_factor
        )
        speed_profile_limit = math.sqrt(
            max(0.0, 2.0 * brake_accel * distance)
        )
        max_speed = (
            MAX_TARGET_VELOCITY_NEAR_PX_S
            + (MAX_TARGET_VELOCITY_FAR_PX_S
               - MAX_TARGET_VELOCITY_NEAR_PX_S) * distance_factor
        )
        speed_limit = min(max_speed, speed_profile_limit)
        if self.seg_speed_cap_px_s > 0.0:
            speed_limit = min(speed_limit, self.seg_speed_cap_px_s)

        # 预计刹停距离已经接近/超过剩余距离时，外环不再继续催球，
        # 而是给0或小幅反向速度，让速度内环提前输出制动倾角。
        brake_active = False
        if closing_velocity >= POSITION_BRAKE_MIN_CLOSING_SPEED_PX_S:
            stopping_distance = (
                closing_velocity * closing_velocity
                / max(1.0, 2.0 * brake_accel)
            )
            brake_threshold = max(0.6, distance * POSITION_BRAKE_MARGIN)
            if stopping_distance >= brake_threshold:
                brake_active = True
                overshoot = clamp(
                    (stopping_distance - brake_threshold)
                    / max(1.0, brake_threshold),
                    0.0,
                    1.0,
                )
                reverse_reference = -error_sign * (
                    POSITION_REVERSE_BRAKE_MAX_PX_S * overshoot
                )
                raw_reference = (
                    raw_reference * (1.0 - overshoot)
                    + reverse_reference * overshoot
                )

        self.dynamic_brake_active = brake_active
        return clamp(raw_reference, -speed_limit, speed_limit)

    def update_control(self, now_ms):
        predicted = self.predicted_state(now_ms, include_motor_delay=True)
        if predicted is None:
            return None

        position_px, velocity_px_s, prediction_ms = predicted
        if self.last_control_ms is None:
            dt = 1.0 / 50.0
        else:
            dt = clamp(
                time.ticks_diff(now_ms, self.last_control_ms) / 1000.0,
                0.005,
                0.100,
            )
        self.last_control_ms = now_ms

        raw_position_error_px = self.target_px - position_px
        if not self.error_filter_ready:
            self.filtered_position_error_px = raw_position_error_px
            self.error_filter_ready = True
        else:
            self.filtered_position_error_px += CENTER_ERROR_LPF_ALPHA * (
                raw_position_error_px - self.filtered_position_error_px
            )

        # 远离中心时使用原始误差保证响应；中点附近使用滤波误差抑制识别噪声。
        if abs(raw_position_error_px) > MOTOR_NEAR_CENTER_POSITION_PX:
            position_error_px = raw_position_error_px
        else:
            position_error_px = self.filtered_position_error_px

        self._update_center_state(position_error_px, velocity_px_s, now_ms)
        self._update_fine_state(position_error_px, velocity_px_s, now_ms)

        motion_context = self._update_motion_context(
            position_error_px,
            velocity_px_s,
            self.kalman.acceleration,
            now_ms,
        )
        velocity_reference_px_s = self._outer_velocity_reference(
            position_error_px,
            velocity_px_s,
            motion_context,
        )

        # 任务3第二段使用独立梯形速度规划。旧外环会随位置误差、
        # 估计速度和刹车标志来回改变给定，实机就表现为“走一点、
        # 刹一下、再走”。这里只按剩余距离生成速度：前半段匀加速，
        # 中段保持，终点前用 v=sqrt(2*a*s) 连续减速。
        if (
            self.seg_velocity_profile_active
            and self.seg_drive_dir != 0.0
            and not self.center_hold
        ):
            current_cm = px_to_cm(position_px)
            target_cm = px_to_cm(self.target_px)
            signed_remaining_cm = self.seg_drive_dir * (
                target_cm - current_cm
            )
            physical_velocity_cm_s = px_s_to_cm_s(velocity_px_s)
            closing_speed_cm_s = (
                self.seg_drive_dir * physical_velocity_cm_s
            )

            if signed_remaining_cm >= 0.0:
                brake_distance_cm = max(
                    0.0,
                    signed_remaining_cm - SEG_LEG2_PROFILE_STOP_MARGIN_CM,
                )
                brake_speed_cm_s = math.sqrt(max(
                    0.0,
                    2.0 * SEG_LEG2_PROFILE_DECEL_CM_S2
                    * brake_distance_cm,
                ))
                desired_speed_cm_s = min(
                    SEG_LEG2_PROFILE_MAX_CM_S,
                    brake_speed_cm_s,
                )
                previous_speed_cm_s = self.seg_profile_speed_cm_s
                if desired_speed_cm_s >= previous_speed_cm_s:
                    self.seg_profile_speed_cm_s = min(
                        desired_speed_cm_s,
                        previous_speed_cm_s
                        + SEG_LEG2_PROFILE_ACCEL_CM_S2 * dt,
                    )
                else:
                    self.seg_profile_speed_cm_s = max(
                        desired_speed_cm_s,
                        previous_speed_cm_s
                        - SEG_LEG2_PROFILE_DECEL_CM_S2 * dt,
                    )
                physical_reference_cm_s = (
                    self.seg_drive_dir * self.seg_profile_speed_cm_s
                )
                if (
                    desired_speed_cm_s < previous_speed_cm_s - 0.02
                    or closing_speed_cm_s
                    > self.seg_profile_speed_cm_s + 0.50
                ):
                    self.dynamic_brake_active = True
            else:
                # 允许小幅越过-5cm，但不立刻用大给定反冲。先把原方向
                # 速度刹住，若仍未被终点保持捕获，再以最多2cm/s慢速回拉。
                self.dynamic_brake_active = True
                if closing_speed_cm_s > 0.50:
                    self.seg_profile_speed_cm_s = 0.0
                    physical_reference_cm_s = 0.0
                else:
                    overshoot_cm = -signed_remaining_cm
                    return_speed_cm_s = min(
                        SEG_LEG2_OVERSHOOT_RETURN_MAX_CM_S,
                        math.sqrt(max(
                            0.0,
                            2.0 * SEG_LEG2_PROFILE_DECEL_CM_S2
                            * overshoot_cm,
                        )),
                    )
                    self.seg_profile_speed_cm_s = min(
                        return_speed_cm_s,
                        self.seg_profile_speed_cm_s
                        + SEG_LEG2_PROFILE_ACCEL_CM_S2 * dt,
                    )
                    physical_reference_cm_s = (
                        -self.seg_drive_dir * self.seg_profile_speed_cm_s
                    )

            velocity_reference_px_s = cm_s_to_px_s(
                physical_reference_cm_s
            )
        velocity_error_px_s = velocity_reference_px_s - velocity_px_s

        self._update_center_bias(position_error_px, velocity_px_s, dt)
        if self.seg_velocity_profile_active:
            # 通用静差恢复会在120ms后不断累加倾角，会把上面的
            # 速度规划再次变成全力推进。规划段只保留末端专用防卡。
            self.static_recovery_candidate_ms = None
            self.static_recovery_last_kick_ms = None
            self.static_recovery_angle_deg = 0.0
            self.static_recovery_sign = 0.0
            static_recovery_deg = 0.0
        else:
            static_recovery_deg = self._update_static_recovery(
                position_error_px, velocity_px_s, dt, now_ms,
            )

        settled = (
            self.center_hold
            and abs(position_error_px) <= CENTER_SETTLED_POSITION_PX
            and abs(velocity_px_s) <= CENTER_SETTLED_SPEED_PX_S
            and not self.fine_active
        )

        position_angle_assist_deg = 0.0
        final_approach_deg = 0.0
        velocity_feedforward_deg = 0.0
        acceleration_damping_deg = 0.0

        if settled:
            base_angle_deg = (
                self.terminal_hold_bias_deg
                if self.terminal_hold_active else self.center_bias_deg
            )
            if not self.settled_state:
                self.velocity_pid.reset()
        elif self.center_hold:
            # 中点附近不用高速PID连续追逐噪声。只有持续误差成立后，
            # 才给一个方向稳定、足以克服静摩擦的最小倾角。
            self.velocity_pid.reset_integral()
            if self.terminal_hold_active:
                # 任务3完成态必须继续闭环。位置误差决定回拉方向，速度项负责
                # 刹车；基准角来自起跑前O点静止时的实际角度，补偿机械零偏。
                position_error_cm = px_to_cm(self.target_px) - px_to_cm(position_px)
                velocity_cm_s = px_s_to_cm_s(velocity_px_s)
                error_sign = 1.0 if position_error_cm >= 0.0 else -1.0
                abs_error_cm = abs(position_error_cm)
                closing_speed_cm_s = error_sign * velocity_cm_s

                if self.terminal_stall_active:
                    if (
                        error_sign != self.terminal_stall_sign
                        or abs_error_cm <= TERMINAL_HOLD_STALL_RELEASE_ERROR_CM
                        or closing_speed_cm_s
                        >= TERMINAL_HOLD_STALL_RELEASE_SPEED_CM_S
                    ):
                        self.terminal_stall_active = False
                        self.terminal_stall_candidate_ms = None
                        self.terminal_stall_sign = error_sign
                        self.terminal_stall_phase = 0
                        self.terminal_stall_phase_started_ms = None
                    elif self.terminal_stall_phase == 1:
                        if (
                            self.terminal_stall_phase_started_ms is None
                            or time.ticks_diff(
                                now_ms, self.terminal_stall_phase_started_ms,
                            ) >= TERMINAL_HOLD_STALL_UNLOAD_MS
                        ):
                            self.terminal_stall_phase = 2
                            self.terminal_stall_phase_started_ms = now_ms
                    elif (
                        self.terminal_stall_phase == 2
                        and self.terminal_stall_phase_started_ms is not None
                        and time.ticks_diff(
                            now_ms, self.terminal_stall_phase_started_ms,
                        ) >= TERMINAL_HOLD_STALL_DRIVE_MS
                        and abs(velocity_cm_s)
                        <= TERMINAL_HOLD_STALL_SPEED_CM_S
                    ):
                        # 负向持续顶住仍不动，说明传动间隙又锁在同一侧；
                        # 回到卸载相位，形成温和的“松一下再推”循环。
                        self.terminal_stall_phase = 1
                        self.terminal_stall_phase_started_ms = now_ms
                elif (
                    abs_error_cm >= TERMINAL_HOLD_STALL_ERROR_CM
                    and abs(velocity_cm_s) <= TERMINAL_HOLD_STALL_SPEED_CM_S
                ):
                    if (
                        self.terminal_stall_candidate_ms is None
                        or error_sign != self.terminal_stall_sign
                    ):
                        self.terminal_stall_candidate_ms = now_ms
                        self.terminal_stall_sign = error_sign
                    elif (
                        time.ticks_diff(now_ms, self.terminal_stall_candidate_ms)
                        >= TERMINAL_HOLD_STALL_DWELL_MS
                    ):
                        self.terminal_stall_active = True
                        self.terminal_stall_phase = 1
                        self.terminal_stall_phase_started_ms = now_ms
                else:
                    self.terminal_stall_candidate_ms = None
                    self.terminal_stall_sign = error_sign
                    self.terminal_stall_phase = 0
                    self.terminal_stall_phase_started_ms = None

                if self.terminal_stall_active:
                    if self.terminal_stall_phase == 1:
                        # 先沿相反方向卸掉丝杆/连杆回差。这里必须是绝对反向角：
                        # 若叠加 terminal_hold_bias，当启动基准为-4.8deg时，
                        # 所谓3deg卸载实际仍是-1.8deg，根本没有跨过回差。
                        base_angle_deg = clamp(
                            -error_sign * CONTROL_SIGN
                            * TERMINAL_HOLD_STALL_UNLOAD_DEG,
                            -TERMINAL_HOLD_TOTAL_LIMIT_DEG,
                            TERMINAL_HOLD_TOTAL_LIMIT_DEG,
                        )
                    else:
                        base_angle_deg = clamp(
                            error_sign * CONTROL_SIGN
                            * TERMINAL_HOLD_STALL_KICK_DEG,
                            -TERMINAL_HOLD_TOTAL_LIMIT_DEG,
                            TERMINAL_HOLD_TOTAL_LIMIT_DEG,
                        )
                else:
                    terminal_correction_deg = clamp(
                        TERMINAL_HOLD_KP_DEG_PER_CM * position_error_cm
                        - TERMINAL_HOLD_KD_DEG_PER_CM_S * velocity_cm_s,
                        -TERMINAL_HOLD_CORRECTION_LIMIT_DEG,
                        TERMINAL_HOLD_CORRECTION_LIMIT_DEG,
                    )
                    base_angle_deg = clamp(
                        self.terminal_hold_bias_deg + terminal_correction_deg,
                        -TERMINAL_HOLD_TOTAL_LIMIT_DEG,
                        TERMINAL_HOLD_TOTAL_LIMIT_DEG,
                    )
            elif self.fine_active:
                desired_sign = 1.0 if position_error_px >= 0.0 else -1.0
                fine_magnitude = clamp(
                    CENTER_FINE_MIN_ANGLE_DEG
                    + CENTER_FINE_KP_DEG_PER_PX
                    * max(0.0, abs(position_error_px) - CENTER_FINE_TRIGGER_PX),
                    CENTER_FINE_MIN_ANGLE_DEG,
                    CENTER_FINE_MAX_ANGLE_DEG,
                )
                position_angle_assist_deg = desired_sign * fine_magnitude
                base_angle_deg = (
                    self.center_bias_deg
                    + CONTROL_SIGN * position_angle_assist_deg
                )
            else:
                # 等待误差确认或小球已经开始朝中点运动时，保持当前平衡角。
                base_angle_deg = self.center_bias_deg
        else:
            abs_error = abs(position_error_px)
            near_factor = 1.0 - clamp(abs_error / 9.0, 0.0, 1.0)
            disturbance_factor = (
                self.dynamic_urgency
                if self.dynamic_disturbance_active
                else 0.0
            )
            velocity_error_factor = clamp(
                abs(velocity_error_px_s) / 155.0, 0.0, 1.0
            )
            gain_factor = max(disturbance_factor, velocity_error_factor)

            # 动态内环：中心附近柔和；外冲/大速度误差时提高P并暂停积分。
            velocity_kp = (
                VELOCITY_KP_NORMAL
                + (VELOCITY_KP_DISTURBANCE - VELOCITY_KP_NORMAL)
                * gain_factor
            )
            velocity_kp += (
                VELOCITY_KP_NEAR - VELOCITY_KP_NORMAL
            ) * near_factor * (1.0 - gain_factor)
            velocity_ki = (
                VELOCITY_KI_NEAR * near_factor
                + VELOCITY_KI_NORMAL * (1.0 - near_factor)
            )
            velocity_ki *= (1.0 - 0.92 * disturbance_factor)
            velocity_kd = (
                VELOCITY_KD_NORMAL
                + (VELOCITY_KD_NEAR - VELOCITY_KD_NORMAL) * near_factor
            )
            velocity_kd += (
                VELOCITY_KD_DISTURBANCE - velocity_kd
            ) * disturbance_factor
            self.dynamic_velocity_kp = velocity_kp

            integrate = (
                not self.dynamic_disturbance_active
                and abs(velocity_error_px_s) <= VELOCITY_I_ACTIVE_ERROR_PX_S
            )
            integral_leak = (
                VELOCITY_INTEGRAL_LEAK_DISTURBANCE
                if self.dynamic_disturbance_active
                else VELOCITY_INTEGRAL_LEAK_NORMAL
            )
            pid_angle_deg = self.velocity_pid.update(
                velocity_error_px_s,
                dt,
                integrate=integrate,
                integral_leak=integral_leak,
                kp=velocity_kp,
                ki=velocity_ki,
                kd=velocity_kd,
            )

            velocity_feedforward_deg = clamp(
                VELOCITY_FEEDFORWARD_K_DEG_PER_PX_S
                * velocity_reference_px_s,
                -VELOCITY_FEEDFORWARD_LIMIT_DEG,
                VELOCITY_FEEDFORWARD_LIMIT_DEG,
            )
            accel_gain = (
                BALL_ACCEL_DAMP_K_NORMAL
                + (BALL_ACCEL_DAMP_K_DISTURBANCE
                   - BALL_ACCEL_DAMP_K_NORMAL)
                * disturbance_factor
            )
            acceleration_damping_deg = clamp(
                -accel_gain * self.kalman.acceleration,
                -BALL_ACCEL_DAMP_LIMIT_DEG,
                BALL_ACCEL_DAMP_LIMIT_DEG,
            )

            # 远离中心时保留位置助推，但其强度仍由实际回中速度动态衰减。
            if abs(position_error_px) >= MOTOR_ACTIVE_POSITION_ERROR_PX:
                desired_sign = 1.0 if position_error_px >= 0.0 else -1.0
                closing_velocity = desired_sign * velocity_px_s
                requested_speed = max(1.0, abs(velocity_reference_px_s))
                assist_factor = clamp(
                    (requested_speed - closing_velocity) / requested_speed,
                    0.0,
                    1.0,
                )
                assist_limit_deg = POSITION_ANGLE_ASSIST_LIMIT_DEG
                if self.seg_velocity_profile_active:
                    # 位置助推原本会在远端立刻饱和到10度，完全绕过
                    # 梯形速度的逐渐加速。助推上限改为跟规划速度同步开放。
                    assist_limit_deg *= clamp(
                        self.seg_profile_speed_cm_s
                        / max(SEG_LEG2_PROFILE_MAX_CM_S, 1e-6),
                        0.0,
                        1.0,
                    )
                position_angle_assist_deg = clamp(
                    POSITION_ANGLE_ASSIST_KP
                    * position_error_px
                    * assist_factor,
                    -assist_limit_deg,
                    assist_limit_deg,
                )

            base_angle_deg = (
                pid_angle_deg
                + velocity_feedforward_deg
                + acceleration_damping_deg
                + position_angle_assist_deg
                + static_recovery_deg
            ) * CONTROL_SIGN + self.center_bias_deg

            # 明显偏离且控制输出仍小于静摩擦阈值时，至少给定有效角度。
            if (
                abs(position_error_px) >= MOTOR_ACTIVE_POSITION_ERROR_PX
                and abs(velocity_error_px_s) >= MOTOR_ACTIVE_VELOCITY_ERROR_PX_S
            ):
                required_sign = (
                    1.0 if velocity_error_px_s >= 0.0 else -1.0
                ) * CONTROL_SIGN
                if (
                    base_angle_deg * required_sign >= 0.0
                    and abs(base_angle_deg - self.center_bias_deg)
                    < MOTOR_MIN_ACTIVE_ANGLE_DEG
                ):
                    base_angle_deg = (
                        required_sign * MOTOR_MIN_ACTIVE_ANGLE_DEG
                        + self.center_bias_deg
                    )

            # 最后10像素持续推进：只保证最小有效角度，不与原控制量重复叠加。
            # 钢球已经以较高速度朝中点移动时不强制，防止冲过目标。
            abs_error = abs(position_error_px)
            if (
                FINAL_APPROACH_ENABLE
                and FINAL_APPROACH_RELEASE_PX < abs_error <= FINAL_APPROACH_ZONE_PX
            ):
                error_sign = 1.0 if position_error_px >= 0.0 else -1.0
                closing_velocity = error_sign * velocity_px_s
                if closing_velocity < FINAL_APPROACH_MAX_CLOSING_SPEED_PX_S:
                    final_magnitude = clamp(
                        FINAL_APPROACH_MIN_ANGLE_DEG
                        + FINAL_APPROACH_KP_DEG_PER_PX
                        * max(0.0, abs_error - FINAL_APPROACH_RELEASE_PX),
                        FINAL_APPROACH_MIN_ANGLE_DEG,
                        FINAL_APPROACH_MAX_ANGLE_DEG,
                    )
                    motor_sign = error_sign * CONTROL_SIGN
                    current_correction = base_angle_deg - self.center_bias_deg
                    if current_correction * motor_sign < final_magnitude:
                        final_approach_deg = motor_sign * final_magnitude
                        base_angle_deg = self.center_bias_deg + final_approach_deg

        if self.center_hold:
            terminal_stall_output = (
                self.terminal_hold_active and self.terminal_stall_phase in (1, 2)
            )
            correction_deg = clamp(
                base_angle_deg - self.center_bias_deg,
                -(
                    TERMINAL_HOLD_TOTAL_LIMIT_DEG
                ) if terminal_stall_output else -(
                    TERMINAL_HOLD_CORRECTION_LIMIT_DEG
                    + abs(self.terminal_hold_bias_deg - self.center_bias_deg)
                ) if self.terminal_hold_active else -CENTER_HOLD_ANGLE_LIMIT_DEG,
                (
                    TERMINAL_HOLD_TOTAL_LIMIT_DEG
                ) if terminal_stall_output else (
                    TERMINAL_HOLD_CORRECTION_LIMIT_DEG
                    + abs(self.terminal_hold_bias_deg - self.center_bias_deg)
                ) if self.terminal_hold_active else CENTER_HOLD_ANGLE_LIMIT_DEG,
            )
            target_angle_deg = clamp(
                self.center_bias_deg + correction_deg,
                -TERMINAL_HOLD_TOTAL_LIMIT_DEG if self.terminal_hold_active else -MOTOR_SOFT_LIMIT_DEG,
                TERMINAL_HOLD_TOTAL_LIMIT_DEG if self.terminal_hold_active else MOTOR_SOFT_LIMIT_DEG,
            )
            angle_limit = min(
                MOTOR_SOFT_LIMIT_DEG,
                TERMINAL_HOLD_TOTAL_LIMIT_DEG if self.terminal_hold_active
                else abs(self.center_bias_deg) + CENTER_HOLD_ANGLE_LIMIT_DEG,
            )
            max_angle_step = (
                TERMINAL_HOLD_STALL_ANGLE_STEP_DEG
                if terminal_stall_output else TERMINAL_HOLD_ANGLE_STEP_DEG
                if self.terminal_hold_active else CENTER_HOLD_ANGLE_STEP_DEG
            )
            target_alpha = (
                TERMINAL_HOLD_TARGET_ALPHA
                if self.terminal_hold_active else (
                    ANGLE_TARGET_ALPHA_SETTLED
                    if settled else ANGLE_TARGET_ALPHA_NEAR
                )
            )
        else:
            angle_limit = self._adaptive_angle_limit(abs(position_error_px))
            target_angle_deg = clamp(base_angle_deg, -angle_limit, angle_limit)

            if self.dynamic_disturbance_active:
                max_angle_step = MOTOR_ANGLE_STEP_DISTURBANCE_DEG
                target_alpha = ANGLE_TARGET_ALPHA_DISTURBANCE
            elif abs(position_error_px) <= MOTOR_NEAR_CENTER_POSITION_PX:
                max_angle_step = MOTOR_ANGLE_STEP_NEAR_DEG
                target_alpha = ANGLE_TARGET_ALPHA_NEAR
            else:
                max_angle_step = MOTOR_ANGLE_STEP_NORMAL_DEG
                target_alpha = ANGLE_TARGET_ALPHA_NORMAL

            # 高速接近中心且预计刹车不足时，允许较快反向制动；
            # 普通符号翻转则限制跨零速度，避免FD目标左右抽搐。
            current_correction = self.angle_command_deg - self.center_bias_deg
            target_correction = target_angle_deg - self.center_bias_deg
            reversing = (
                current_correction * target_correction < 0.0
                and abs(current_correction) >= ANGLE_REVERSAL_GUARD_DEG
            )
            if reversing:
                reversal_limit = (
                    MOTOR_ANGLE_STEP_BRAKE_REVERSAL_DEG
                    if self.dynamic_brake_active
                    else MOTOR_ANGLE_STEP_REVERSAL_DEG
                )
                max_angle_step = min(max_angle_step, reversal_limit)

        self.seg_stall_recovery_active = False
        self.seg_remaining_cm = 0.0

        # 项3 的两段行程都走这里：推动方向限幅小、制动方向留足。
        if self.neg_to_pos_soft_active and not self.center_hold:
            current_cm = px_to_cm(position_px)
            target_cm = px_to_cm(self.target_px)
            remaining_cm = abs(target_cm - current_cm)
            self.seg_remaining_cm = remaining_cm
            near_zone = (
                self.seg_near_zone_cm
                if self.seg_near_zone_cm >= 0.0
                else SEG_NEAR_ZONE_CM
            )
            if near_zone <= 0.0:
                near_factor = 0.0          # 这一段不收力
            else:
                near_factor = clamp(
                    (near_zone - remaining_cm) / max(0.01, near_zone),
                    0.0,
                    1.0,
                )
            # 真卡住了(停住 + 离目标还有一段)就把推力放回去; 否则按距离收力。
            stalled = (
                abs(velocity_px_s) < SEG_COAST_SPEED_PX_S
                and remaining_cm > SEG_STALL_MIN_CM
            )
            drive_far_cap_deg = SEG_DRIVE_MAX_ANGLE_DEG
            if self.seg_drive_accel_limit_cm_s2 > 0.0:
                drive_far_cap_deg = min(
                    drive_far_cap_deg,
                    accel_to_motor_deg(self.seg_drive_accel_limit_cm_s2),
                )
            drive_near_cap_deg = min(
                SEG_DRIVE_NEAR_ANGLE_DEG, drive_far_cap_deg,
            )
            if stalled:
                drive_cap_deg = drive_far_cap_deg
            else:
                drive_cap_deg = (
                    drive_far_cap_deg
                    + (drive_near_cap_deg
                       - drive_far_cap_deg) * near_factor
                )

            correction_deg = target_angle_deg - self.center_bias_deg
            # 用这一段【出发时就定下】的行进方向, 而不是每帧拿 target-current
            # 现算 —— 球冲过目标后现算的符号会翻转, 制动会被当成推动。
            if self.seg_drive_dir != 0.0:
                physical_target_sign = self.seg_drive_dir
            else:
                physical_target_sign = 1.0 if target_cm >= current_cm else -1.0
            target_drive_motor_sign = (
                physical_target_sign * BALL_X_SIGN * CONTROL_SIGN
            )
            driving_toward_target = (
                correction_deg * target_drive_motor_sign > 0.0
            )
            # 原代码在卡滞时只放宽了后面的分段限幅，但 target_angle_deg
            # 此前已经被 _adaptive_angle_limit() 截小，因此末端仍可能推不动。
            # 仅在最后 SRZONE 厘米、球已低速且仍未进稳定窗时，保证一个
            # 可调的最小推进角；起步和高速过靶均不会触发。
            stall_candidate = (
                stalled
                and remaining_cm <= SEG_STALL_RECOVERY_ZONE_CM
            )
            if stall_candidate:
                if self.seg_stall_candidate_ms is None:
                    self.seg_stall_candidate_ms = now_ms
                stall_recovery = (
                    time.ticks_diff(now_ms, self.seg_stall_candidate_ms)
                    >= SEG_STALL_RECOVERY_DWELL_MS
                )
            else:
                self.seg_stall_candidate_ms = None
                stall_recovery = False
            if stall_recovery:
                # 行程锁存方向只适合区分“推进/制动”限幅，不能用来决定
                # 卡滞恢复方向。小球若已经越过目标，恢复方向必须随实时
                # target-current 反转，否则会把球继续往远离目标的一侧推。
                recovery_physical_sign = (
                    1.0 if target_cm >= current_cm else -1.0
                )
                recovery_motor_sign = (
                    recovery_physical_sign * BALL_X_SIGN * CONTROL_SIGN
                )
                recovery_angle_deg = min(
                    SEG_STALL_RECOVERY_ANGLE_DEG,
                    MOTOR_SOFT_LIMIT_DEG,
                )
                correction_deg = recovery_motor_sign * max(
                    abs(correction_deg), recovery_angle_deg,
                )
                driving_toward_target = (
                    correction_deg * target_drive_motor_sign > 0.0
                )
                self.seg_stall_recovery_active = True
            special_cap_deg = (
                drive_cap_deg
                if driving_toward_target
                else SEG_BRAKE_MAX_ANGLE_DEG
            )
            if stall_recovery:
                # 三轮V11均在第二段末端被普通9.64deg推进上限卡死。
                # 只在低速、末段且仍有明显余量时开放短时脱困角；正常行程
                # 的速度、加速度和推进上限保持V11不变。
                special_cap_deg = max(special_cap_deg, recovery_angle_deg)
            correction_deg = clamp(
                correction_deg,
                -special_cap_deg,
                special_cap_deg,
            )
            target_angle_deg = clamp(
                self.center_bias_deg + correction_deg,
                -MOTOR_SOFT_LIMIT_DEG,
                MOTOR_SOFT_LIMIT_DEG,
            )
            angle_limit = min(angle_limit, special_cap_deg)
            if driving_toward_target:
                max_angle_step = min(max_angle_step, SEG_MAX_ANGLE_STEP_DEG)
                target_alpha = min(target_alpha, SEG_TARGET_ALPHA)
            else:
                # 制动: 让角度尽快翻过去, 别再被"柔和"拖着
                max_angle_step = max(max_angle_step, SEG_BRAKE_ANGLE_STEP_DEG)
                target_alpha = max(target_alpha, SEG_BRAKE_TARGET_ALPHA)
        else:
            self.seg_stall_candidate_ms = None

        if self.fine_active:
            target_alpha = max(target_alpha, ANGLE_TARGET_ALPHA_NEAR)

        self.dynamic_angle_step_deg = max_angle_step
        self.filtered_target_angle_deg += target_alpha * (
            target_angle_deg - self.filtered_target_angle_deg
        )

        angle_step = clamp(
            self.filtered_target_angle_deg - self.angle_command_deg,
            -max_angle_step,
            max_angle_step,
        )
        self.angle_command_deg = clamp(
            self.angle_command_deg + angle_step,
            -MOTOR_SOFT_LIMIT_DEG,
            MOTOR_SOFT_LIMIT_DEG,
        )

        if self.dynamic_disturbance_active:
            self.dynamic_motor_speed_rpm = MOTOR_POSITION_SPEED_DISTURBANCE_RPM
        elif self.dynamic_brake_active:
            self.dynamic_motor_speed_rpm = MOTOR_POSITION_SPEED_BRAKE_RPM
        elif self.terminal_hold_active:
            # 完成后仍要及时换向刹住回弹；42RPM的普通近端速度
            # 会使命令角已经反向，实际机构还没有跟上。
            self.dynamic_motor_speed_rpm = TERMINAL_HOLD_MOTOR_SPEED_RPM
        elif self.center_hold or abs(position_error_px) <= MOTOR_NEAR_CENTER_POSITION_PX:
            self.dynamic_motor_speed_rpm = MOTOR_POSITION_SPEED_NEAR_RPM
        else:
            self.dynamic_motor_speed_rpm = int(
                MOTOR_POSITION_SPEED_NORMAL_RPM
                + (MOTOR_POSITION_SPEED_BRAKE_RPM
                   - MOTOR_POSITION_SPEED_NORMAL_RPM)
                * self.dynamic_urgency
            )

        self.settled_state = settled

        if abs(self.angle_command_deg) < MOTOR_DEG_PER_PULSE * 0.5:
            self.angle_command_deg = 0.0

        # 车辆起步前馈只叠加到真正下发给电机的角度。控制器内部的
        # angle_command_deg仍表示球位置闭环自身输出，二者不会互相积分。
        motor_angle_command_deg = clamp(
            self.angle_command_deg + self.external_feedforward_deg,
            -MOTOR_SOFT_LIMIT_DEG,
            MOTOR_SOFT_LIMIT_DEG,
        )

        return (
            position_px,                 # 0
            velocity_px_s,               # 1
            position_error_px,           # 2
            velocity_reference_px_s,     # 3
            velocity_error_px_s,         # 4
            motor_angle_command_deg,      # 5
            prediction_ms,                # 6
            self.kalman.acceleration,     # 7
            self.center_hold,             # 8
            settled,                      # 9
            position_angle_assist_deg,    # 10
            self.center_bias_deg,         # 11
            angle_limit,                  # 12
            static_recovery_deg,          # 13
            final_approach_deg,            # 14
            self.dynamic_disturbance_active, # 15
            self.dynamic_urgency,             # 16
            self.dynamic_position_kp,         # 17
            self.dynamic_velocity_kp,         # 18
            self.dynamic_brake_active,        # 19
            velocity_feedforward_deg,         # 20
            acceleration_damping_deg,         # 21
            self.dynamic_angle_step_deg,       # 22
            self.dynamic_motor_speed_rpm,      # 23
            self.external_feedforward_deg,     # 24
        )


def shift_detection_to_predicted_x(detection, predicted_x):
    """把最近YOLO框平移到预测位置，仅用于低延迟OSD/图传显示。"""
    if detection is None or predicted_x is None:
        return detection
    score, x1, y1, x2, y2 = detection
    half_width = 0.5 * (x2 - x1)
    new_x1 = clamp(predicted_x - half_width, 0.0, AI_WIDTH - 1.0)
    new_x2 = clamp(predicted_x + half_width, 1.0, AI_WIDTH)
    if new_x2 <= new_x1:
        return detection
    return (score, new_x1, y1, new_x2, y2)


def update_motor_from_controller(
    controller, motor_uart, now_ms, motor_pulses, last_motor_command_ms,
):
    """动态限频发送FD命令：冲击时快，正常时稳，中心附近避免追噪声。"""
    state = controller.update_control(now_ms)
    if state is None:
        return None, motor_pulses, last_motor_command_ms

    new_pulses = angle_to_signed_pulses(state[5])
    elapsed_ms = time.ticks_diff(now_ms, last_motor_command_ms)
    near_center = abs(state[2]) <= MOTOR_NEAR_CENTER_POSITION_PX
    disturbance_active = bool(state[15])
    brake_active = bool(state[19])

    if disturbance_active or brake_active:
        min_interval_ms = 22
        hysteresis = 2
    elif near_center:
        min_interval_ms = 36
        hysteresis = MOTOR_COMMAND_NEAR_CENTER_HYSTERESIS
    else:
        min_interval_ms = MOTOR_COMMAND_MIN_INTERVAL_MS
        # 远处普通运动提高脉冲滞回，过滤Kalman小幅波动；紧迫度高时自动减小。
        hysteresis = max(
            2,
            int(MOTOR_COMMAND_PULSE_HYSTERESIS + 3 * (1.0 - state[16])),
        )

    pulse_delta = abs(new_pulses - motor_pulses)
    zero_transition = (new_pulses == 0) != (motor_pulses == 0)
    sign_changed = (
        new_pulses != 0
        and motor_pulses != 0
        and ((new_pulses > 0) != (motor_pulses > 0))
    )
    change_is_meaningful = (
        zero_transition
        or sign_changed
        or pulse_delta >= hysteresis
    )
    refresh_due = (
        new_pulses != 0
        and elapsed_ms >= MOTOR_COMMAND_REFRESH_MS
    )
    interval_ready = elapsed_ms >= min_interval_ms

    should_send = (new_pulses == 0 and motor_pulses != 0) or (
        interval_ready and (change_is_meaningful or refresh_due)
    )
    if should_send:
        motor_pulses = motor_position_emm(
            motor_uart, new_pulses, int(state[23])
        )
        last_motor_command_ms = now_ms

    return state, motor_pulses, last_motor_command_ms


# ============================== UART2 / ZDT_X42S ==============================
def create_motor_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART2_TX_IO, FPIOA.UART2_TXD)
    fpioa.set_function(UART2_RX_IO, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


def motor_enable(uart, enable):
    uart.write(bytes((
        MOTOR_ID, 0xF3, 0xAB,
        0x01 if enable else 0x00,
        0x00, 0x6B,
    )))


def motor_zero_position(uart):
    uart.write(bytes((MOTOR_ID, 0x0A, 0x6D, 0x6B)))


def angle_to_signed_pulses(angle_deg):
    angle_deg = clamp(angle_deg, -MOTOR_SOFT_LIMIT_DEG, MOTOR_SOFT_LIMIT_DEG)
    magnitude = abs(angle_deg)
    pulses = int(magnitude / MOTOR_DEG_PER_PULSE + 0.5)

    # 非零角度至少转换成一个脉冲，避免round在半脉冲附近得到0。
    if magnitude >= MOTOR_DEG_PER_PULSE * 0.35 and pulses == 0:
        pulses = 1

    pulses = int(clamp(pulses, 0, MOTOR_MAX_PULSES))
    return -pulses if angle_deg < 0 else pulses


def motor_position_emm(uart, signed_pulses, speed_rpm=None):
    signed_pulses = int(clamp(
        int(signed_pulses),
        -MOTOR_MAX_PULSES,
        MOTOR_MAX_PULSES,
    ))
    direction = MOTOR_POSITIVE_DIRECTION
    if signed_pulses < 0:
        direction = 1 - MOTOR_POSITIVE_DIRECTION

    pulses = abs(signed_pulses)
    speed = (
        MOTOR_POSITION_SPEED_NORMAL_RPM
        if speed_rpm is None
        else int(speed_rpm)
    )
    speed = int(clamp(speed, 1, 3000))
    uart.write(bytes((
        MOTOR_ID,
        0xFD,
        direction,
        (speed >> 8) & 0xFF,
        speed & 0xFF,
        MOTOR_ACCELERATION,
        (pulses >> 24) & 0xFF,
        (pulses >> 16) & 0xFF,
        (pulses >> 8) & 0xFF,
        pulses & 0xFF,
        0x01,
        0x00,
        0x6B,
    )))
    return signed_pulses


# ============================== 链路：与 3507 的命令 / 回传 ==============================
def create_link_uart():
    """初始化UART4，用于K230与MSPM0G3507之间的任务通信。"""
    fpioa = FPIOA()
    fpioa.set_function(LINK_UART_TX_IO, FPIOA.UART4_TXD)
    fpioa.set_function(LINK_UART_RX_IO, FPIOA.UART4_RXD)
    return UART(
        UART.UART4,
        baudrate=LINK_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )


class LinkReceiver:
    """下行命令帧 [P3]..[P6] / [G3]..[G6] / [C6] / [S] 的状态机。

    【必须非阻塞】它跑在控制主循环里，任何等待都会直接压低控制帧率，
    而球杆是开环发散系统，帧率一掉相位裕度立刻不够。
    """

    def __init__(self):
        self.payload = ""
        self.active = False

    def poll(self, uart):
        commands = []
        if uart is None:
            return commands

        try:
            pending = uart.any()
        except Exception:
            pending = 0
        if not pending:
            return commands

        try:
            data = uart.read(pending)
        except Exception:
            data = None
        if not data:
            return commands

        for raw in data:
            char = chr(raw) if isinstance(raw, int) else raw
            if char == "[":
                self.active = True
                self.payload = ""
            elif char == "]":
                if self.active and self.payload:
                    commands.append(self.payload)
                self.active = False
                self.payload = ""
            elif self.active:
                if len(self.payload) >= LINK_RX_MAX_PAYLOAD:
                    # 越界说明这一帧已经废了（丢了 ']' 或纯粹是干扰），
                    # 整帧丢弃，不能让残渣拼进下一帧变成一条假命令。
                    self.active = False
                    self.payload = ""
                else:
                    self.payload += char
        return commands


class CarMotionFeedforward:
    """接收3507起步前馈，并生成有界且能在超时后释放的附加摆杆角。"""

    def __init__(self):
        self.sequence = -1
        self.configured = False
        self.target_deg = 0.0
        self.current_deg = 0.0
        self.max_deg = 0.0
        self.last_sample_ms = None
        self.last_service_ms = None
        self.timeout_reported = False
        self.lead_ms = 0
        self.ramp_ms = 0
        self.gain_x100 = 0
        self.damp_x100 = 0
        self.sender_sign = 1
        self.reference_accel = 0
        self.measured_accel = 0
        self.last_drive_sign = 0.0
        self.peak_target_abs_deg = 0.0
        self.recovery_started_ms = None
        self.recovery_active = False

    def _clear_target(self):
        self.configured = False
        self.target_deg = 0.0
        self.last_sample_ms = None

    def _cancel_recovery(self):
        self.recovery_started_ms = None
        self.recovery_active = False

    def on_command(self, command, now_ms):
        """识别FC/FA/FE时返回True；旧任务命令继续交给MissionController。"""
        parts = [item.strip() for item in str(command).strip().split(",")]
        if not parts:
            return False

        frame_type = parts[0].upper()
        if frame_type not in ("FC", "FA", "FE"):
            return False

        try:
            if frame_type == "FC":
                if len(parts) != 8:
                    raise ValueError("FC field count")
                sequence = int(parts[1])
                lead_ms = int(parts[2])
                ramp_ms = int(parts[3])
                gain_x100 = int(parts[4])
                damp_x100 = int(parts[5])
                max_x10 = int(parts[6])
                sender_sign = int(parts[7])

                if not (0 <= sequence <= 255):
                    raise ValueError("FC sequence")
                if not (0 <= lead_ms <= 2000 and 50 <= ramp_ms <= 5000):
                    raise ValueError("FC timing")
                if not (0 <= max_x10 <= int(MOTOR_SOFT_LIMIT_DEG * 10.0)):
                    raise ValueError("FC limit")
                if sender_sign not in (-1, 1):
                    raise ValueError("FC sign")

                new_sequence = sequence != self.sequence
                self.sequence = sequence
                self.lead_ms = lead_ms
                self.ramp_ms = ramp_ms
                self.gain_x100 = gain_x100
                self.damp_x100 = damp_x100
                self.max_deg = max_x10 / 10.0
                self.sender_sign = sender_sign
                self.configured = bool(CAR_START_FF_ENABLE and max_x10 > 0)
                self.target_deg = 0.0
                self.last_sample_ms = now_ms
                self.timeout_reported = False
                # 同一FC若重复到达，不把正在生效的角度突然跳回0。
                if new_sequence:
                    self.current_deg = 0.0
                    self.last_service_ms = now_ms
                    self.last_drive_sign = 0.0
                    self.peak_target_abs_deg = 0.0
                    self._cancel_recovery()
                print(
                    "[CAR-FF] FC seq=%d lead=%dms ramp=%dms max=%.1fdeg sign=%+d"
                    % (
                        self.sequence, self.lead_ms, self.ramp_ms,
                        self.max_deg, self.sender_sign,
                    )
                )
                return True

            if frame_type == "FA":
                if len(parts) != 5:
                    raise ValueError("FA field count")
                sequence = int(parts[1])
                ff_x10 = int(parts[2])
                reference_accel = int(parts[3])
                measured_accel = int(parts[4])
                if self.configured and sequence == self.sequence:
                    self.target_deg = clamp(
                        ff_x10 / 10.0,
                        -self.max_deg,
                        self.max_deg,
                    )
                    self.reference_accel = reference_accel
                    self.measured_accel = measured_accel
                    self.last_sample_ms = now_ms
                    self.timeout_reported = False
                    if abs(self.target_deg) > CAR_START_FF_ZERO_EPS_DEG:
                        self.last_drive_sign = (
                            1.0 if self.target_deg > 0.0 else -1.0
                        )
                        self.peak_target_abs_deg = max(
                            self.peak_target_abs_deg,
                            abs(self.target_deg),
                        )
                return True

            if len(parts) != 2:
                raise ValueError("FE field count")
            sequence = int(parts[1])
            if sequence == self.sequence:
                self._clear_target()
                self.recovery_started_ms = now_ms
                self.recovery_active = (
                    self.last_drive_sign != 0.0
                    and self.peak_target_abs_deg
                    > CAR_START_FF_ZERO_EPS_DEG
                )
                print("[CAR-FF] FE seq=%d" % sequence)
            return True
        except (ValueError, TypeError) as error:
            print("[CAR-FF] invalid frame:", str(command), repr(error))
            return True

    def service(
        self,
        now_ms,
        active,
        position_error_px=0.0,
        velocity_px_s=0.0,
        state_valid=False,
    ):
        """按时间更新附加角，并在FE后按球状态平滑交还位置环。"""
        if self.last_service_ms is None:
            dt_s = 0.02
        else:
            dt_s = clamp(
                time.ticks_diff(now_ms, self.last_service_ms) / 1000.0,
                0.0,
                0.100,
            )
        self.last_service_ms = now_ms

        if not active:
            self._clear_target()
            self._cancel_recovery()
        elif (
            self.configured
            and self.last_sample_ms is not None
            and time.ticks_diff(now_ms, self.last_sample_ms)
            > CAR_START_FF_SAMPLE_TIMEOUT_MS
        ):
            self.target_deg = 0.0
            self.configured = False
            if not self.timeout_reported:
                print("[CAR-FF] sample timeout, releasing")
                self.timeout_reported = True

        if self.recovery_active:
            recovery_elapsed_ms = time.ticks_diff(
                now_ms,
                self.recovery_started_ms,
            )
            error_px = float(position_error_px)
            error_sign = 1.0 if error_px >= 0.0 else -1.0
            abs_error_px = abs(error_px)
            closing_speed_px_s = error_sign * float(velocity_px_s)
            same_disturbance_side = (
                error_sign == self.last_drive_sign
            )

            if not state_valid:
                # 一帧识别丢失不能永久取消释放段；先向0释放，下一帧恢复
                # 有效位置后仍可继续判断，直至超时。
                self.target_deg = 0.0
            elif (
                recovery_elapsed_ms > CAR_START_FF_RECOVERY_MS
                or not same_disturbance_side
                or abs_error_px <= CAR_START_FF_RECOVERY_RELEASE_PX
            ):
                self.target_deg = 0.0
                self._cancel_recovery()
            else:
                recovery_abs_deg = (
                    CAR_START_FF_RECOVERY_KP_DEG_PER_PX * abs_error_px
                    - CAR_START_FF_RECOVERY_KD_DEG_PER_PX_S
                    * max(0.0, closing_speed_px_s)
                )
                recovery_limit_deg = min(
                    CAR_START_FF_RECOVERY_MAX_DEG,
                    self.peak_target_abs_deg,
                )
                recovery_abs_deg = clamp(
                    recovery_abs_deg,
                    0.0,
                    recovery_limit_deg,
                )
                self.target_deg = (
                    self.last_drive_sign * recovery_abs_deg
                )

        moving_away_from_zero = abs(self.target_deg) > abs(self.current_deg)
        rate_deg_s = (
            CAR_START_FF_ATTACK_DEG_PER_S
            if moving_away_from_zero else CAR_START_FF_RELEASE_DEG_PER_S
        )
        max_step = rate_deg_s * dt_s
        self.current_deg += clamp(
            self.target_deg - self.current_deg,
            -max_step,
            max_step,
        )
        if (
            self.target_deg == 0.0
            and abs(self.current_deg) <= CAR_START_FF_ZERO_EPS_DEG
        ):
            self.current_deg = 0.0
            if not active:
                self.sequence = -1

        return self.current_deg


def link_send_frame(uart, value, negative):
    """发一帧 [±XXXX*]。value 必须已经是 0..9999 的整数。"""
    if uart is None:
        return
    try:
        uart.write(
            ("[%s%04d*]" % ("-" if negative else "+", int(value))).encode()
        )
    except Exception:
        pass


def link_send_position(uart, position_cm, valid):
    """上行球位置（0.1mm，原点为中心 O）。

    【丢球也要发】发哨兵而不是停发 —— 停发会被 3507 判成整条链路挂了，
    那是完全不同的故障，处置方式也不一样。
    """
    if not valid:
        link_send_frame(uart, LINK_LOST_MARK, False)
        return

    value = position_cm * 100.0
    value = int(value + (0.5 if value >= 0.0 else -0.5))
    negative = value < 0
    # 上限压在 9899：9998/9999 是哨兵，标定跑飞时若让真实位置撞上它们，
    # 3507 那边会把一次量程溢出当成"序列完成"来停表。
    link_send_frame(uart, clamp(abs(value), 0, 9899), negative)


def link_send_done(uart):
    """项3 序列完成哨兵。连发几次，单帧被干扰吃掉就前功尽弃。"""
    for _ in range(LINK_DONE_REPEAT):
        link_send_frame(uart, LINK_DONE_MARK, False)


def link_send_text(uart, text):
    """发送简单ASCII状态帧，例如[R6]、[P6]、[RETURN]。"""
    if uart is None:
        return False
    try:
        uart.write(("[%s]\n" % str(text)).encode())
        print("[UART4 TX]", text)
        return True
    except Exception as error:
        print("UART4 send warning:", repr(error))
        return False


def link_send_task6_ready(uart, target_cm):
    """任务6锁定后上报目标，单位0.1mm，例如 [R6,+0500]。"""
    value = float(target_cm) * 100.0
    value = int(value + (0.5 if value >= 0.0 else -0.5))
    value = int(clamp(value, -9899, 9899))
    sign = "-" if value < 0 else "+"
    return link_send_text(uart, "R6,%s%04d" % (sign, abs(value)))


def create_task6_button():
    """初始化庐山派K230板载用户按键GPIO53。"""
    fpioa = FPIOA()
    gpio_func = getattr(FPIOA, "GPIO%d" % TASK6_BUTTON_GPIO)
    fpioa.set_function(TASK6_BUTTON_GPIO, gpio_func)
    button = Pin(TASK6_BUTTON_GPIO, Pin.IN, Pin.PULL_DOWN)
    print(
        "task6 button ready: GPIO%d pull-down, press=HIGH"
        % TASK6_BUTTON_GPIO
    )
    return button


class DebouncedButton:
    """上升沿触发的非阻塞按键消抖器。"""
    def __init__(self, debounce_ms=30):
        self.debounce_ms = int(debounce_ms)
        self.last_state = 0
        self.last_press_ms = -100000

    def poll_pressed(self, pin, now_ms):
        if pin is None:
            return False
        try:
            state = 1 if pin.value() == TASK6_BUTTON_PRESS_LEVEL else 0
        except Exception:
            return False

        pressed = False
        if state == 1 and self.last_state == 0:
            if time.ticks_diff(now_ms, self.last_press_ms) >= self.debounce_ms:
                pressed = True
                self.last_press_ms = now_ms
        self.last_state = state
        return pressed


def task_return_wait_ms(current_pulses, target_pulses):
    """根据回位脉冲差和速度估算等待时间。"""
    delta = abs(int(current_pulses) - int(target_pulses))
    pulses_per_rev = 360.0 / max(MOTOR_DEG_PER_PULSE, 1e-6)
    travel_ms = (
        delta / pulses_per_rev * 60000.0 / max(TASK_RETURN_SPEED_RPM, 1)
    )
    return int(clamp(
        travel_ms + TASK_RETURN_EXTRA_WAIT_MS,
        TASK_RETURN_MIN_WAIT_MS,
        TASK_RETURN_MAX_WAIT_MS,
    ))


# ============================== 任务状态机 ==============================
MISSION_HOLD_CENTER = 0     # 上电默认 / T4 / T5：把球稳在中心 O
MISSION_HOLD_LOCK = 1       # T6：稳在收到命令那一刻球所在的位置
MISSION_SEQ3 = 2            # T3：摆球序列
MISSION_OFF = 3             # T2 / [S]：摆杆不工作，回 0 位

# 3507是唯一发车主控：P/C只做赛前准备，G才允许正式运行。
LINK_STATE_IDLE = 0
LINK_STATE_PREPARING = 1
LINK_STATE_CENTER6 = 2
LINK_STATE_CENTER6_READY = 3
LINK_STATE_SETTING6 = 4
LINK_STATE_LOCKING6 = 5
LINK_STATE_READY = 6
LINK_STATE_RUNNING = 7
LINK_STATE_RETURNING = 8

SEQ3_GO_FIRST = 0
SEQ3_GO_SECOND = 1
SEQ3_ENDED = 2


class MissionController:
    """把 3507 的一条命令翻译成"目标点放哪、摆杆动不动"。

    它只做这两件事，不碰任何控制律 —— 串级控制器是已经调稳的部分，
    项3 需要的仅仅是"把目标点搬到 ±5cm 再搬回来"。
    """

    def __init__(self):
        # Serial-linked power-on state is passive.  Ball control starts only
        # after a command from MSPM0 (P3/P4/P5/C6/P6 or G3..G6).
        self.mode = MISSION_OFF
        self.target_cm = 0.0
        self.seq_phase = SEQ3_ENDED
        self.seq_start_ms = None
        self.seq_stable_ms = None
        self.seq_inside_ms = None
        self.seq_pos_min = 0.0
        self.seq_pos_max = 0.0
        self.seq_reach_cnt = 0
        self.seq_creep = []
        self.seq_timeout = False
        self.seq_elapsed_ms = 0
        self.done_pending = False
        self.motor_active = False
        self.last_command = "-"

        # 独立运行用
        self.boot_ms = None
        self.auto_armed_ms = None
        self.arm_pos_min = 0.0
        self.arm_pos_max = 0.0
        self.auto_wait_ms = None
        self.auto_runs = 0
        self.link_seen = False
        self.local_configured = False
        self.local_mode = K230_TEST_MODE
        self.tune_hold_only = False
        self.mode6_locked = False

        # 任务6按键标定状态。
        self.task6_wait_button = False
        self.task6_target_cm = None
        self.task6_message = ""

        # 当前任务起始电机位置与任务切换回位状态。
        self.active_task_id = 0
        self.task_start_motor_pulses = 0
        self.returning_to_start = False
        self.return_target_pulses = 0
        self.return_started_ms = None
        self.return_wait_ms = 0
        self.pending_command = None
        self.transition_event = None

        # 与3507的两阶段协议：先P/C准备并READY，再由G正式启动。
        self.link_state = LINK_STATE_IDLE
        self.prepared_task_id = 0
        self.prepare_stable_ms = None
        self.prepare_pos_min_cm = 0.0
        self.prepare_pos_max_cm = 0.0

    # ---------------- 目标点 ----------------
    def _set_target(self, controller, target_cm):
        self.target_cm = target_cm
        controller.set_target_px(cm_to_px(target_cm))

    # ---------------- 模式切换 ----------------
    def _clear_sequence(self):
        self.seq_phase = SEQ3_ENDED
        self.seq_start_ms = None
        self.seq_stable_ms = None
        self.seq_inside_ms = None
        self.seq_pos_min = 0.0
        self.seq_pos_max = 0.0
        self.seq_reach_cnt = 0
        self.seq_creep = []
        self.seq_timeout = False
        self.done_pending = False

    def _clear_segment_control(self, controller):
        """离开任务3分段行程时，清除全部锁存限幅与恢复状态。"""
        controller.neg_to_pos_soft_active = False
        controller.seg_drive_dir = 0.0
        controller.seg_speed_cap_px_s = 0.0
        controller.seg_drive_accel_limit_cm_s2 = 0.0
        controller.seg_near_zone_cm = -1.0
        controller.seg_stall_recovery_active = False
        controller.seg_remaining_cm = 0.0
        controller.seg_velocity_profile_active = False
        controller.seg_profile_speed_cm_s = 0.0
        controller.seg_stall_candidate_ms = None
        controller.terminal_hold_active = False
        controller.terminal_stall_candidate_ms = None
        controller.terminal_stall_active = False
        controller.terminal_stall_sign = 0.0
        controller.terminal_stall_phase = 0
        controller.terminal_stall_phase_started_ms = None

    def _enter_seq3_terminal_hold(self, controller):
        """锁住任务3理论-5cm终点，且完成后继续闭环保持。"""
        self._clear_segment_control(controller)
        self.target_cm = -SEQ3_FIRST_DIR * SEQ3_STROKE_CM
        controller.target_px = cm_to_px(self.target_cm)
        controller.terminal_hold_active = True
        controller.terminal_stall_candidate_ms = None
        controller.terminal_stall_active = False
        controller.terminal_stall_sign = 0.0
        controller.terminal_stall_phase = 0
        controller.terminal_stall_phase_started_ms = None
        controller.center_hold = True
        controller.center_candidate_ms = None
        controller.fine_active = False
        controller.fine_candidate_ms = None
        controller.static_recovery_candidate_ms = None
        controller.static_recovery_last_kick_ms = None
        controller.static_recovery_angle_deg = 0.0
        controller.static_recovery_sign = 0.0
        controller.velocity_pid.reset()

    def _enter_off(self, controller):
        """项2 / 中止：摆杆放平。

        【不是"维持最后的倾角"】没有球还压着角度，下一项把球放上去的
        瞬间它就先被推出去一截，等于每次开始都自带一个初始误差。
        """
        self._clear_sequence()
        self._clear_segment_control(controller)
        self.mode = MISSION_OFF
        self.motor_active = False
        self._set_target(controller, 0.0)

    def _enter_hold_center(self, controller):
        self._clear_sequence()
        self._clear_segment_control(controller)
        self.mode = MISSION_HOLD_CENTER
        self.motor_active = True
        self._set_target(controller, 0.0)

    def _enter_hold_lock(self, controller, target_cm):
        """任务6按键确认后，锁定指定的实际钢球位置。"""
        self._clear_sequence()
        self._clear_segment_control(controller)
        self.mode = MISSION_HOLD_LOCK
        self.motor_active = True
        self.task6_wait_button = False
        self.task6_target_cm = float(target_cm)
        self.mode6_locked = True
        self._set_target(controller, self.task6_target_cm)

    def _enter_task6_wait(self, controller):
        """任务6准备页：不闭环，等待板载按键采集当前钢球位置。"""
        self._clear_sequence()
        self._clear_segment_control(controller)
        self.mode = MISSION_OFF
        self.motor_active = False
        self.mode6_locked = False
        self.task6_wait_button = True
        self.task6_target_cm = None
        self.task6_message = "PLACE BALL, PRESS KEY"
        self.last_command = "T6-WAIT-KEY"

    def capture_task6_button(self, controller, now_ms, ball_valid):
        """按下板载按键时，把摄像头当前识别位置锁定为任务6目标。"""
        if self.returning_to_start or not self.task6_wait_button:
            return False

        if not ball_valid or not controller.kalman.ready:
            self.task6_message = "NO BALL - PRESS AGAIN"
            self.transition_event = ("T6_ERROR", "NO_BALL")
            print("[T6] key ignored: ball not valid")
            return False

        capture_px = controller.kalman.position
        if (
            controller.last_measurement_px is not None
            and controller.last_measurement_ms is not None
            and time.ticks_diff(now_ms, controller.last_measurement_ms) <= 100
        ):
            capture_px = controller.last_measurement_px

        lock_cm = px_to_cm(capture_px)
        lock_cm = clamp(lock_cm, TASK6_MIN_CM, TASK6_MAX_CM)
        self._enter_hold_lock(controller, lock_cm)
        self.task6_message = "LOCKED %+.2fcm" % lock_cm
        self.last_command = "T6-LOCK"
        if LINK_ENABLE:
            self.active_task_id = 0
            self.prepared_task_id = 6
            self.link_state = LINK_STATE_LOCKING6
            self._reset_prepare_window()
            self.task6_message = "LOCKING %+.2fcm" % lock_cm
            self.transition_event = None
        else:
            self.transition_event = ("START", 6)
        print("[T6] button lock target=%+.3fcm" % lock_cm)
        return True

    def _enter_seq3(self, controller, now_ms):
        self._clear_sequence()
        self.mode = MISSION_SEQ3
        self.motor_active = True
        self.seq_phase = SEQ3_GO_FIRST
        self.seq_start_ms = now_ms
        self.seq_elapsed_ms = 0
        # 第二段最终运动方向为 -SEQ3_FIRST_DIR。终点保持必须继续压在刚才
        # 到达终点的同一回差侧，不能复用O点静止时可能落在另一侧的角度。
        controller.terminal_hold_bias_deg = clamp(
            -SEQ3_FIRST_DIR * CONTROL_SIGN * TERMINAL_HOLD_PRELOAD_DEG,
            -10.0, 10.0,
        )
        # 【没有"先回中心"的准备段】3507 按下 KEY1 的同一刻就开始计时，
        # 限时只有 5s，任何准备动作都是直接从成绩里扣。球在按键之前就该
        # 已经稳在 O —— 上电默认模式做的正是这件事。
        self._set_target(controller, SEQ3_FIRST_DIR * SEQ3_STROKE_CM)
        # 第一段同样限幅 —— 原来这里关掉, 第一段就用 20 度总限位(41cm/s2)
        # 一路推过去, "到 +5 会冲过"就是这么来的。
        controller.neg_to_pos_soft_active = SEG_SOFT_ENABLE and SEG_SOFT_FIRST_LEG
        controller.seg_drive_dir = SEQ3_FIRST_DIR
        controller.seg_speed_cap_px_s = SEG_LEG1_SPEED_CAP_PX_S
        controller.seg_drive_accel_limit_cm_s2 = SEG_LEG1_DRIVE_ACCEL_CM_S2
        controller.seg_near_zone_cm = SEG_LEG1_NEAR_ZONE_CM
        controller.seg_velocity_profile_active = False
        controller.seg_profile_speed_cm_s = 0.0
        controller.seg_stall_candidate_ms = None

    # ---------------- 3507两阶段串口协议 ----------------
    def _reset_prepare_window(self):
        self.prepare_stable_ms = None
        self.prepare_pos_min_cm = 0.0
        self.prepare_pos_max_cm = 0.0

    def _prepare_center_task(self, task_id, controller):
        if (
            self.link_state == LINK_STATE_READY
            and self.prepared_task_id == task_id
        ):
            self.transition_event = ("READY", task_id)
            return
        if (
            self.link_state == LINK_STATE_PREPARING
            and self.prepared_task_id == task_id
        ):
            # Pn is retried by MSPM0 until READY.  Acknowledge reception
            # immediately so the car can distinguish UART failure from
            # "ball is still moving to center".
            self.transition_event = ("PREP", task_id)
            return

        self._enter_hold_center(controller)
        self.active_task_id = 0
        self.prepared_task_id = int(task_id)
        self.task6_wait_button = False
        self.mode6_locked = False
        self.task6_target_cm = None
        self.task6_message = ""
        self.link_state = LINK_STATE_PREPARING
        self.last_command = "P%d" % task_id
        self._reset_prepare_window()
        self.transition_event = ("PREP", task_id)
        print("[LINK] prepare T%d: hold center" % task_id)

    def _prepare_task6_center(self, controller):
        if self.link_state == LINK_STATE_CENTER6_READY:
            self.transition_event = ("CENTER6_READY", 6)
            return
        if self.link_state == LINK_STATE_CENTER6:
            self.transition_event = ("CENTER6", 6)
            return

        self._enter_hold_center(controller)
        self.active_task_id = 0
        self.prepared_task_id = 0
        self.task6_wait_button = False
        self.mode6_locked = False
        self.task6_target_cm = None
        self.task6_message = "CENTERING"
        self.link_state = LINK_STATE_CENTER6
        self.last_command = "C6"
        self._reset_prepare_window()
        self.transition_event = ("CENTER6", 6)
        print("[LINK] C6: return ball to center")

    def _prepare_task6_target(self, controller):
        if (
            self.link_state == LINK_STATE_READY
            and self.prepared_task_id == 6
            and self.mode6_locked
            and self.task6_target_cm is not None
        ):
            self.transition_event = ("TASK6_READY", self.task6_target_cm)
            return
        if self.link_state == LINK_STATE_SETTING6:
            self.transition_event = ("SET6", 6)
            return
        if self.link_state == LINK_STATE_LOCKING6:
            # P6会持续重发到R6为止；锁定过程中不能重新进入按键等待。
            self.transition_event = ("SET6", 6)
            return

        self._enter_task6_wait(controller)
        self.active_task_id = 0
        self.prepared_task_id = 6
        self.link_state = LINK_STATE_SETTING6
        self.last_command = "P6"
        self._reset_prepare_window()
        self.transition_event = ("SET6", 6)
        print("[LINK] P6: place ball and press K230 key")

    def _start_prepared_task(
        self, task_id, controller, now_ms, current_motor_pulses
    ):
        if (
            self.link_state == LINK_STATE_RUNNING
            and self.active_task_id == task_id
        ):
            # 3507会重发G命令；只重发RUN确认，绝不能重启任务3序列。
            self.transition_event = ("RUN", task_id)
            return

        if (
            self.link_state != LINK_STATE_READY
            or self.prepared_task_id != task_id
        ):
            self.transition_event = ("ERROR", "G%d" % task_id)
            print("[LINK] reject G%d: task not ready" % task_id)
            return

        if task_id == 6 and (
            not self.mode6_locked or self.task6_target_cm is None
        ):
            self.transition_event = ("ERROR", "G6")
            print("[LINK] reject G6: target not locked")
            return

        self.active_task_id = int(task_id)
        self.prepared_task_id = 0
        self.task_start_motor_pulses = int(current_motor_pulses)
        self.pending_command = None
        self.returning_to_start = False
        self.link_state = LINK_STATE_RUNNING
        self.last_command = "G%d" % task_id

        if task_id == 3:
            self._enter_seq3(controller, now_ms)
        elif task_id in (4, 5):
            self._enter_hold_center(controller)
        # T6保持按键锁定的目标，不重新采样、不改变目标。

        self.transition_event = ("RUN", task_id)
        print(
            "[LINK] run T%d, motor_start=%+d pulses"
            % (task_id, self.task_start_motor_pulses)
        )

    def service_link_state(self, controller, now_ms, ball_valid):
        """非阻塞推进准备状态；READY只表示K3前的全部动作已完成。"""
        if not LINK_ENABLE or self.link_state not in (
            LINK_STATE_PREPARING,
            LINK_STATE_CENTER6,
            LINK_STATE_LOCKING6,
        ):
            return

        if not ball_valid or not controller.kalman.ready:
            self._reset_prepare_window()
            return

        position_cm = px_to_cm(controller.kalman.position)
        target_cm = (
            self.task6_target_cm
            if self.link_state == LINK_STATE_LOCKING6
            and self.task6_target_cm is not None
            else 0.0
        )
        if abs(position_cm - target_cm) > STANDALONE_ARM_TOL_CM:
            self._reset_prepare_window()
            return

        if self.prepare_stable_ms is None:
            self.prepare_stable_ms = now_ms
            self.prepare_pos_min_cm = position_cm
            self.prepare_pos_max_cm = position_cm
            return

        self.prepare_pos_min_cm = min(self.prepare_pos_min_cm, position_cm)
        self.prepare_pos_max_cm = max(self.prepare_pos_max_cm, position_cm)
        spread_limit_cm = (
            MODE6_ARM_SPREAD_CM
            if self.link_state == LINK_STATE_LOCKING6
            else STANDALONE_ARM_SPREAD_CM
        )
        if (
            self.prepare_pos_max_cm - self.prepare_pos_min_cm
            > spread_limit_cm
        ):
            self.prepare_stable_ms = now_ms
            self.prepare_pos_min_cm = position_cm
            self.prepare_pos_max_cm = position_cm
            return

        stable_required_ms = (
            MODE6_ARM_MS
            if self.link_state == LINK_STATE_LOCKING6
            else STANDALONE_ARM_MS
        )
        if time.ticks_diff(now_ms, self.prepare_stable_ms) < stable_required_ms:
            return

        self._reset_prepare_window()
        if self.link_state == LINK_STATE_LOCKING6:
            self.link_state = LINK_STATE_READY
            self.transition_event = ("TASK6_READY", self.task6_target_cm)
            self.task6_message = "LOCKED %+.2fcm" % self.task6_target_cm
            print("[LINK] R6: target locked and stable")
        elif self.link_state == LINK_STATE_CENTER6:
            self.link_state = LINK_STATE_CENTER6_READY
            self.transition_event = ("CENTER6_READY", 6)
            self.task6_message = "CENTER READY"
            print("[LINK] C6OK: ball centered")
        else:
            task_id = self.prepared_task_id
            self.link_state = LINK_STATE_READY
            self.transition_event = ("READY", task_id)
            print("[LINK] R%d: preparation complete" % task_id)

    def _start_command_now(
        self, command, controller, now_ms, current_motor_pulses
    ):
        command = command.strip().upper()
        if command not in ("T3", "T4", "T5", "T6"):
            return

        task_id = int(command[1])
        self.active_task_id = task_id
        self.task_start_motor_pulses = int(current_motor_pulses)
        self.pending_command = None
        self.returning_to_start = False
        self.task6_wait_button = False
        self.task6_message = ""
        self.last_command = command

        print(
            "[TASK] start T%d, motor_start=%+d pulses"
            % (task_id, self.task_start_motor_pulses)
        )

        if command == "T3":
            self._enter_seq3(controller, now_ms)
            self.transition_event = ("START", 3)
        elif command in ("T4", "T5"):
            self._enter_hold_center(controller)
            self.transition_event = ("START", task_id)
        else:
            self._enter_task6_wait(controller)
            self.transition_event = ("T6_WAIT", 6)

    def _begin_return_to_start(
        self, controller, now_ms, current_motor_pulses, pending_command=None
    ):
        self._clear_sequence()
        self.mode = MISSION_OFF
        self.motor_active = False
        self.task6_wait_button = False
        self.mode6_locked = False
        self.task6_message = ""

        self.returning_to_start = True
        self.return_target_pulses = int(self.task_start_motor_pulses)
        self.return_started_ms = now_ms
        self.return_wait_ms = task_return_wait_ms(
            current_motor_pulses, self.return_target_pulses
        )
        self.pending_command = pending_command
        self.transition_event = ("RETURN", self.return_target_pulses)

        print(
            "[TASK] end T%d, return %+d -> %+d pulses, wait=%dms next=%s"
            % (
                self.active_task_id,
                int(current_motor_pulses),
                self.return_target_pulses,
                self.return_wait_ms,
                str(pending_command),
            )
        )

    def on_command(self, command, controller, now_ms, current_motor_pulses):
        command = command.strip().upper()
        if command == "STOP":
            command = "S"
        if command == "T2":
            command = "S"
        if command in ("T3", "T4", "T5", "T6"):
            # 兼容旧上位机，但只进入准备，绝不绕过3507的G发车权。
            command = "P" + command[1]
        if command not in (
            "S", "C6", "P3", "P4", "P5", "P6",
            "G3", "G4", "G5", "G6",
        ):
            return

        self.last_command = command
        self.link_seen = True

        if self.returning_to_start:
            if command == "S":
                self.pending_command = None
                print("[TASK] return pending cleared by STOP")
            else:
                self.transition_event = ("ERROR", "BUSY")
                print("[LINK] command ignored while returning:", command)
            return

        if command == "S":
            self.prepared_task_id = 0
            self._reset_prepare_window()
            if self.active_task_id != 0:
                self.link_state = LINK_STATE_RETURNING
                self._begin_return_to_start(
                    controller, now_ms, current_motor_pulses, None
                )
            else:
                self._enter_off(controller)
                self.active_task_id = 0
                self.link_state = LINK_STATE_IDLE
                self.transition_event = ("IDLE", 0)
            return

        if command in ("P3", "P4", "P5"):
            self._prepare_center_task(int(command[1]), controller)
            return

        if command == "C6":
            self._prepare_task6_center(controller)
            return

        if command == "P6":
            self._prepare_task6_target(controller)
            return

        self._start_prepared_task(
            int(command[1]), controller, now_ms, current_motor_pulses
        )

    def service_return(self, controller, now_ms):
        if not self.returning_to_start or self.return_started_ms is None:
            return
        if time.ticks_diff(now_ms, self.return_started_ms) < self.return_wait_ms:
            return

        returned_pulses = int(self.return_target_pulses)
        pending = self.pending_command

        self.returning_to_start = False
        self.return_started_ms = None
        self.return_wait_ms = 0
        self.pending_command = None
        self.active_task_id = 0
        self.mode = MISSION_OFF
        self.motor_active = False
        self.prepared_task_id = 0
        self.link_state = LINK_STATE_IDLE
        controller.sync_motor_output_pulses(returned_pulses)
        print("[TASK] return complete at %+d pulses" % returned_pulses)

        if pending is None:
            self.transition_event = ("RETURN_DONE", returned_pulses)
        else:
            self._start_command_now(
                pending, controller, now_ms, returned_pulses
            )

    def take_transition_event(self):
        event = self.transition_event
        self.transition_event = None
        return event

    def shutdown_return_target(self):
        if self.returning_to_start:
            return int(self.return_target_pulses)
        if self.active_task_id != 0:
            return int(self.task_start_motor_pulses)
        return 0

    # ---------------- K230本地模式初始化 ----------------
    def configure_local_mode(self, controller):
        """按K230_TEST_MODE设置启动状态，只在main()启动时调用一次。"""
        self.local_configured = True
        self.local_mode = K230_TEST_MODE
        self.auto_runs = 0
        self.auto_wait_ms = None
        self.tune_hold_only = False
        self.mode6_locked = False
        self._arm_reset()

        if LINK_ENABLE:
            # 联机时上电保持完全空闲；不能因本地测试模式或菜单浏览
            # 自动控制钢球。只有3507发来的正式准备命令才能使能控制。
            self._enter_off(controller)
            self.active_task_id = 0
            self.prepared_task_id = 0
            self.link_state = LINK_STATE_IDLE
            self.last_command = "LINK-IDLE"
            return

        if self.local_mode in (3, 4, 5):
            # 模式3先稳在O点等待起跑；模式4/5始终保持O点。
            self._enter_hold_center(controller)
            self.last_command = "MODE%d" % self.local_mode
        else:
            # 模式6不再自动锁定；等待板载按键采集当前钢球位置。
            self._enter_task6_wait(controller)
            self.active_task_id = 6
            self.last_command = "MODE6-WAIT-KEY"

    # ---------------- 独立运行 ----------------
    def _arm_reset(self):
        self.auto_armed_ms = None
        self.arm_pos_min = 0.0
        self.arm_pos_max = 0.0

    def arm_progress_ms(self, now_ms):
        """已连续满足多久(ms)。-1 = 当前不满足, 用来在终端上看卡在哪。"""
        if self.auto_armed_ms is None:
            return -1
        return time.ticks_diff(now_ms, self.auto_armed_ms)

    def autorun(self, controller, now_ms, ball_valid):
        """不接3507时，根据K230_TEST_MODE自动运行对应测试项。"""
        if LINK_ENABLE or self.link_seen:
            return

        # UART4调参时 [CENTER]/[OFF] 会暂停自动重跑；[RUN]再解锁。
        if TASK3_UART_TUNE_ENABLE and self.tune_hold_only:
            return

        if self.boot_ms is None:
            self.boot_ms = now_ms
        if time.ticks_diff(now_ms, self.boot_ms) < STANDALONE_BOOT_MS:
            return

        # 模式4/5：K230侧动作相同，始终保持中心O点。
        if K230_TEST_MODE in (4, 5):
            if self.mode != MISSION_HOLD_CENTER or not self.motor_active:
                self._enter_hold_center(controller)
            self.last_command = "MODE%d-HOLD" % K230_TEST_MODE
            return

        # 模式6：只等待板载按键，不再按时间自动锁定。
        if K230_TEST_MODE == 6:
            if not self.mode6_locked and not self.task6_wait_button:
                self._enter_task6_wait(controller)
            return

        # 以下仅模式3。序列正在运行时由update()推进。
        if self.mode == MISSION_SEQ3 and self.seq_phase != SEQ3_ENDED:
            self._arm_reset()
            return

        # 序列已经结束：不循环时保持终点；循环时延时回O。
        if self.mode == MISSION_SEQ3:
            if not STANDALONE_REPEAT:
                return
            if self.auto_wait_ms is None:
                self.auto_wait_ms = now_ms
            elif time.ticks_diff(now_ms, self.auto_wait_ms) >= STANDALONE_RERUN_MS:
                self.auto_wait_ms = None
                self._enter_hold_center(controller)
            return

        # 等球回到O并稳定后启动模式3。
        self.auto_wait_ms = None
        if not ball_valid:
            self._arm_reset()
            return

        position_cm = px_to_cm(controller.kalman.position)
        if abs(position_cm) > STANDALONE_ARM_TOL_CM:
            self._arm_reset()
            return

        if self.auto_armed_ms is None:
            self.auto_armed_ms = now_ms
            self.arm_pos_min = position_cm
            self.arm_pos_max = position_cm
            return

        if position_cm < self.arm_pos_min:
            self.arm_pos_min = position_cm
        if position_cm > self.arm_pos_max:
            self.arm_pos_max = position_cm

        if self.arm_pos_max - self.arm_pos_min > STANDALONE_ARM_SPREAD_CM:
            self.auto_armed_ms = now_ms
            self.arm_pos_min = position_cm
            self.arm_pos_max = position_cm
            return

        if time.ticks_diff(now_ms, self.auto_armed_ms) >= STANDALONE_ARM_MS:
            self._arm_reset()
            self.auto_runs += 1
            self.last_command = "MODE3-RUN"
            self._enter_seq3(controller, now_ms)

    # ---------------- 每帧推进 ----------------
    def update(self, controller, now_ms, ball_valid):
        if self.mode != MISSION_SEQ3 or self.seq_phase == SEQ3_ENDED:
            return

        self.seq_elapsed_ms = time.ticks_diff(now_ms, self.seq_start_ms)
        if self.seq_elapsed_ms >= SEQ3_TIMEOUT_MS:
            # 超时只停掉成绩判定，仍切入-5cm专用保持，不得
            # 因为5s时限到了就把球留在-4cm或其他错误位置。
            # 【不发 DONE】这一次没做到，报一个假的完成时间比不报更糟。
            self.seq_phase = SEQ3_ENDED
            self.seq_timeout = True
            if not controller.terminal_hold_active:
                self._enter_seq3_terminal_hold(controller)
            return

        if not ball_valid:
            # 丢球时不推进相位：拿过期位置去判到位，序列可能在球还没动
            # 的时候就"完成"了。
            self.seq_stable_ms = None
            self.seq_inside_ms = None
            return

        position_cm = px_to_cm(controller.kalman.position)
        velocity_cm_s = px_s_to_cm_s(controller.kalman.velocity)

        if self.seq_phase == SEQ3_GO_FIRST:
            # 中间点只要求"到过"，不要求停住 —— 停一下再折返是白扔时间。
            #
            # 【必须连续确认】判据吃的是 Kalman 估计位置, 而识别噪声有
            # ±2~3.5px = ±0.16~0.28cm。球真实只到 4.80cm 时, 估计值偶尔跳到
            # 4.90 就判"到过"了 —— 程序以为走够, 实际差 0.2cm, 这就是"每次
            # 都差一点点碰不到线"。单帧噪声不该决定行程。
            progress = SEQ3_FIRST_DIR * position_cm
            if progress >= SEQ3_STROKE_CM + SEQ3_REACH_SURE_CM:
                # 已经确凿越线, 立刻折返, 不再等确认帧
                reached = True
            else:
                if progress >= SEQ3_STROKE_CM - SEQ3_REACH_TOL_CM:
                    if self.seq_reach_cnt < 255:
                        self.seq_reach_cnt += 1
                else:
                    self.seq_reach_cnt = 0
                reached = self.seq_reach_cnt >= SEQ3_REACH_CONFIRM

            gap = SEQ3_STROKE_CM - progress

            # 【停住即折返】离目标已经进了题目的 ±1cm 容差, 又确认不再前进,
            # 就不用再判"到底压没压到线"了 —— 它推不动, 等下去只是白扔第二段
            # 的时间预算。剩下那几毫米对最终得分没有影响, 最终稳定点才计分。
            self.seq_creep.append((now_ms, position_cm))
            while (
                len(self.seq_creep) > 2
                and time.ticks_diff(now_ms, self.seq_creep[0][0])
                > SEQ3_STALL_WIN_MS
            ):
                self.seq_creep.pop(0)

            if not reached and gap <= SEQ3_STALL_GAP_CM:
                n = len(self.seq_creep)
                span = time.ticks_diff(now_ms, self.seq_creep[0][0])
                if n >= SEQ3_STALL_MIN_SAMPLES and span >= SEQ3_STALL_WIN_MS // 2:
                    # 前半/后半各取均值再作差, 把单帧噪声平均掉
                    half = n // 2
                    t_old = 0.0
                    p_old = 0.0
                    for i in range(half):
                        t_old += self.seq_creep[i][0]
                        p_old += self.seq_creep[i][1]
                    t_new = 0.0
                    p_new = 0.0
                    for i in range(n - half, n):
                        t_new += self.seq_creep[i][0]
                        p_new += self.seq_creep[i][1]
                    dt_s = (t_new / half - t_old / half) / 1000.0
                    if dt_s > 1e-3:
                        creep_cm_s = ((p_new - p_old) / half) / dt_s
                        # 只看"朝目标方向还在不在前进", 往回退同样算停住
                        if SEQ3_FIRST_DIR * creep_cm_s <= SEQ3_STALL_CREEP_CM_S:
                            reached = True

            # 兜底: 卡死在更远处(已超出题目容差), 只能保证整轮不作废。
            if not reached:
                if gap > SEQ3_REACH_STALL_TOL2_CM:
                    self.seq_stable_ms = None
                elif self.seq_stable_ms is None:
                    self.seq_stable_ms = now_ms
                    self.seq_pos_min = position_cm
                    self.seq_pos_max = position_cm
                else:
                    if position_cm < self.seq_pos_min:
                        self.seq_pos_min = position_cm
                    if position_cm > self.seq_pos_max:
                        self.seq_pos_max = position_cm
                    if (
                        self.seq_pos_max - self.seq_pos_min
                        > SEQ3_REACH_STALL_SPREAD_CM
                    ):
                        # 还在动, 从当前点重新开窗
                        self.seq_stable_ms = now_ms
                        self.seq_pos_min = position_cm
                        self.seq_pos_max = position_cm
                    else:
                        held = time.ticks_diff(now_ms, self.seq_stable_ms)
                        reached = held >= SEQ3_REACH_STALL_MS2

            if reached:
                self.seq_phase = SEQ3_GO_SECOND
                self.seq_stable_ms = None
                self.seq_inside_ms = None
                self.seq_reach_cnt = 0
                self.seq_creep = []
                # 对外任务目标保持理论±5cm；仅控制器内部多走一小段，补偿
                # 本机第二段在静摩擦下稳定停在目标右侧约0.3cm的欠行程。
                self.target_cm = -SEQ3_FIRST_DIR * SEQ3_STROKE_CM
                controller.set_target_px(cm_to_px(
                    -SEQ3_FIRST_DIR
                    * (SEQ3_STROKE_CM + SEQ3_LEG2_TARGET_EXTRA_CM)
                ))
                controller.neg_to_pos_soft_active = SEG_SOFT_ENABLE
                controller.seg_drive_dir = -SEQ3_FIRST_DIR
                controller.seg_speed_cap_px_s = SEG_LEG2_SPEED_CAP_PX_S
                controller.seg_drive_accel_limit_cm_s2 = SEG_LEG2_DRIVE_ACCEL_CM_S2
                controller.seg_near_zone_cm = -1.0      # 第二段用全局收力区
                controller.seg_velocity_profile_active = True
                controller.seg_profile_speed_cm_s = 0.0
                controller.seg_stall_candidate_ms = None
            return

        # 终点要求"稳定"：位置进容差还不够，速度也得压下来并连续保持，
        # 否则高速穿过终点的那一帧也会被判成到位。
        error_cm = position_cm - (-SEQ3_FIRST_DIR * SEQ3_STROKE_CM)
        inside = abs(error_cm) <= SEQ3_STABLE_TOL_CM
        loose = abs(error_cm) <= SEQ3_STABLE_FALLBACK_TOL_CM

        # 先进保持、后报完成：进入终点前捕获区且速度已可控时，
        # 立即退出大角度行程控制，用专用终点闭环抓住它。稳定计时
        # 从切入这一刻重新开始，防止之前的“穿过窗口”时间被算进去。
        if (
            abs(error_cm) <= SEQ3_TERMINAL_CAPTURE_TOL_CM
            and abs(velocity_cm_s) <= SEQ3_TERMINAL_CAPTURE_SPEED_CM_S
            and not controller.terminal_hold_active
        ):
            self._enter_seq3_terminal_hold(controller)
            self.seq_stable_ms = now_ms
            self.seq_inside_ms = now_ms
            self.seq_pos_min = position_cm
            self.seq_pos_max = position_cm

        if loose:
            if self.seq_inside_ms is None:
                self.seq_inside_ms = now_ms
        else:
            self.seq_inside_ms = None

        # 判"球停住了"看窗口内位置的极差, 不看 Kalman 速度 —— 速度估计里的
        # innovation/dt 会把 1 像素的识别抖动放大成几十 px/s 的尖峰, 那条
        # 判据实测几乎从不成立, 每次都落到兜底上白等一秒。
        if not inside:
            self.seq_stable_ms = None
        elif self.seq_stable_ms is None:
            self.seq_stable_ms = now_ms
            self.seq_pos_min = position_cm
            self.seq_pos_max = position_cm
        else:
            if position_cm < self.seq_pos_min:
                self.seq_pos_min = position_cm
            if position_cm > self.seq_pos_max:
                self.seq_pos_max = position_cm
            if self.seq_pos_max - self.seq_pos_min > SEQ3_STABLE_SPREAD_CM:
                # 波动超标: 从当前点重新开窗, 不彻底作废 —— 球正在收敛时
                # 作废会让计时反复从头开始。
                self.seq_stable_ms = now_ms
                self.seq_pos_min = position_cm
                self.seq_pos_max = position_cm

        settled_by_strict_window = (
            self.seq_stable_ms is not None
            and time.ticks_diff(now_ms, self.seq_stable_ms)
            >= SEQ3_STABLE_DWELL_MS
        )
        settled_by_fallback = (
            self.seq_inside_ms is not None
            and time.ticks_diff(now_ms, self.seq_inside_ms)
            >= SEQ3_INSIDE_DWELL_MS
        )
        settled = (
            controller.terminal_hold_active
            and abs(velocity_cm_s) <= SEQ3_DONE_MAX_SPEED_CM_S
            and (settled_by_strict_window or settled_by_fallback)
        )
        if settled:
            # 保持器已经连续验证合格，只结束计时，不关闭终点闭环。
            self.seq_phase = SEQ3_ENDED
            self.done_pending = True

    def take_done_flag(self):
        if self.done_pending:
            self.done_pending = False
            return True
        return False

    def status_text(self):
        if self.returning_to_start:
            return "RETURN"
        if LINK_ENABLE:
            if self.link_state == LINK_STATE_PREPARING:
                return "P%d-CENTER" % self.prepared_task_id
            if self.link_state == LINK_STATE_CENTER6:
                return "C6-CENTER"
            if self.link_state == LINK_STATE_CENTER6_READY:
                return "C6-READY"
            if self.link_state == LINK_STATE_LOCKING6:
                return "T6-LOCKING"
            if self.link_state == LINK_STATE_READY:
                return "T%d-READY" % self.prepared_task_id
        if self.task6_wait_button:
            return "T6-KEY"
        if self.active_task_id == 4:
            return "T4-HOLD"
        if self.active_task_id == 5:
            return "T5-HOLD"
        if self.active_task_id == 6 and self.mode6_locked:
            return "T6-LOCK"

        if self.active_task_id == 3 or (not LINK_ENABLE and K230_TEST_MODE == 3):
            if self.mode == MISSION_SEQ3:
                if self.seq_phase == SEQ3_GO_FIRST:
                    return "M3-A"
                if self.seq_phase == SEQ3_GO_SECOND:
                    return "M3-B"
                return "M3-TMO" if self.seq_timeout else "M3-OK"
            return "M3-ARM" if self.auto_armed_ms is not None else "M3-WAIT"

        if not LINK_ENABLE:
            if K230_TEST_MODE == 4:
                return "M4-HOLD"
            if K230_TEST_MODE == 5:
                return "M5-HOLD"
            if K230_TEST_MODE == 6:
                return "M6-LOCK" if self.mode6_locked else "M6-KEY"

        return "IDLE"


class Task3SerialTuner:
    """UART4任务3调参协议，帧格式统一为 [PAYLOAD]\n。"""

    def __init__(self):
        self.last_telemetry_ms = None

    def _send(self, uart, payload):
        if uart is None:
            return False
        try:
            uart.write(("[%s]\n" % str(payload)).encode())
            return True
        except Exception as error:
            print("tune uart tx warning:", repr(error))
            return False

    def send_ready(self, uart):
        self._send(uart, "TUNE,READY,115200")
        self._send(uart, "BUILD,T3-PROFILE7-SOFTBRAKE-HOLD8P8,V15")
        self._send(uart, "HELP,GET|SET,name,value|RUN|CENTER|OFF|STATUS|PING")

    def send_parameters(self, uart):
        namespace = globals()
        for spec in TASK3_TUNE_PARAM_SPECS:
            value = namespace[spec[1]]
            self._send(
                uart,
                "PAR,%s,%.6g,%.6g,%.6g"
                % (spec[0], value, spec[2], spec[3]),
            )
        self._send(uart, "PAR,END")

    def send_status(self, uart, mission, controller, motor_pulses, ball_valid):
        position_cm = (
            px_to_cm(controller.kalman.position)
            if controller.kalman.ready else 0.0
        )
        velocity_cm_s = (
            px_s_to_cm_s(controller.kalman.velocity)
            if controller.kalman.ready else 0.0
        )
        phase = mission.status_text()
        if mission.tune_hold_only:
            phase = "CENTER" if mission.motor_active else "OFF"
        self._send(
            uart,
            "STAT,%s,%d,%+.3f,%+.3f,%+.3f,%+.2f,%+d,%d,%+.2f,%d,%.2f,%d,%+.2f,%d,%d,%.2f"
            % (
                phase,
                int(mission.seq_elapsed_ms),
                position_cm,
                velocity_cm_s,
                mission.target_cm,
                controller.angle_command_deg,
                int(motor_pulses),
                1 if ball_valid else 0,
                controller.static_recovery_angle_deg,
                1 if controller.seg_stall_recovery_active else 0,
                controller.seg_remaining_cm,
                1 if controller.terminal_hold_active else 0,
                controller.terminal_hold_bias_deg,
                1 if controller.terminal_stall_active else 0,
                int(controller.terminal_stall_phase),
                controller.seg_profile_speed_cm_s,
            ),
        )
    def handle(self, command, uart, mission, controller, now_ms, motor_pulses):
        parts = [item.strip() for item in str(command).strip().split(",")]
        if not parts or not parts[0]:
            return
        action = parts[0].upper()

        if action in ("?", "HELP"):
            self.send_ready(uart)
            return
        if action == "PING":
            self._send(uart, "PONG,%d" % int(now_ms))
            return
        if action == "GET":
            self.send_parameters(uart)
            return
        if action == "STATUS":
            self.send_status(
                uart, mission, controller, motor_pulses,
                controller.kalman.ready,
            )
            return
        if action == "SET":
            if len(parts) != 3:
                self._send(uart, "ERR,SET_FORMAT")
                return
            alias = parts[1].upper()
            ok, value = task3_tune_set_parameter(alias, parts[2], controller)
            if not ok:
                self._send(uart, "ERR,BAD_PARAM,%s" % alias)
                return
            self._send(uart, "ACK,SET,%s,%.6g" % (alias, value))
            return
        if action == "RUN":
            # 先闭环回O并等待稳定，再由原本autorun逻辑启动任务3。
            mission.tune_hold_only = False
            mission.auto_wait_ms = None
            mission._arm_reset()
            mission._enter_hold_center(controller)
            mission.active_task_id = 3
            self._send(uart, "ACK,RUN,CENTER_FIRST")
            return
        if action == "CENTER":
            mission.tune_hold_only = True
            mission.auto_wait_ms = None
            mission._arm_reset()
            mission._enter_hold_center(controller)
            mission.active_task_id = 3
            self._send(uart, "ACK,CENTER")
            return
        if action == "OFF":
            mission.tune_hold_only = True
            mission.auto_wait_ms = None
            mission._arm_reset()
            mission._enter_off(controller)
            self._send(uart, "ACK,OFF")
            return

        self._send(uart, "ERR,UNKNOWN,%s" % action)

    def notify_done(self, uart, mission, controller):
        position_cm = (
            px_to_cm(controller.kalman.position)
            if controller.kalman.ready else 0.0
        )
        self._send(
            uart,
            "EVT,DONE,%d,%+.3f"
            % (int(mission.seq_elapsed_ms), position_cm),
        )

    def service(self, uart, mission, controller, now_ms, motor_pulses, ball_valid):
        if self.last_telemetry_ms is not None and (
            time.ticks_diff(now_ms, self.last_telemetry_ms)
            < TASK3_TUNE_TELEMETRY_MS
        ):
            return
        self.last_telemetry_ms = now_ms
        self.send_status(uart, mission, controller, motor_pulses, ball_valid)



# ============================== 灰度帧绘制 ==============================
def draw_cross_gray(img, x, y, color=PREDICT_COLOR):
    x = int(x)
    y = int(y)
    img.draw_line(x - 6, y, x + 6, y, color=color, thickness=2)
    img.draw_line(x, y - 6, x, y + 6, color=color, thickness=2)


def draw_gray_overlay(
    img,
    detection,
    predicted_x_control,
    target_px_control,
    mission_text,
    position_cm,
    fps,
    candidate_count,
    miss_count,
    threshold_value,
    task6_waiting=False,
    task6_locked=False,
    task6_target_cm=None,
):
    # ROI框和摆杆横向中心线。
    img.draw_rectangle(
        ROI_X, ROI_Y, ROI_W, ROI_H,
        color=ROI_COLOR,
        thickness=1,
    )
    img.draw_line(
        ROI_X, ROD_CENTER_Y,
        ROI_X + ROI_W, ROD_CENTER_Y,
        color=110,
        thickness=1,
    )

    # -5/0/+5cm刻度。
    for mark_cm in DISPLAY_SCALE_MARKS_CM:
        x_full = int(control_to_full_x(cm_to_px(mark_cm)) + 0.5)
        is_center = abs(mark_cm) < 1e-6
        color = CENTER_COLOR if is_center else SCALE_COLOR
        thickness = 2 if is_center else 1
        img.draw_line(
            x_full, ROI_Y - 14,
            x_full, ROI_Y + ROI_H + 5,
            color=color,
            thickness=thickness,
        )
        label = "0cm" if is_center else ("-5cm" if mark_cm < 0 else "+5cm")
        img.draw_string_advanced(
            x_full - (16 if is_center else 22),
            ROI_Y - 35,
            16,
            label,
            color=color,
        )

    # 当前任务目标线。
    target_full = int(control_to_full_x(target_px_control) + 0.5)
    img.draw_line(
        target_full, ROI_Y - 4,
        target_full, ROI_Y + ROI_H + 4,
        color=TARGET_COLOR,
        thickness=2,
    )

    # 传统视觉检测框。
    if detection is not None:
        _, x1, y1, x2, y2 = detection
        fx1 = int(control_to_full_x(x1) + 0.5)
        fy1 = int(control_to_full_y(y1) + 0.5)
        fx2 = int(control_to_full_x(x2) + 0.5)
        fy2 = int(control_to_full_y(y2) + 0.5)
        img.draw_rectangle(
            fx1, fy1,
            max(1, fx2 - fx1),
            max(1, fy2 - fy1),
            color=BALL_COLOR,
            thickness=3,
        )

        # 任务6把当前/已锁定位置直接标在钢球框上方，便于读数。
        if task6_waiting or task6_locked:
            if task6_locked and task6_target_cm is not None:
                label_text = "LOCK %+.2fcm" % task6_target_cm
                label_color = TASK6_LOCK_COLOR
            else:
                label_text = "KEY %+.2fcm" % position_cm
                label_color = TASK6_WAIT_COLOR

            label_w = 150
            label_x = int(clamp(fx1 - 48, 2, DISPLAY_WIDTH - label_w - 2))
            label_y = fy1 - TASK6_LABEL_HEIGHT - 4
            if label_y < 52:
                label_y = fy2 + 5

            try:
                img.draw_rectangle(
                    label_x, label_y, label_w, TASK6_LABEL_HEIGHT,
                    color=0, fill=True,
                )
            except Exception:
                img.draw_rectangle(
                    label_x, label_y, label_w, TASK6_LABEL_HEIGHT,
                    color=0, thickness=3,
                )
            img.draw_rectangle(
                label_x, label_y, label_w, TASK6_LABEL_HEIGHT,
                color=label_color, thickness=2,
            )
            img.draw_string_advanced(
                label_x + 4, label_y + 2, TASK6_LABEL_FONT_SIZE,
                label_text, color=label_color,
            )
            img.draw_line(
                int((fx1 + fx2) * 0.5), label_y + TASK6_LABEL_HEIGHT,
                int((fx1 + fx2) * 0.5), fy1,
                color=label_color, thickness=2,
            )

    if predicted_x_control is not None:
        draw_cross_gray(
            img,
            int(control_to_full_x(predicted_x_control) + 0.5),
            ROD_CENTER_Y,
            PREDICT_COLOR,
        )

    if threshold_value >= 0:
        threshold_text = "TH:%d" % int(threshold_value)
    else:
        threshold_text = "TH:ADP"

    img.draw_string_advanced(
        8, 8, 18,
        "%s  X:%+.2fcm  FPS:%.1f" % (
            mission_text, position_cm, fps,
        ),
        color=TEXT_COLOR,
    )
    img.draw_string_advanced(
        8, 31, 16,
        "OpenCV gray  cand:%d miss:%d %s" % (
            candidate_count, miss_count, threshold_text,
        ),
        color=TEXT_COLOR,
    )

    if task6_waiting:
        img.draw_string_advanced(
            170, 56, 22,
            "T6: PLACE BALL -> PRESS K230 KEY",
            color=TASK6_WAIT_COLOR,
        )
    elif task6_locked and task6_target_cm is not None:
        img.draw_string_advanced(
            245, 56, 22,
            "T6 LOCKED %+.2fcm" % task6_target_cm,
            color=TASK6_LOCK_COLOR,
        )


# ============================== RTSP / Wi-Fi辅助函数 ==============================
def check_result(result, message):
    if result not in (0, None):
        raise RuntimeError(message + ", 返回值=" + str(result))


def set_wifi_default_device():
    try:
        network.set_default_dev("w0")
    except BaseException as error:
        print("[WiFi] default device warning:", repr(error))


def connect_wifi(ssid, password, timeout_s=30):
    """连接2.4GHz热点；失败时抛异常，由main决定关闭图传继续控制。"""
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)

    try:
        wlan.config(auto_reconnect=True)
    except BaseException:
        pass

    if not wlan.isconnected():
        print("[WiFi] connecting:", ssid)
        wlan.connect(ssid, password)
        start_ms = time.ticks_ms()
        last_print_ms = start_ms
        while not wlan.isconnected():
            os.exitpoint()
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_print_ms) >= 1000:
                print("[WiFi] waiting", time.ticks_diff(now_ms, start_ms) // 1000, "s")
                last_print_ms = now_ms
            if time.ticks_diff(now_ms, start_ms) >= timeout_s * 1000:
                try:
                    wlan.disconnect()
                except BaseException:
                    pass
                raise OSError("Wi-Fi连接超时，请检查SSID/密码并确认热点为2.4GHz")
            time.sleep_ms(100)

    dhcp_start_ms = time.ticks_ms()
    while wlan.ifconfig()[0] == "0.0.0.0":
        os.exitpoint()
        if time.ticks_diff(time.ticks_ms(), dhcp_start_ms) >= 10000:
            raise OSError("Wi-Fi已连接，但DHCP获取IP超时")
        time.sleep_ms(100)

    set_wifi_default_device()
    print("[WiFi] connected, ifconfig:", wlan.ifconfig())
    return wlan


def create_h264_channel_attr(encoder, width, height):
    """优先使用新版可设置码率/GOP/帧率的ChnAttrStr。"""
    try:
        return ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_MAIN,
            width,
            height,
            VIDEO_BITRATE_KBPS,
            VIDEO_GOP,
            VIDEO_SOURCE_FPS,
            VIDEO_OUTPUT_FPS,
        )
    except TypeError as error:
        print("[RTSP] full ChnAttrStr unsupported, fallback:", repr(error))
        return ChnAttrStr(
            encoder.PAYLOAD_TYPE_H264,
            encoder.H264_PROFILE_MAIN,
            width,
            height,
        )


def pump_rtsp_stream(encoder, stream_data, rtsp_server, stats):
    """非阻塞搬运最多一帧H.264码流，不允许图传阻塞PID主循环。"""
    if encoder is None or stream_data is None or rtsp_server is None:
        return

    try:
        result = encoder.GetStream(
            stream_data,
            timeout=RTSP_GETSTREAM_TIMEOUT_MS,
        )
    except TypeError:
        try:
            result = encoder.GetStream(stream_data, RTSP_GETSTREAM_TIMEOUT_MS)
        except TypeError:
            # 固件若只支持无限阻塞接口，为保护闭环控制，直接跳过而不调用。
            if not stats.get("api_warning_printed", False):
                print("[RTSP] 当前GetStream不支持timeout=0，已停止发送以避免阻塞PID")
                stats["api_warning_printed"] = True
            return
    except Exception:
        return

    if result not in (0, None):
        return

    acquired = True
    frame_bytes = 0
    sent_any = False
    try:
        pack_count = int(stream_data.pack_cnt)
        for pack_index in range(pack_count):
            packet_size = int(stream_data.data_size[pack_index])
            if packet_size <= 0:
                continue

            packet_data = bytes(
                uctypes.bytearray_at(
                    stream_data.data[pack_index],
                    packet_size,
                )
            )
            packet_pts = stream_data.pts[pack_index]
            if packet_pts is None or packet_pts <= 0:
                packet_pts = (
                    stats.get("total_frames", 0) * 1000
                ) // max(1, VIDEO_OUTPUT_FPS)

            send_result = rtsp_server.rtspserver_sendvideodata(
                RTSP_SESSION,
                packet_data,
                packet_size,
                packet_pts,
            )
            if send_result not in (0, None):
                stats["send_errors"] = stats.get("send_errors", 0) + 1
            frame_bytes += packet_size
            sent_any = True

        if sent_any:
            stats["total_frames"] = stats.get("total_frames", 0) + 1
            stats["period_frames"] = stats.get("period_frames", 0) + 1
            stats["period_bytes"] = stats.get("period_bytes", 0) + frame_bytes
    finally:
        if acquired:
            try:
                encoder.ReleaseStream(stream_data)
            except Exception as error:
                stats["release_errors"] = stats.get("release_errors", 0) + 1


def print_rtsp_stats(stats, now_ms):
    start_ms = stats.get("period_start_ms", now_ms)
    elapsed_ms = time.ticks_diff(now_ms, start_ms)
    if elapsed_ms < RTSP_STATS_PERIOD_MS:
        return
    elapsed_s = max(0.001, elapsed_ms / 1000.0)
    fps = stats.get("period_frames", 0) / elapsed_s
    mbps = stats.get("period_bytes", 0) * 8.0 / elapsed_s / 1000000.0
    print(
        "[RTSP] FPS=%.1f bitrate=%.2fMbps frames=%d errors=%d"
        % (
            fps,
            mbps,
            stats.get("total_frames", 0),
            stats.get("send_errors", 0),
        )
    )
    stats["period_frames"] = 0
    stats["period_bytes"] = 0
    stats["period_start_ms"] = now_ms


def main():
    sensor = None
    motor_uart = None
    link_uart = None
    task6_button = None
    button_filter = DebouncedButton(TASK6_BUTTON_DEBOUNCE_MS)
    motor_pulses = 0
    motor_powered = False
    mission = None
    car_feedforward = None
    task3_tuner = None

    wifi = None
    encoder = None
    media_link = None
    rtsp_server = None
    stream_data = None

    display_ready = False
    media_ready = False
    sensor_ready = False
    encoder_created = False
    encoder_started = False
    rtsp_initialized = False
    rtsp_session_created = False
    rtsp_started = False
    stream_runtime_enabled = False

    # 所有可能引用媒体缓冲区的对象都在函数顶部声明，finally中可统一清空。
    frame = None
    gray_np = None
    blurred_roi = None
    candidates = None
    circle_candidates = None
    target_candidate = None
    latest_detection = None

    try:
        print("OpenCV traditional detector ready")
        print("cv2.findContours:", hasattr(cv2, "findContours"))

        motor_uart = create_motor_uart()
        motor_enable(motor_uart, True)
        time.sleep_ms(100)
        motor_zero_position(motor_uart)
        time.sleep_ms(20)
        motor_pulses = motor_position_emm(motor_uart, 0)
        last_motor_command_ms = time.ticks_ms()
        if LINK_ENABLE:
            # Initialization may establish the motor zero reference, but the
            # driver must not hold or move the rod before an MSPM0 command.
            motor_enable(motor_uart, False)
            motor_powered = False
        else:
            motor_powered = True

        link_uart = (
            create_link_uart()
            if (LINK_ENABLE or TASK3_UART_TUNE_ENABLE) else None
        )
        link_rx = LinkReceiver()
        car_feedforward = CarMotionFeedforward()

        try:
            task6_button = create_task6_button()
        except Exception as error:
            task6_button = None
            print("task6 button init warning:", repr(error))

        print(
            "motor: UART2 IO11/IO12 115200 ID=1 Emm-FD limit=+/-%.4fdeg"
            % MOTOR_SOFT_LIMIT_DEG
        )
        print("K230 local test mode:", K230_TEST_MODE)
        print(
            "uart4: TX=GPIO%d RX=GPIO%d baud=%d mode=%s"
            % (
                LINK_UART_TX_IO, LINK_UART_RX_IO, LINK_BAUDRATE,
                "TASK3_TUNE" if TASK3_UART_TUNE_ENABLE else "MSPM0_3507",
            )
        )
        print(
            "vision: CH0 GRAYSCALE %dx%d, LCD+IDE shared"
            % (DISPLAY_WIDTH, DISPLAY_HEIGHT)
        )
        print(
            "ROI=(%d,%d,%d,%d), calibration full px: %.2f %.2f %.2f"
            % (
                ROI_X, ROI_Y, ROI_W, ROI_H,
                CAL_LEFT_FULL_PX, CAL_CENTER_FULL_PX, CAL_RIGHT_FULL_PX,
            )
        )

        detector = OpenCVSteelBallDetector()
        controller = CascadeController()
        mission = MissionController()
        mission.configure_local_mode(controller)
        if LINK_ENABLE:
            link_send_text(link_uart, "IDLE")
        elif TASK3_UART_TUNE_ENABLE:
            task3_tuner = Task3SerialTuner()
            task3_tuner.send_ready(link_uart)
        if not LINK_ENABLE:
            mission.active_task_id = K230_TEST_MODE
            mission.task_start_motor_pulses = int(motor_pulses)

        # Wi-Fi失败不影响滚球控制：只关闭本次图传并继续运行。
        if WIRELESS_STREAM_ENABLE == 1:
            try:
                wifi = connect_wifi(WIFI_SSID, WIFI_PASSWORD, timeout_s=30)
                stream_runtime_enabled = True
            except Exception as error:
                print("[RTSP] Wi-Fi startup failed, stream disabled:", repr(error))
                stream_runtime_enabled = False
        else:
            print("[RTSP] disabled by WIRELESS_STREAM_ENABLE=0")

        sensor = Sensor(id=SENSOR_ID)
        sensor.reset()
        try:
            sensor.set_hmirror(SENSOR_HMIRROR)
            sensor.set_vflip(SENSOR_VFLIP)
        except Exception as error:
            print("mirror/vflip warning:", repr(error))

        # 通道0：灰度检测与本地显示。
        sensor.set_framesize(
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            chn=VISION_CHANNEL,
        )
        sensor.set_pixformat(
            Sensor.GRAYSCALE,
            chn=VISION_CHANNEL,
        )

        # 通道1：完整800×480原彩画面，直接绑定H.264编码器。
        if stream_runtime_enabled:
            try:
                sensor.set_framesize(
                    width=STREAM_WIDTH,
                    height=STREAM_HEIGHT,
                    chn=STREAM_CHANNEL,
                    alignment=12,
                )
                sensor.set_pixformat(
                    Sensor.YUV420SP,
                    chn=STREAM_CHANNEL,
                )

                encoder = Encoder()
                encoder.SetOutBufs(
                    VENC_BUFFER_NUM,
                    STREAM_WIDTH,
                    STREAM_HEIGHT,
                )
                channel_attr = create_h264_channel_attr(
                    encoder,
                    STREAM_WIDTH,
                    STREAM_HEIGHT,
                )
                encoder.Create(channel_attr)
                encoder_created = True

                media_link = MediaManager.link(
                    sensor.bind_info(chn=STREAM_CHANNEL)["src"],
                    (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, encoder.chn),
                )

                rtsp_server = mm.rtsp_server()
                check_result(
                    rtsp_server.rtspserver_init(RTSP_PORT),
                    "RTSP端口初始化失败",
                )
                rtsp_initialized = True
                check_result(
                    rtsp_server.rtspserver_createsession(
                        RTSP_SESSION,
                        mm.multi_media_type.media_h264,
                        False,
                    ),
                    "RTSP会话创建失败",
                )
                rtsp_session_created = True
                stream_data = StreamData()

                print(
                    "[RTSP] CH1 full original color output=%dx%d"
                    % (STREAM_WIDTH, STREAM_HEIGHT)
                )
                print(
                    "[RTSP] quality bitrate=%dkbps fps=%d gop=%d"
                    % (VIDEO_BITRATE_KBPS, VIDEO_OUTPUT_FPS, VIDEO_GOP)
                )
            except Exception as error:
                print("[RTSP] pipeline setup failed, stream disabled:", repr(error))
                try:
                    sys.print_exception(error)
                except BaseException:
                    pass
                if media_link is not None:
                    try:
                        media_link.destroy()
                    except BaseException:
                        pass
                    media_link = None
                if encoder_created and encoder is not None:
                    try:
                        encoder.Destroy()
                    except BaseException:
                        pass
                encoder_created = False
                encoder = None
                if rtsp_initialized and rtsp_server is not None:
                    try:
                        rtsp_server.rtspserver_deinit()
                    except BaseException:
                        pass
                rtsp_initialized = False
                rtsp_session_created = False
                rtsp_server = None
                stream_data = None
                stream_runtime_enabled = False

        # 不做bind_layer；灰度帧由Python直接送LCD和IDE。
        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            to_ide=ENABLE_IDE_PREVIEW,
        )
        display_ready = True

        MediaManager.init()
        media_ready = True

        if stream_runtime_enabled:
            rtsp_server.rtspserver_start()
            rtsp_started = True
            encoder.Start()
            encoder_started = True

        sensor.run()
        sensor_ready = True
        time.sleep_ms(500)

        if stream_runtime_enabled:
            try:
                rtsp_url = rtsp_server.rtspserver_getrtspurl(RTSP_SESSION)
            except BaseException:
                rtsp_url = "rtsp://%s:%d/%s" % (
                    wifi.ifconfig()[0], RTSP_PORT, RTSP_SESSION
                )
            print("[RTSP] play URL:", rtsp_url)

        latest_confidence = 0.0
        control_state = None
        last_detection_ms = None
        consecutive_misses = 0
        frame_index = 0
        control_count = 0
        detect_count = 0
        cv_ms_ema = 0.0
        print_start_ms = time.ticks_ms()
        last_gc_ms = print_start_ms
        last_link_tx_ms = print_start_ms
        clock = time.clock()
        rtsp_stats = {
            "total_frames": 0,
            "period_frames": 0,
            "period_bytes": 0,
            "send_errors": 0,
            "release_errors": 0,
            "period_start_ms": print_start_ms,
        }

        while True:
            os.exitpoint()
            clock.tick()

            frame = sensor.snapshot(chn=VISION_CHANNEL)
            capture_ms = time.ticks_ms()
            if frame is None:
                if stream_runtime_enabled:
                    pump_rtsp_stream(encoder, stream_data, rtsp_server, rtsp_stats)
                continue

            frame_index += 1
            control_count += 1

            if last_detection_ms is None:
                capture_track_age_ms = 9999
            else:
                capture_track_age_ms = time.ticks_diff(capture_ms, last_detection_ms)

            ball_valid_now = (
                controller.kalman.ready
                and capture_track_age_ms <= PREDICTION_HOLD_MS
            )

            for command in link_rx.poll(link_uart):
                print("[UART4 RX]", command)
                if car_feedforward.on_command(command, capture_ms):
                    continue
                if TASK3_UART_TUNE_ENABLE and task3_tuner is not None:
                    task3_tuner.handle(
                        command, link_uart, mission, controller,
                        capture_ms, motor_pulses,
                    )
                else:
                    mission.on_command(
                        command, controller, capture_ms, motor_pulses
                    )

            # 回位状态机不阻塞主循环；回位命令只发送一次。
            mission.service_return(controller, capture_ms)
            mission.service_link_state(controller, capture_ms, ball_valid_now)
            event = mission.take_transition_event()
            if event is not None:
                if event[0] == "RETURN":
                    motor_pulses = motor_position_emm(
                        motor_uart, event[1], TASK_RETURN_SPEED_RPM
                    )
                    last_motor_command_ms = capture_ms
                    link_send_text(link_uart, "RETURN")
                elif event[0] == "RETURN_DONE":
                    motor_pulses = int(event[1])
                    last_motor_command_ms = capture_ms
                    link_send_text(link_uart, "IDLE")
                elif event[0] == "START":
                    link_send_text(link_uart, "R%d" % event[1])
                elif event[0] == "T6_WAIT":
                    link_send_text(link_uart, "P6")
                elif event[0] == "IDLE":
                    link_send_text(link_uart, "IDLE")
                elif event[0] == "READY":
                    link_send_text(link_uart, "R%d" % event[1])
                elif event[0] == "PREP":
                    link_send_text(link_uart, "PREP%d" % event[1])
                elif event[0] == "CENTER6":
                    link_send_text(link_uart, "CENTER6")
                elif event[0] == "RUN":
                    link_send_text(link_uart, "RUN%d" % event[1])
                elif event[0] == "SET6":
                    link_send_text(link_uart, "SET6")
                elif event[0] == "TASK6_READY":
                    link_send_task6_ready(link_uart, event[1])
                elif event[0] == "CENTER6_READY":
                    link_send_text(link_uart, "C6OK")
                elif event[0] == "ERROR":
                    link_send_text(link_uart, "ERR,%s" % event[1])

            mission.autorun(controller, capture_ms, ball_valid_now)
            mission.update(controller, capture_ms, ball_valid_now)

            # 车辆前馈只允许在3507已经正式放行的任务4/5/6运行阶段生效。
            # STOP、准备页、回位、任务3或串口采样超时都会自动释放到0。
            car_ff_active = (
                LINK_ENABLE
                and mission.link_state == LINK_STATE_RUNNING
                and mission.active_task_id in (4, 5, 6)
                and mission.motor_active
            )
            car_ff_state_valid = (
                ball_valid_now and controller.kalman.ready
            )
            car_ff_position_error_px = 0.0
            car_ff_velocity_px_s = 0.0
            if car_ff_state_valid:
                car_ff_position_error_px = (
                    controller.target_px - controller.kalman.position
                )
                car_ff_velocity_px_s = controller.kalman.velocity
            controller.set_external_feedforward_deg(
                car_feedforward.service(
                    capture_ms,
                    car_ff_active,
                    car_ff_position_error_px,
                    car_ff_velocity_px_s,
                    car_ff_state_valid,
                )
            )

            # Power follows the serial mission state.  P/C/G commands enable
            # the actuator; IDLE/S and power-on keep it disabled.  Return-to-
            # start remains powered until its non-blocking move is complete.
            motor_should_power = (
                mission.motor_active or mission.returning_to_start
            )
            if motor_should_power and not motor_powered:
                motor_enable(motor_uart, True)
                motor_powered = True
                last_motor_command_ms = capture_ms
            elif not motor_should_power and motor_powered:
                motor_enable(motor_uart, False)
                motor_powered = False

            if motor_powered and mission.motor_active and ball_valid_now:
                control_state, motor_pulses, last_motor_command_ms = (
                    update_motor_from_controller(
                        controller,
                        motor_uart,
                        capture_ms,
                        motor_pulses,
                        last_motor_command_ms,
                    )
                )

            measurement_ms = ticks_shift(capture_ms, -CAMERA_PIPELINE_DELAY_MS)
            predicted_for_detection = controller.predicted_state(
                measurement_ms,
                include_motor_delay=False,
            )
            if predicted_for_detection is None:
                predicted_x_control = None
                predicted_x_full = CAL_CENTER_FULL_PX
                tracking_valid = False
            else:
                predicted_x_control = predicted_for_detection[0]
                predicted_x_full = control_to_full_x(predicted_x_control)
                tracking_valid = True

            gray_np = frame.to_numpy_ref()
            cv_start_ms = time.ticks_ms()

            candidates, blurred_roi = detector.detect_contours(gray_np)
            candidates = merge_close_candidates(candidates)
            target_candidate = select_best_candidate(
                candidates,
                predicted_x_full,
                tracking_valid,
            )

            circle_candidates = None
            if (
                target_candidate is None
                and ENABLE_HOUGH_REACQUIRE
                and consecutive_misses + 1 >= HOUGH_START_AFTER_MISSES
                and (consecutive_misses % HOUGH_REACQUIRE_INTERVAL) == 0
            ):
                os.exitpoint()
                circle_candidates = detector.hough_reacquire(blurred_roi)
                if circle_candidates:
                    circle_candidates = merge_close_candidates(circle_candidates)
                    hough_target = select_best_candidate(
                        circle_candidates,
                        predicted_x_full,
                        tracking_valid,
                    )
                    if hough_target is not None:
                        target_candidate = hough_target
                        candidates.extend(circle_candidates)

            cv_end_ms = time.ticks_ms()
            cv_ms = max(0, time.ticks_diff(cv_end_ms, cv_start_ms))
            cv_ms_ema += 0.18 * (cv_ms - cv_ms_ema)
            detect_count += 1

            if target_candidate is not None:
                detection = candidate_to_detection(target_candidate)
                measured_position_px = ball_center_px(detection)[0]
                controller.correct_measurement(
                    measured_position_px,
                    measurement_ms,
                    detection[0],
                )
                latest_detection = detection
                latest_confidence = detection[0]
                last_detection_ms = measurement_ms
                consecutive_misses = 0
            else:
                consecutive_misses += 1
                latest_confidence = 0.0

            # 必须在显示、停止sensor和MediaManager之前释放零拷贝引用。
            gray_np = None
            blurred_roi = None
            circle_candidates = None

            now_ms = time.ticks_ms()
            if last_detection_ms is None:
                track_age_ms = 9999
            else:
                track_age_ms = time.ticks_diff(now_ms, last_detection_ms)

            if not mission.motor_active:
                # 等待任务6按键、任务回位或空闲时保持最后一次绝对位置命令，
                # 不能每帧强制回0，否则会覆盖“回任务起始位置”的命令。
                control_state = None
            elif controller.kalman.ready and track_age_ms <= PREDICTION_HOLD_MS:
                control_state, motor_pulses, last_motor_command_ms = (
                    update_motor_from_controller(
                        controller,
                        motor_uart,
                        now_ms,
                        motor_pulses,
                        last_motor_command_ms,
                    )
                )
            else:
                control_state = None
                if motor_pulses != 0:
                    motor_pulses = motor_position_emm(motor_uart, 0)
                    last_motor_command_ms = now_ms
                if track_age_ms >= BALL_LOST_RESET_MS:
                    controller.reset()
                    latest_detection = None
                    latest_confidence = 0.0
                    consecutive_misses = 0

            ball_valid = (
                controller.kalman.ready
                and track_age_ms <= PREDICTION_HOLD_MS
            )

            # 任务6：按下板载按键时，读取当前钢球横向位置并立即锁定。
            if button_filter.poll_pressed(task6_button, now_ms):
                print("[KEY] task6 button pressed")
                mission.capture_task6_button(controller, now_ms, ball_valid)
                event = mission.take_transition_event()
                if event is not None:
                    if event[0] == "START":
                        link_send_text(link_uart, "R6")
                    elif event[0] == "TASK6_READY":
                        link_send_task6_ready(link_uart, event[1])
                    elif event[0] == "T6_ERROR":
                        link_send_text(link_uart, "ERR,NO_BALL")

            if mission.take_done_flag():
                if LINK_ENABLE:
                    link_send_done(link_uart)
                    link_send_text(link_uart, "D3")
                    last_link_tx_ms = now_ms
                elif TASK3_UART_TUNE_ENABLE and task3_tuner is not None:
                    task3_tuner.notify_done(link_uart, mission, controller)
            elif LINK_ENABLE and (
                time.ticks_diff(now_ms, last_link_tx_ms)
                >= LINK_POSITION_INTERVAL_MS
            ):
                link_send_position(
                    link_uart,
                    px_to_cm(controller.kalman.position) if ball_valid else 0.0,
                    ball_valid,
                )
                last_link_tx_ms = now_ms

            if TASK3_UART_TUNE_ENABLE and task3_tuner is not None:
                task3_tuner.service(
                    link_uart, mission, controller, now_ms,
                    motor_pulses, ball_valid,
                )

            predicted_display_x = (
                control_state[0] if control_state is not None else (
                    controller.kalman.position if controller.kalman.ready else None
                )
            )
            position_cm = (
                px_to_cm(predicted_display_x)
                if predicted_display_x is not None
                else 0.0
            )

            draw_gray_overlay(
                frame,
                latest_detection,
                predicted_display_x,
                controller.target_px,
                mission.status_text(),
                position_cm,
                clock.fps(),
                len(candidates) if candidates is not None else 0,
                consecutive_misses,
                detector.last_dark_threshold,
                task6_waiting=mission.task6_wait_button,
                task6_locked=mission.mode6_locked,
                task6_target_cm=mission.task6_target_cm,
            )

            if frame_index % DISPLAY_INTERVAL == 0:
                Display.show_image(frame, x=0, y=0)

            # 图传通道由VENC异步编码；这里只做一次非阻塞码流搬运。
            if stream_runtime_enabled:
                pump_rtsp_stream(encoder, stream_data, rtsp_server, rtsp_stats)
                print_rtsp_stats(rtsp_stats, now_ms)

            elapsed_ms = time.ticks_diff(now_ms, print_start_ms)
            if elapsed_ms >= TERMINAL_INTERVAL_MS:
                fps = control_count * 1000.0 / elapsed_ms
                cv_fps = detect_count * 1000.0 / elapsed_ms

                if control_state is not None:
                    print(
                        "[%s#%d arm=%dms tgt=%+.1fcm pos=%+.2fcm] "
                        "FPS=%.1f cv=%.1f cv_ms=%.1f src=%s q=%.2f "
                        "cand=%d miss=%d th=%.0f x=%.1f e=%+.1f v=%+.1f "
                        "angle=%+.3f pulse=%+d rpm=%d"
                        % (
                            mission.status_text(),
                            mission.auto_runs,
                            mission.arm_progress_ms(now_ms),
                            mission.target_cm,
                            px_to_cm(control_state[0]),
                            fps,
                            cv_fps,
                            cv_ms_ema,
                            target_candidate.get("source", "-") if target_candidate else "-",
                            latest_confidence,
                            len(candidates) if candidates is not None else 0,
                            consecutive_misses,
                            detector.last_dark_threshold,
                            control_state[0],
                            control_state[2],
                            control_state[1],
                            motor_pulses * MOTOR_DEG_PER_PULSE,
                            motor_pulses,
                            control_state[23],
                        )
                    )
                else:
                    print(
                        "[%s#%d arm=%dms tgt=%+.1fcm] "
                        "FPS=%.1f cv=%.1f cv_ms=%.1f ball=%d age=%dms "
                        "cand=%d miss=%d th=%.0f"
                        % (
                            mission.status_text(),
                            mission.auto_runs,
                            mission.arm_progress_ms(now_ms),
                            mission.target_cm,
                            fps,
                            cv_fps,
                            cv_ms_ema,
                            1 if ball_valid else 0,
                            track_age_ms,
                            len(candidates) if candidates is not None else 0,
                            consecutive_misses,
                            detector.last_dark_threshold,
                        )
                    )

                control_count = 0
                detect_count = 0
                print_start_ms = now_ms

            target_candidate = None
            candidates = None
            frame = None

            if time.ticks_diff(now_ms, last_gc_ms) >= GC_INTERVAL_MS:
                gc.collect()
                last_gc_ms = now_ms

    except KeyboardInterrupt:
        print("user stop")

    except BaseException as error:
        print("runtime error type:", type(error))
        print("runtime error:", repr(error))
        try:
            sys.print_exception(error)
        except Exception:
            pass

    finally:
        print("[STOP] 1/11 release frame references")
        frame = None
        gray_np = None
        blurred_roi = None
        candidates = None
        circle_candidates = None
        target_candidate = None
        latest_detection = None
        stream_data = None
        gc.collect()
        time.sleep_ms(30)

        print("[STOP] 2/11 return motor to task start and stop")
        if motor_uart is not None:
            try:
                return_target = mission.shutdown_return_target() if mission is not None else 0
                wait_ms = task_return_wait_ms(motor_pulses, return_target)
                print(
                    "[STOP] motor %+d -> %+d pulses, wait=%dms"
                    % (int(motor_pulses), int(return_target), wait_ms)
                )
                motor_position_emm(
                    motor_uart, return_target, TASK_RETURN_SPEED_RPM
                )
                time.sleep_ms(wait_ms)
                motor_enable(motor_uart, False)
            except Exception as error:
                print("motor stop warning:", repr(error))

        print("[STOP] 3/11 close UART")
        if motor_uart is not None:
            try:
                motor_uart.deinit()
            except Exception:
                pass
            motor_uart = None
        if link_uart is not None:
            try:
                link_uart.deinit()
            except Exception:
                pass
            link_uart = None

        print("[STOP] 4/11 stop sensor")
        if sensor_ready and sensor is not None:
            try:
                sensor.stop()
            except BaseException as error:
                print("sensor stop warning:", repr(error))
        sensor_ready = False

        print("[STOP] 5/11 stop encoder")
        if encoder_started and encoder is not None:
            try:
                encoder.Stop()
            except BaseException as error:
                print("encoder.Stop warning:", repr(error))
        encoder_started = False

        print("[STOP] 6/11 destroy media link")
        if media_link is not None:
            try:
                media_link.destroy()
            except BaseException as error:
                print("media_link.destroy warning:", repr(error))
            media_link = None

        print("[STOP] 7/11 destroy encoder")
        if encoder_created and encoder is not None:
            try:
                encoder.Destroy()
            except BaseException as error:
                print("encoder.Destroy warning:", repr(error))
        encoder_created = False
        encoder = None

        print("[STOP] 8/11 stop RTSP")
        if rtsp_started and rtsp_server is not None:
            try:
                rtsp_server.rtspserver_stop()
            except BaseException as error:
                print("rtspserver_stop warning:", repr(error))
        rtsp_started = False
        if rtsp_session_created and rtsp_server is not None:
            try:
                rtsp_server.rtspserver_destroysession(RTSP_SESSION)
            except BaseException as error:
                print("rtsp destroy session warning:", repr(error))
        rtsp_session_created = False
        if rtsp_initialized and rtsp_server is not None:
            try:
                rtsp_server.rtspserver_deinit()
            except BaseException as error:
                print("rtspserver_deinit warning:", repr(error))
        rtsp_initialized = False
        if rtsp_server is not None:
            try:
                rtsp_server.rtspserver_destroy()
            except BaseException:
                pass
        rtsp_server = None

        print("[STOP] 9/11 deinit display")
        if display_ready:
            try:
                Display.deinit()
            except BaseException as error:
                print("display deinit warning:", repr(error))
        display_ready = False

        print("[STOP] 10/11 deinit media")
        if media_ready:
            try:
                time.sleep_ms(80)
                MediaManager.deinit()
            except BaseException as error:
                print("MediaManager.deinit warning:", repr(error))
        media_ready = False
        sensor = None

        print("[STOP] 11/11 disconnect Wi-Fi and collect memory")
        if wifi is not None:
            try:
                if wifi.isconnected():
                    wifi.disconnect()
            except BaseException as error:
                print("Wi-Fi disconnect warning:", repr(error))
        wifi = None
        gc.collect()
        print("program end")


main()
