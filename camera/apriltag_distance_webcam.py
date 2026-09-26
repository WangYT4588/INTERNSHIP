# python apriltag_distance_webcam.py --camera 1 --backend dshow --params camera_params.npz --tag-size 0.058 --print-interval 1.0
# python apriltag_distance_webcam3.py --camera 0 --backend msmf --params camera_params.npz --tag-size 0.058 --rio-ip 10.227.172.220 --rio-port 5005

import argparse, os, time, math
import cv2, numpy as np
import socket as pysocket

# ---- AprilTag 偵測器 ----
try:
    from pupil_apriltags import Detector
    _DET_BACKEND = "pupil_apriltags"
except Exception:
    from dt_apriltags import Detector
    _DET_BACKEND = "dt_apriltags"
    print("[INFO] Using dt_apriltags (fallback)")

# ---- 串口 ----
try:
    import serial
except ImportError:
    serial = None

PORT = "COM6"   # 請確認你的裝置管理員是 COM6
BAUD = 115200
ip = "10.227.172.220"   # 你的 myRIO IP
port = 5005

HERE = os.path.abspath(os.path.dirname(__file__))
DEFAULT_PARAMS = os.path.join(HERE, "camera_params.npz")

ap = argparse.ArgumentParser()

# ========= 相機參數 =========
ap.add_argument("--camera", default="0")
ap.add_argument("--camera-name", default=None)
ap.add_argument("--backend", default="dshow", choices=["dshow","msmf","any"])
ap.add_argument("--params", default=DEFAULT_PARAMS)
ap.add_argument("--tag-size", type=float, default=0.058)
ap.add_argument("--family", default="tag36h11")
ap.add_argument("--print-interval", dest="print_interval", type=float, default=1.0)

# ========= 控制參數 =========
ap.add_argument("--vmax", type=float, default=0.25)      # 最大線速度
ap.add_argument("--ctrl-hz", type=float, default=30.0)   # 控制頻率 (Hz)
ap.add_argument("--wmax", type=float, default=1.0)

# X（左右） deadband / ramp（單位 mm）
ap.add_argument("--x-dead-mm",  type=float, default=0.1)
ap.add_argument("--x-enter-mm", type=float, default=10.0)
ap.add_argument("--x-exit-mm",  type=float, default=14.0)
ap.add_argument("--x-ramp-mm",  type=float, default=5.0)

# Z（距離 |t|）（單位 mm，這裡預設 300~301mm 為死區）
ap.add_argument("--z-dead-low-mm",  type=float, default=300.0)
ap.add_argument("--z-dead-high-mm", type=float, default=301.0)
ap.add_argument("--z-ramp-mm",      type=float, default=5.0)

# 最小啟動速度
ap.add_argument("--vmin",   type=float, default=0.02)
ap.add_argument("--vmin-x", type=float, default=0.15)
ap.add_argument("--vmin-z", type=float, default=0.15)

# PI / D
ap.add_argument("--kix", type=float, default=0.8)
ap.add_argument("--kiz", type=float, default=0.8)
ap.add_argument("--i-leak", type=float, default=0.5)
ap.add_argument("--i-limit-frac", type=float, default=0.5)
ap.add_argument("--kdx", type=float, default=0.0)
ap.add_argument("--kdz", type=float, default=0.0)
ap.add_argument("--d-lpf-hz", type=float, default=6.0)
ap.add_argument("--x-shape", type=float, default=1.8)
ap.add_argument("--z-shape", type=float, default=1.8)

# === 新增：myRIO TCP 參數（可選） ===
ap.add_argument("--rio-ip", default=None, help="myRIO IP")
ap.add_argument("--rio-port", type=int, default=5005)

args = ap.parse_args()

# === myRIO TCP 連線 (optional) ===
sock = None

def ensure_tcp_connected():
    global sock
    if not args.rio_ip: return
    if sock is not None: return
    try:
        sock = pysocket.create_connection((args.rio_ip, args.rio_port), timeout=1.0)
        sock.setsockopt(pysocket.IPPROTO_TCP, pysocket.TCP_NODELAY, 1)
        print(f"[INFO] TCP connected to myRIO {args.rio_ip}:{args.rio_port}")
    except Exception as e:
        print(f"[WARN] TCP connect failed: {e}")

def tcp_send_line(line: str):
    global sock
    if not args.rio_ip: return
    if sock is None:
        ensure_tcp_connected()
        if sock is None: return
    try:
        sock.sendall(line.encode("ascii"))
    except Exception as e:
        print(f"[WARN] TCP send failed: {e}")
        try:
            sock.close()
        except: pass
        sock = None

# ========= Serial =========
ser = None
if serial is not None:
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f"[INFO] Serial opened: {PORT} @ {BAUD}")
    except Exception as e:
        print(f"[WARN] Cannot open serial {PORT}: {e}")
else:
    print("[ERROR] Pyserial not installed.")

# ========= Camera intrinsics (硬編碼) =========
HAS_INTR = True  # 強制開啟

# 這裡填入你的數值，如果不知道，就用下面的通用估計值
# 假設畫面是 640x480
img_width = 640
img_height = 480

fx = 600.0  # 焦距 (大約是寬度)
fy = 600.0
cx = img_width / 2.0  # 光心 X (通常是寬度的一半)
cy = img_height / 2.0 # 光心 Y (通常是高度的一半)

# 建立矩陣 (給 cv2.undistort 用)
K = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0,  0,  1]
], dtype=np.float32)

# 畸變係數 (假設沒有畸變，填 0 即可)
dist = np.zeros(5, dtype=np.float32)

print(f"[INFO] Using hardcoded intrinsics: fx={fx}, cx={cx}")

def open_capture(src, backend_name):
    be = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}[backend_name]
    cap = cv2.VideoCapture(src, be) if be != cv2.CAP_ANY else cv2.VideoCapture(src)
    if cap.isOpened():
        ok, _ = cap.read()
        if ok: return cap
        cap.release()
    return None

if args.camera_name:
    src = f"video={args.camera_name}"
else:
    src = int(args.camera)

cap = open_capture(src, args.backend)
if cap is None:
    raise SystemExit("Cannot open camera.")

detector = Detector(families=args.family)
palette = [(0,255,255),(255,0,255),(0,165,255),(0,255,0),(255,0,0)]
win = "AprilTag Distance"

# 全域變數
last_print = time.monotonic()
last_ctrl  = 0.0
last_move_cmd = 0.0 
deadzone_start_time = None

# 門檻計算
X_DEAD  = args.x_dead_mm  / 1000.0
X_ENTER = args.x_enter_mm / 1000.0
X_EXIT  = args.x_exit_mm  / 1000.0
X_RAMP  = args.x_ramp_mm  / 1000.0
Z_LOW_ABS  = args.z_dead_low_mm  / 1000.0
Z_HIGH_ABS = args.z_dead_high_mm / 1000.0
Z_CENTER   = 0.5 * (Z_LOW_ABS + Z_HIGH_ABS)
Z_DEAD     = 0.5 * (Z_HIGH_ABS - Z_LOW_ABS)
Z_RAMP     = max(0.001, (args.z_ramp_mm or 0.0) / 1000.0)
V_MIN_X = args.vmin_x if args.vmin_x is not None else args.vmin
V_MIN_Z = args.vmin_z if args.vmin_z is not None else args.vmin

# Yaw 設定
YAW_START_RAD = math.radians(20.0)
YAW_STOP_RAD  = math.radians(2.0)
W_AUTO = args.wmax 
yaw_aligning = False

MODE_ALIGN_X = 0
MODE_MOVE_Z  = 1
mode = MODE_ALIGN_X
ix = iz = 0.0
prev_ex = prev_ez = 0.0
dx_f = dz_f = 0.0

# === 關鍵：手動變數 ===
# 如果這些是 None，代表「自動模式」
# 如果這些是 數字 (包含 0.0)，代表「手動模式」
manual_vx = None
manual_vs = None
manual_wz = None

def leak(acc, dt, rate):
    if rate <= 0: return acc
    return acc * max(0.0, 1.0 - rate * dt)

def _clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def shaped_speed(err, deadband, ramp, vmax, vmin, shape=1.8):
    a = abs(err)
    if a <= deadband: return 0.0
    v0 = max(vmin, 0.2 * vmax)
    return math.copysign(v0, err)

def pi_update(acc, err, deadband, dt, ki, i_limit, cmd_pre_sat, vmax):
    aerr = abs(err)
    signed_over = math.copysign(max(0.0, aerr - deadband), err)
    delta_i = ki * signed_over * dt
    acc += delta_i
    return _clip(acc, -i_limit, i_limit)

def d_lpf(prev, x, dt, hz):
    if hz <= 0: return 0.0
    beta = 1.0 - math.exp(-2.0 * math.pi * hz * dt)
    return prev + beta * (x - prev)

def enforce_sign(cmd, err, deadband):
    if abs(err) <= deadband: return 0.0
    return math.copysign(abs(cmd), err)

def main_loop():
    global last_print, last_ctrl, ser, mode
    global ix, iz, prev_ex, prev_ez, dx_f, dz_f
    global manual_vx, manual_vs, manual_wz, yaw_aligning
    global last_move_cmd, deadzone_start_time

    while True:
        ok, frame = cap.read()
        if not ok: continue

        if HAS_INTR: frame = cv2.undistort(frame, K, dist)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if HAS_INTR and args.tag_size > 0:
            results = detector.detect(gray, estimate_tag_pose=True, 
                                      camera_params=(float(fx), float(fy), float(cx), float(cy)), 
                                      tag_size=float(args.tag_size))
        else:
            results = detector.detect(gray, estimate_tag_pose=False)

        rows = []
        for r in results:
            color = palette[r.tag_id % len(palette)]
            corners = r.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), color, 2)
            cv2.circle(frame, tuple(map(int, r.center)), 4, color, -1)

            yaw = float("nan")
            X = Y = Zc = D = float("nan")

            if HAS_INTR and args.tag_size > 0 and getattr(r, "pose_t", None) is not None:
                t = r.pose_t.reshape(-1).astype(float)
                R = r.pose_R.astype(float)
                X, Y, Zc = t[0], -t[1], t[2]
                D = float(np.linalg.norm(t))
                yaw = math.atan2(R[0,2], R[2,2])
                
                org = (int(corners[:,0].min()), int(max(20, corners[:,1].min() - 10)))
                cv2.putText(frame, f"ID={r.tag_id} D={D*1000:.0f}mm", org, 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            rows.append({"id": int(r.tag_id), "X": X, "D": D, "yaw": yaw})

        # === 顯示目前模式 ===
        if manual_vx is not None:
            # 手動模式
            status_text = "MANUAL / STOP" if (manual_vx==0 and manual_vs==0 and manual_wz==0) else "MANUAL DRIVE"
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            # 自動模式
            cv2.putText(frame, "AUTO PID", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # === 控制邏輯 ===
        now_t = time.monotonic()
        if now_t - last_ctrl >= (1.0 / max(1e-3, args.ctrl_hz)):
            dt = now_t - last_ctrl if last_ctrl > 0 else (1.0 / args.ctrl_hz)
            last_ctrl = now_t

            vx = vs = wz = 0.0
            valid = [r for r in rows if np.isfinite(r["D"])]
            i_limit = args.i_limit_frac * args.vmax
            mode_str = "NO_TAG"

            # ---------------------------------------------------------
            # 1. 優先檢查手動覆蓋
            # ---------------------------------------------------------
            if (manual_vx is not None) or (manual_vs is not None) or (manual_wz is not None):
                vx = 0.0 if manual_vx is None else manual_vx
                vs = 0.0 if manual_vs is None else manual_vs
                wz = 0.0 if manual_wz is None else manual_wz
                mode_str = "MANUAL"
                
                # 手動時，還是讓積分器慢慢漏掉，避免切回自動時暴衝
                ix = leak(ix, dt, args.i_leak)
                iz = leak(iz, dt, args.i_leak)

            # ---------------------------------------------------------
            # 2. 如果沒有手動，且有 Tag，則跑自動 PID
            # ---------------------------------------------------------
            elif valid:
                tgt = min(valid, key=lambda r: r["D"])
                X, D, yaw = tgt["X"], tgt["D"], tgt["yaw"]

                if np.isfinite(yaw):
                    yaw_abs = abs(yaw)
                    if yaw_aligning:
                        if yaw_abs <= YAW_STOP_RAD: yaw_aligning = False
                    else:
                        if yaw_abs >= YAW_START_RAD: yaw_aligning = True
                else:
                    yaw_aligning = False

                if yaw_aligning and np.isfinite(yaw):
                    turn_dir = -1.0 if yaw > 0 else +1.0
                    wz = turn_dir * W_AUTO
                    ix = leak(ix, dt, args.i_leak)
                    iz = leak(iz, dt, args.i_leak)
                    mode_str = "ALIGN_YAW"
                else:
                    # PID Control
                    if mode == MODE_MOVE_Z:
                        if abs(X) >= X_EXIT: mode = MODE_ALIGN_X
                    else:
                        if abs(X) <= X_ENTER: mode = MODE_MOVE_Z

                    if mode == MODE_ALIGN_X:
                        e_x_cmd = X
                        base_vs = shaped_speed(e_x_cmd, X_DEAD, X_RAMP, args.vmax, V_MIN_X, args.x_shape)
                        raw_dx = (e_x_cmd - prev_ex) / max(1e-6, dt)
                        dx_f = d_lpf(dx_f, raw_dx, dt, args.d_lpf_hz)
                        u_d = args.kdx * dx_f
                        ix = pi_update(ix, e_x_cmd, X_DEAD, dt, args.kix, i_limit, base_vs + u_d, args.vmax)
                        vs = _clip(base_vs + u_d + ix, -args.vmax, args.vmax)
                        prev_ex = e_x_cmd
                        mode_str = "ALIGN_X"
                    else:
                        e_d_cmd = D - Z_CENTER
                        if prev_ez * e_d_cmd < 0: iz = 0.0
                        if Z_LOW_ABS <= D <= Z_HIGH_ABS:
                            base_vx = 0.0
                            e_d_eff = 0.0
                        else:
                            e_d_eff = e_d_cmd
                            base_vx = shaped_speed(e_d_eff, Z_DEAD, Z_RAMP, args.vmax, V_MIN_Z, args.z_shape)
                        
                        raw_dz = (e_d_cmd - prev_ez) / max(1e-6, dt)
                        dz_f = d_lpf(dz_f, raw_dz, dt, args.d_lpf_hz)
                        u_d = args.kdz * dz_f
                        iz = pi_update(iz, e_d_eff, Z_DEAD, dt, args.kiz, i_limit, base_vx + u_d, args.vmax)
                        vx_cmd = base_vx + u_d + iz
                        vx = _clip(enforce_sign(vx_cmd, e_d_eff, Z_DEAD), -args.vmax, args.vmax)
                        prev_ez = e_d_cmd
                        mode_str = "MOVE_Z"

            else:
                # 無 Tag，減速停車
                ix = leak(ix, dt, args.i_leak)
                iz = leak(iz, dt, args.i_leak)

            # 死區訊號發送
            deadzone_condition = False
            if valid and mode_str not in ("MANUAL", "ALIGN_YAW") and not yaw_aligning:
                 tgt = min(valid, key=lambda r: r["D"])
                 if (abs(tgt["X"]) <= X_ENTER) and (Z_LOW_ABS <= tgt["D"] <= Z_HIGH_ABS) and (abs(vx)<1e-3 and abs(vs)<1e-3):
                     deadzone_condition = True
            
            if deadzone_condition:
                if deadzone_start_time is None: deadzone_start_time = now_t
                elif now_t - deadzone_start_time >= 0.8:
                    tcp_send_line("1\n")
                    deadzone_start_time = None
            else:
                deadzone_start_time = None

            # 傳送指令
            if ser is not None:
                try:
                    ser.write(f"CMD:VEL,{vx:.3f},{vs:.3f},{wz:.3f}\n".encode("ascii"))
                except Exception: ser = None
            
            # Print Log
            if now_t - last_print >= args.print_interval:
                print(f"[{mode_str}] vx={vx:.2f} vs={vs:.2f} wz={wz:.2f}")
                last_print = now_t

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27: break

        # === 鍵盤邏輯修改 ===
        if key == ord('s'):
            manual_vx = +0.25; manual_vs = 0.0; manual_wz = 0.0
        elif key == ord('w'):
            manual_vx = -0.25; manual_vs = 0.0; manual_wz = 0.0
        elif key == ord('d'):
            manual_vx = 0.0; manual_vs = -0.25; manual_wz = 0.0
        elif key == ord('a'):
            manual_vx = 0.0; manual_vs = +0.25; manual_wz = 0.0
        elif key == ord('q'):
            manual_vx = 0.0; manual_vs = 0.0; manual_wz = +args.wmax
        elif key == ord('e'):
            manual_vx = 0.0; manual_vs = 0.0; manual_wz = -args.wmax
        
        # 按下 F：強制停車 (進入手動模式，但速度為0)
        elif key == ord('f'):
            manual_vx = 0.0
            manual_vs = 0.0
            manual_wz = 0.0
            print("[KEY] F pressed -> FORCE STOP (Manual Hold)")

        # 按下 R：恢復自動 (Resume Auto)
        elif key == ord('r'):
            manual_vx = None
            manual_vs = None
            manual_wz = None
            print("[KEY] R pressed -> RESUME AUTO PID")
            
        elif key in (ord('m'), ord('M')):
            s = pysocket.create_connection((ip, port), timeout=3)
            s.sendall(b"1.0\n")
            s.close()
            print("Sent 1.0 to myRIO")

    if ser is not None:
        ser.write(b"STOP\n")
        ser.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_loop()