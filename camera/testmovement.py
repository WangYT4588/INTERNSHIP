import serial, time

# ‼️ 確保你的 COM 埠是正確的
PORT = "COM3"
BAUD = 115200

def send_cmd(ser, cmd: str, duration_s: float):
    """發送命令並等待"""
    print(f"發送: {cmd}")
    ser.write((cmd + "\n").encode('ascii'))
    
    # 讀取 STM32 的 ACK 回傳
    start_time = time.monotonic()
    ack_received = False
    while time.monotonic() - start_time < 0.5: # 等待 0.5 秒
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('ascii').strip()
                if line.startswith("ACK:"):
                    print(f"  -> 收到: {line}")
                    ack_received = True
                    break
            except Exception as e:
                print(f"  -> 讀取錯誤: {e}")
                break
    if not ack_received:
        print("  -> 警告：未收到 STM32 的 ACK 回傳！")

    print(f"  -> 保持轉動 {duration_s} 秒...")
    time.sleep(duration_s)
    
    # 發送停止命令
    ser.write(b"STOP\n")
    print("  -> 發送: STOP")
    time.sleep(0.5) # 停止後的緩衝時間

def main_test():
    print(f"正在連接 {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print("連接成功！")
        print("--- ‼️‼️ 最終硬體除錯 ‼️‼️ ---")
        print("請將車子架空，輪子即將轉動...")
        print("你「必須」有一張紙筆，記下「純前進」時四顆輪子的實際轉向。")
        time.sleep(3)

        # --- 測試 C：純前進 ---
        print("\n" + "="*30)
        print("[TEST C] 測試「純前進」(CMD:VEL,0.2,0.0,0.0)")
        print("  -> 理論上：四顆輪子都必須「正轉」(讓車子前進)。")
        print("  -> ‼️ 請用紙筆記下 FL, FR, RL, RR 實際是「正轉」還是「反轉」‼️")
        print("="*30)
        send_cmd(ser, "CMD:VEL,0.2,0.0,0.0", 3.0)

        # --- 測試 D：純左平移 ---
        print("\n" + "="*30)
        print("[TEST D] 測試「純左平移」(CMD:VEL,0.0,0.2,0.0)")
        print("  -> 理論上 (O-Pattern)：FL/RR 應「正轉」，FR/RL 應「反轉」。")
        print("="*30)
        send_cmd(ser, "CMD:VEL,0.0,0.2,0.0", 3.0)
        
        # --- 測試 E：純右平移 ---
        print("\n" + "="*30)
        print("[TEST E] 測試「純右平移」(CMD:VEL,0.0,-0.2,0.0)")
        print("  -> 理論上 (O-Pattern)：FL/RR 應「反轉」，FR/RL 應「正轉」。")
        print("="*30)
        send_cmd(ser, "CMD:VEL,0.0,-0.2,0.0", 3.0)

        print("\n--- 測試完畢 ---")
        print("請把你在 [TEST C] 記下的筆記回傳給我。")
        print("例如：『FL: 正轉, FR: 反轉, RL: 反轉, RR: 正轉』")

    except Exception as e:
        print(f"\n[錯誤] 測試失敗: {e}")
        print(f"請檢查：1. COM 埠號是否正確？ 2. STM32 是否已燒錄並插電？")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("序列埠已關閉。")

if __name__ == "__main__":
    main_test()