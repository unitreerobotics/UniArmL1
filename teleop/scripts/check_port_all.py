import serial
import serial.tools.list_ports
import struct
import time


def list_available_ports():
    """只列出电脑上当前连接的 AT32 ACM 电机控制板串口。"""
    ports = serial.tools.list_ports.comports()
    acm_ports = [port for port in ports if port.device.startswith("/dev/ttyACM")]
    motor_ports = []

    print("--- 当前可用 ACM 端口列表 ---")
    if not acm_ports:
        print("未发现任何 ACM 端口，请检查驱动及硬件连接。")
        return []

    for i, port in enumerate(acm_ports):
        print(f"[{i}] 端口名: {port.device} | 描述: {port.description}")
        desc = (port.description or "").lower()
        if "camera" in desc:
            print(f"    ↳ 跳过相机控制接口: {port.device}")
            continue
        if "at32" not in desc:
            print(f"    ↳ 跳过非 AT32 端口: {port.device}")
            continue
        motor_ports.append(port)

    print("\n--- 将扫描的 AT32 电机端口 ---")
    if not motor_ports:
        print("未发现 AT32 电机控制板端口。")
    else:
        for port in motor_ports:
            print(f"  {port.device} | {port.description}")
    return motor_ports


def crc32_core(data_bytes):
    """J288/S288 专用 CRC32 校验。"""
    if len(data_bytes) % 4 != 0:
        return 0

    crc = 0xFFFFFFFF
    poly = 0x04C11DB7

    for i in range(0, len(data_bytes), 4):
        word = struct.unpack('<I', data_bytes[i:i+4])[0]
        xbit = 0x80000000

        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF

            if word & xbit:
                crc = (crc ^ poly) & 0xFFFFFFFF

            xbit >>= 1

    return crc


def build_scan_packet(motor_id):
    """构造扫描指定 motor_id 的命令包。"""
    head = b'\xFE\xEE'
    mode_byte = (motor_id & 0x0F) | (0 << 4) | (0 << 7)

    # 按原协议填充 14 字节 payload
    payload = struct.pack('<B B h h I h h', mode_byte, 0, 0, 0, 0, 0, 0)
    full_data = head + payload
    checksum = crc32_core(full_data)
    packet = full_data + struct.pack('<I', checksum)
    return packet


def scan_motor_ids(port_name, id_range=range(15)):
    """
    在指定端口上扫描多个电机 ID。
    返回该端口发现的所有电机 ID 列表。
    """
    found_ids = []

    try:
        with serial.Serial(port_name, 6000000, timeout=0.02, write_timeout=0.02) as ser:
            print(f"\n正在端口 {port_name} 上扫描电机 ID {min(id_range)}-{max(id_range)} ...")

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            for motor_id in id_range:
                packet = build_scan_packet(motor_id)

                try:
                    ser.write(packet)
                    ser.flush()
                except serial.SerialTimeoutException:
                    print(f"  写入超时，跳过端口 {port_name}")
                    break
                time.sleep(0.01)

                response = ser.read(ser.in_waiting or 0)

                if len(response) >= 26 and b'\xFC\xEE' in response:
                    print(f"  ✅ 发现电机 ID: {motor_id}")
                    found_ids.append(motor_id)

                ser.reset_input_buffer()

        if found_ids:
            print(f"端口 {port_name} 扫描完成，发现电机 ID: {found_ids}")
        else:
            print(f"端口 {port_name} 扫描完成，未发现响应电机。")

    except Exception as e:
        print(f"错误: 无法打开端口 {port_name}。原因: {e}")

    return found_ids


def find_all_motor_ports(ports, id_range=range(15)):
    """
    遍历所有端口，找出所有能响应的端口及其对应的电机 ID。
    返回格式:
    {
        "/dev/ttyACM0": [1, 3],
        "/dev/ttyACM1": [0, 2, 4]
    }
    """
    all_results = {}

    for port in ports:
        print(f"\n=== 正在尝试端口: {port.device} ({port.description}) ===")
        found_ids = scan_motor_ids(port.device, id_range=id_range)

        if found_ids:
            all_results[port.device] = found_ids

    print("\n================ 最终扫描结果 ================")
    if all_results:
        total_motors = 0
        for port_name, ids in all_results.items():
            print(f"✅ 端口 {port_name} 上发现电机 ID: {ids}")
            total_motors += len(ids)
        print(f"\n共发现 {len(all_results)} 个有效端口，{total_motors} 个电机。")
    else:
        print("❌ 未找到任何能响应电机的端口。")

    return all_results


if __name__ == "__main__":
    available_ports = list_available_ports()
    if available_ports:
        results = find_all_motor_ports(available_ports, id_range=range(15))
        print("\n返回结果字典：")
        print(results)
