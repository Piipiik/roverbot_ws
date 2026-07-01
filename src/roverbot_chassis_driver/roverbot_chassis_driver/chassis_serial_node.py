#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import threading
import struct
import time
from typing import Optional

# ====================== 协议常量（与下位机完全对齐，禁止修改）======================
FRAME_HEAD = 0xA5          # 帧头
FRAME_TAIL = 0x5A          # 帧尾

# 帧类型定义
TYPE_SPEED_CMD = 0x01      # 下发：速度指令帧
TYPE_SPEED_FEEDBACK = 0x81 # 上传：当前速度反馈帧（3个float，12字节）
TYPE_COUNTER = 0x82        # 上传：计数器帧（1个uint32，4字节）

# 速度指令载荷长度（3个float = 12字节）
SPEED_PAYLOAD_LEN = 12

# 合法帧类型集合（用于快速校验）
VALID_FRAME_TYPES = {TYPE_SPEED_CMD, TYPE_SPEED_FEEDBACK, TYPE_COUNTER}


class ChassisSerialNode(Node):
    def __init__(self):
        super().__init__('chassis_serial_node')

        # ====================== 1. ROS 参数声明与校验 ======================
        self.declare_parameter('serial_port', '/dev/jlink_chassis')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_period', 0.1)          # 发送间隔，默认10Hz
        self.declare_parameter('reconnect_interval', 1.0)   # 串口断开后重连间隔

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.send_period = self.get_parameter('send_period').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        # 参数合法性校验
        if self.send_period <= 0:
            self.get_logger().warn(f'发送周期 {self.send_period}s 非法，已重置为默认 0.1s')
            self.send_period = 0.1
        if self.baudrate <= 0:
            raise ValueError(f'波特率 {self.baudrate} 非法，节点启动失败')

        self.get_logger().info('='*50)
        self.get_logger().info('底盘串口驱动节点启动中...')
        self.get_logger().info(f'串口设备: {self.serial_port}')
        self.get_logger().info(f'波特率: {self.baudrate}')
        self.get_logger().info(f'指令发送周期: {self.send_period}s')

        # ====================== 2. 全局变量与线程安全组件 ======================
        # 目标速度（多线程访问，加锁保护）
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.speed_lock = threading.Lock()

        # 发送帧序号（0~255循环，仅ROS主线程定时器修改，无并发竞争）
        self.send_seq = 0

        # 线程运行标志（Event原子操作，更安全的线程通信）
        self._running = threading.Event()
        self._running.set()

        # 串口实例
        self.ser: Optional[serial.Serial] = None

        # ====================== 3. ROS 话题 ======================
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('订阅话题: /cmd_vel [geometry_msgs/Twist]')

        self.current_vel_pub = self.create_publisher(Twist, 'current_velocity', 10)
        self.counter_pub = self.create_publisher(Int32, '/chassis_counter', 10)
        self.get_logger().info('发布话题: current_velocity [速度反馈]')
        self.get_logger().info('发布话题: /chassis_counter [运行计数器]')

        # ====================== 4. 串口初始化 ======================
        if not self._open_serial():
            self.get_logger().warn('首次串口打开失败，将进入自动重连流程')

        # ====================== 5. 启动接收线程 ======================
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True, name='serial_recv')
        self.recv_thread.start()
        self.get_logger().info('📥 串口接收线程启动')

        # ====================== 6. 启动发送定时器 ======================
        self.send_timer = self.create_timer(self.send_period, self._send_speed_cmd)
        self.get_logger().info(f'📤 串口发送定时器启动 | 间隔 {self.send_period}s')

        self.get_logger().info('='*50)
        self.get_logger().info('🎉 底盘串口驱动节点启动完成')

    # ====================== 工具方法：串口开关 ======================
    def _open_serial(self) -> bool:
        """打开串口，返回是否打开成功"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            self.get_logger().info(f'✅ 串口打开成功 | 设备 {self.serial_port} 波特率 {self.baudrate}')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 串口打开失败: {str(e)}')
            self.ser = None
            return False

    def _close_serial(self):
        """安全关闭串口，屏蔽关闭异常"""
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    # ====================== 工具方法：帧构造 ======================
    @staticmethod
    def _build_speed_frame(vx: float, vy: float, wz: float, seq: int) -> bytearray:
        """
        构造速度指令帧
        帧结构：帧头(1) + 类型(1) + 长度(1) + 序号(1) + 速度数据(12) + 校验(1) + 帧尾(1) = 18字节
        """
        frame = bytearray()
        frame.append(FRAME_HEAD)
        frame.append(TYPE_SPEED_CMD)
        frame.append(SPEED_PAYLOAD_LEN)
        frame.append(seq & 0xFF)
        frame.extend(struct.pack('<fff', vx, vy, wz))  # 小端模式float

        # 校验和：帧头之后、校验位之前所有字节累加取低8位
        checksum = sum(frame[1:]) & 0xFF
        frame.append(checksum)
        frame.append(FRAME_TAIL)
        return frame

    # ====================== 话题回调：接收速度指令 ======================
    def cmd_vel_callback(self, msg: Twist):
        with self.speed_lock:
            old_vx = self.target_vx
            old_vy = self.target_vy
            old_wz = self.target_wz
            self.target_vx = msg.linear.x
            self.target_vy = msg.linear.y
            self.target_wz = msg.angular.z

        # 任意轴速度变化时打INFO日志，无变化打DEBUG，避免刷屏
        speed_changed = (
            abs(old_vx - msg.linear.x) > 1e-6
            or abs(old_vy - msg.linear.y) > 1e-6
            or abs(old_wz - msg.angular.z) > 1e-6
        )
        if speed_changed:
            self.get_logger().info(
                f'🎯 更新目标速度 | vx={msg.linear.x:.3f}  vy={msg.linear.y:.3f}  wz={msg.angular.z:.3f}'
            )
        else:
            self.get_logger().debug(
                f'收到速度指令 | vx={msg.linear.x:.3f}  vy={msg.linear.y:.3f}  wz={msg.angular.z:.3f}'
            )

    # ====================== 定时发送：速度指令帧 ======================
    def _send_speed_cmd(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().debug('串口未打开，跳过本次发送')
            return

        # 线程安全读取目标速度
        with self.speed_lock:
            vx = self.target_vx
            vy = self.target_vy
            wz = self.target_wz

        # 构造帧并发送
        frame = self._build_speed_frame(vx, vy, wz, self.send_seq)
        self.send_seq = (self.send_seq + 1) & 0xFF

        try:
            self.ser.write(frame)
            # 仅当DEBUG级别启用时打印发送帧（不消耗性能）
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                hex_str = ' '.join(f'{b:02x}' for b in frame)
                self.get_logger().debug(f'[SEND] 序号 {self.send_seq-1:3d} | Hex: {hex_str}')
        except Exception as e:
            self.get_logger().error(f'❌ 串口发送失败: {str(e)}')
            self._close_serial()  # 发送异常触发关闭，等待重连

    # ====================== 后台线程：串口接收循环 ======================
    def _recv_loop(self):
        buffer = bytearray()
        self.get_logger().debug('接收线程进入主循环')

        while self._running.is_set():
            # 串口未打开时，周期性重连
            if self.ser is None or not self.ser.is_open:
                if self._open_serial():
                    buffer.clear()  # 重连成功后清空旧数据，避免干扰
                    self.get_logger().info('串口重连成功，恢复数据收发')
                else:
                    time.sleep(self.reconnect_interval)
                continue

            try:
                # 批量读取所有可用数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    # 原始数据日志仅当调试级别极高时启用（此处注释掉，减少刷屏）
                    # self.get_logger().debug('原始接收 ' + ...)

                # 索引指针解析帧（避免逐字节pop的O(n)性能损耗）
                parse_pos = 0
                buffer_len = len(buffer)
                # 最小完整帧长度：头+类型+长度+校验+尾 = 5字节
                while buffer_len - parse_pos >= 5:
                    # 1. 查找帧头
                    if buffer[parse_pos] != FRAME_HEAD:
                        parse_pos += 1
                        continue

                    # 2. 读取帧头后基础字段，并进行合法性校验
                    frame_type = buffer[parse_pos + 1]
                    # 类型不合法，跳过当前帧头继续搜索
                    if frame_type not in VALID_FRAME_TYPES:
                        self.get_logger().debug(f'未知帧类型 0x{frame_type:02x}，跳过该帧头')
                        parse_pos += 1
                        continue

                    payload_len = buffer[parse_pos + 2]
                    if payload_len > 64 or payload_len <= 0:
                        self.get_logger().debug(f'异常载荷长度 {payload_len}，跳过帧头')
                        parse_pos += 1
                        continue

                    # 计算完整帧总长度
                    total_frame_len = payload_len + 6
                    if buffer_len - parse_pos < total_frame_len:
                        break  # 数据不足，等待下一轮读取

                    # 3. 截取完整一帧
                    frame_end = parse_pos + total_frame_len
                    full_frame = buffer[parse_pos:frame_end]
                    parse_pos = frame_end

                    # 4. 校验帧尾
                    if full_frame[-1] != FRAME_TAIL:
                        # 帧尾不匹配降级为DEBUG，不频繁警告
                        self.get_logger().debug(
                            f'帧尾不匹配 (期望 0x{FRAME_TAIL:02x})，帧起始偏移 {parse_pos-total_frame_len}'
                        )
                        continue

                    # 5. 校验和校验
                    recv_checksum = full_frame[-2]
                    calc_checksum = sum(full_frame[1:-2]) & 0xFF
                    if recv_checksum != calc_checksum:
                        self.get_logger().warn(
                            f'⚠️ 校验和错误 | 接收 0x{recv_checksum:02x} 计算 0x{calc_checksum:02x}'
                        )
                        continue

                    # 6. 分发解析载荷
                    payload = full_frame[4:-2]
                    try:
                        self._parse_frame(frame_type, payload)
                    except Exception as e:
                        self.get_logger().error(f'帧解析异常: {str(e)}')

                # 移除已解析的前缀数据，保留剩余未解析部分
                if parse_pos > 0:
                    buffer = buffer[parse_pos:]

                # 小让步避免CPU空转
                time.sleep(0.001)

            except Exception as e:
                self.get_logger().error(f'❌ 接收线程异常: {str(e)}')
                self._close_serial()
                time.sleep(0.1)

        self.get_logger().info('接收线程已退出')

    # ====================== 帧解析分发 ======================
    def _parse_frame(self, frame_type: int, payload: bytes):
        """根据帧类型解析数据并发布ROS话题"""

        # 速度反馈帧
        if frame_type == TYPE_SPEED_FEEDBACK:
            if len(payload) != 12:
                self.get_logger().warn(f'速度帧长度异常，期望12字节，实际{len(payload)}字节')
                return

            try:
                vx, vy, wz = struct.unpack('<fff', payload)
            except struct.error as e:
                self.get_logger().warn(f'速度帧数据解包失败: {e}')
                return

            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
            self.current_vel_pub.publish(msg)

            # 速度反馈正常，使用DEBUG输出（不刷屏）
            self.get_logger().debug(
                f'[RECV] 当前速度 | vx={vx:7.3f}  vy={vy:7.3f}  wz={wz:7.3f}'
            )

        # 计数器帧
        elif frame_type == TYPE_COUNTER:
            if len(payload) != 4:
                self.get_logger().warn(f'计数器帧长度异常，期望4字节，实际{len(payload)}字节')
                return

            try:
                cnt = struct.unpack('<I', payload)[0]
            except struct.error as e:
                self.get_logger().warn(f'计数器帧数据解包失败: {e}')
                return

            msg = Int32()
            msg.data = cnt
            self.counter_pub.publish(msg)

            self.get_logger().debug(f'[RECV] 计 数 器 | 数值 = {cnt}')

        # 未知帧类型（理论上不会发生，因为已在 _recv_loop 过滤）
        else:
            self.get_logger().debug(
                f'收到未知帧类型 0x{frame_type:02x} | 载荷长度 {len(payload)}'
            )

    # ====================== 节点退出清理 ======================
    def destroy_node(self):
        self.get_logger().info('收到退出信号，正在清理资源...')
        self._running.clear()  # 通知所有子线程退出

        # 停止发送定时器
        if hasattr(self, 'send_timer'):
            self.send_timer.cancel()

        # 安全停机：退出前强制下发零速度
        if self.ser is not None and self.ser.is_open:
            try:
                zero_frame = self._build_speed_frame(0.0, 0.0, 0.0, self.send_seq)
                self.ser.write(zero_frame)
                self.get_logger().info('已下发零速度指令，底盘安全停机')
                time.sleep(0.05)  # 等待数据发送完成
            except Exception:
                pass

        # 等待接收线程退出
        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
            if self.recv_thread.is_alive():
                self.get_logger().warn('接收线程超时未退出，强制结束')
            else:
                self.get_logger().info('接收线程已退出')

        # 关闭串口
        self._close_serial()
        self.get_logger().info('串口已关闭')

        super().destroy_node()
        self.get_logger().info('节点正常退出')


# ====================== 主入口 ======================
def main(args=None):
    rclpy.init(args=args)
    node = ChassisSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户按下Ctrl+C，退出中...')
    finally:
        node.destroy_node()
        # 确保只调用一次shutdown，避免重复关闭错误
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()