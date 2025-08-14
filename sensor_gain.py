from pymavlink import mavutil

# 连接到 SITL 端口（即 udpout 发送的）
conn = mavutil.mavlink_connection('udpin:localhost:14552')

print("等待心跳...")
conn.wait_heartbeat()
print(f"连接成功：System ID {conn.target_system}, Component ID {conn.target_component}")

while True:
    msg = conn.recv_match(type='HEARTBEAT', blocking=True)
    print(f"[{msg._timestamp:.1f}] HEARTBEAT from {msg.get_srcSystem()}/{msg.get_srcComponent()}")
