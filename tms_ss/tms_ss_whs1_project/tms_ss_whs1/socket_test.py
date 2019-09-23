import socket

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.bind(('192.168.4.87', 65001))
    while True:
        data, addr = s.recvfrom(1024)

        hakei = int.from_bytes(data[0:2], 'little')
        msec = int.from_bytes(data[2:4], "little")
        temp = int.from_bytes(data[6:8], "little") * 0.01

        acc_x = int.from_bytes(data[4:6], "little") * 0.01
        acc_y = int.from_bytes(data[10:12], "little") * 0.01
        acc_z = int.from_bytes(data[8:10], "little") * 0.01

        print("msec: " + str(temp))

def main(args=None):
    rclpy.init(args=args)
