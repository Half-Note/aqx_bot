import socket

UDP_IP = "0.0.0.0"     # Listen on all interfaces
UDP_PORT = 8101      # Must match sender

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(65536)  # buffer size
    print(f"Received {len(data)} bytes from {addr}")
    try:
        print("Data (decoded):", data.decode())
    except:
        print("Binary or non-UTF8 data.")
