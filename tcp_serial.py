import socket
import threading
import queue
import serial
import time
import select

# ==== Config ====
SERIAL_PORT_HEADER = "/dev/ttyUSB"
BAUDRATE = 115200
TCP_PORT = 8000

tcp_to_serial_q = queue.Queue()

serial_fd = None
client_socket = None

# ==== Thread 1: Serial -> TCP ====
def serial_data_to_tcp_thread():
    global serial_fd, client_socket
    while True:
        if serial_fd is None:
            time.sleep(1)
            continue
        try:
            ready, _, _ = select.select([serial_fd], [], [], 0.1)
            if ready:
                data = serial_fd.read(128)
                if data and client_socket:
                    client_socket.sendall(data)
        except Exception as e:
            print("Serial read error:", e)
            time.sleep(1)

# ==== Thread 2: Serial Manager ====
def serial_port_thread():
    global serial_fd
    count_fail = 0

    while True:
        if count_fail > 5:
            print("Failed to connect serial after 5 retries")
            return

        for i in range(10):
            port = f"{SERIAL_PORT_HEADER}{i}"
            try:
                serial_fd = serial.Serial(port, BAUDRATE, timeout=0.1)
                print(f"Connected to {port}")
                break
            except serial.SerialException:
                continue

        if not serial_fd or not serial_fd.is_open:
            count_fail += 1
            time.sleep(1)
            continue

        # Start serial -> TCP thread
        threading.Thread(target=serial_data_to_tcp_thread, daemon=True).start()

        # Read from queue (TCP -> Serial)
        while True:
            try:
                msg = tcp_to_serial_q.get(timeout=0.1)
                serial_fd.write(msg)
            except queue.Empty:
                continue
            except serial.SerialException as e:
                print("Serial write error:", e)
                break

# ==== Thread 3: TCP Server ====
def tcp_thread():
    global client_socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", TCP_PORT))
    server_socket.listen(1)
    print(f"TCP server listening on port {TCP_PORT}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Client connected from {addr}")

        while True:
            try:
                data = client_socket.recv(128)
                if not data:
                    print("Client disconnected")
                    client_socket.close()
                    client_socket = None
                    break
                print(f"From TCP: {data.decode(errors='ignore')}")
                tcp_to_serial_q.put(data)
            except ConnectionResetError:
                print("TCP connection lost")
                client_socket = None
                break

# ==== Main ====
if __name__ == "__main__":
    t1 = threading.Thread(target=serial_port_thread, daemon=True)
    t2 = threading.Thread(target=tcp_thread, daemon=True)
    t1.start()
    time.sleep(1)
    t2.start()

    t1.join()
    t2.join()
