#!/usr/bin/env python3
"""
Quest VR Socket Monitor
Simple TCP Socket server to receive Quest VR data (JSON format)
Compatible with your existing Quest VR setup (port 5454)
"""

import socket
import json
import threading
import time
from typing import Optional, Dict
import numpy as np


class QuestSocketMonitor:
    """
    Quest VR Socket Monitor - receives Quest JSON data via TCP socket

    Expected JSON format from Quest:
    {
        "timestamp": 123.456,
        "head": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "euler": {"x": 0.0, "y": 0.0, "z": 0.0}
        },
        "left": {
            "enabled": true,
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "euler": {"x": 0.0, "y": 0.0, "z": 0.0},
            "trigger": 0.0
        },
        "right": {...},
        "agv": {"x": 0.0, "y": 0.0},
        "lift": {"enabled": true, "value": 0}
    }
    """

    def __init__(self, host='0.0.0.0', port=5454):
        self.host = host
        self.port = port
        self.running = False
        self.latest_quest_data = None
        self.data_lock = threading.Lock()
        self.socket_thread = None
        self.connected = False

    def start(self):
        """Start socket server in background thread"""
        self.running = True
        self.socket_thread = threading.Thread(target=self._socket_server, daemon=True)
        self.socket_thread.start()
        print(f"[Quest Socket] Server starting on {self.host}:{self.port}")

    def stop(self):
        """Stop socket server"""
        self.running = False
        if self.socket_thread:
            self.socket_thread.join(timeout=2)
        print("[Quest Socket] Server stopped")

    def _socket_server(self):
        """Socket server thread - receives Quest data"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(1.0)  # Allow periodic checking of self.running

        try:
            server.bind((self.host, self.port))
            server.listen(1)
            print(f"[Quest Socket] Listening on {self.host}:{self.port}")
            print(f"[Quest Socket] Waiting for Quest connection...")

            while self.running:
                try:
                    conn, addr = server.accept()
                    self.connected = True
                    print(f"[Quest Socket] ✓ Connected from: {addr}")
                    self._handle_connection(conn)
                except socket.timeout:
                    continue  # Check if still running
                except Exception as e:
                    if self.running:
                        print(f"[Quest Socket] Accept error: {e}")
                        time.sleep(1)

        except Exception as e:
            print(f"[Quest Socket] Server error: {e}")
        finally:
            server.close()
            print("[Quest Socket] Server closed")

    def _handle_connection(self, conn):
        """Handle a single client connection"""
        buffer = ""

        try:
            while self.running:
                try:
                    data = conn.recv(1024)
                    if not data:
                        print("[Quest Socket] Connection closed by client")
                        break

                    buffer += data.decode('utf-8', errors='ignore')
                    lines = buffer.split('\n')

                    # Process complete lines (all except last partial line)
                    for line in lines[:-1]:
                        if line.strip():
                            try:
                                quest_data = json.loads(line)
                                with self.data_lock:
                                    self.latest_quest_data = quest_data
                            except json.JSONDecodeError as e:
                                print(f"[Quest Socket] JSON decode error: {e}")
                                print(f"[Quest Socket] Problematic line: {line[:100]}")

                    # Keep the last partial line in buffer
                    buffer = lines[-1]

                except Exception as e:
                    print(f"[Quest Socket] Receive error: {e}")
                    break

        except Exception as e:
            print(f"[Quest Socket] Connection error: {e}")
        finally:
            conn.close()
            self.connected = False
            print("[Quest Socket] Connection closed")

    def get_latest_data(self) -> Optional[Dict]:
        """Get latest Quest data (thread-safe)"""
        with self.data_lock:
            return self.latest_quest_data.copy() if self.latest_quest_data else None

    def get_dual_controllers(self) -> Dict:
        """Get left and right controller data"""
        data = self.get_latest_data()
        if not data:
            return {
                "left": None,
                "right": None,
                "head": None,
                "agv": None,
                "timestamp": 0
            }

        return {
            "left": data.get("left"),
            "right": data.get("right"),
            "head": data.get("head"),
            "agv": data.get("agv"),
            "lift": data.get("lift"),
            "timestamp": data.get("timestamp", 0)
        }

    def is_connected(self) -> bool:
        """Check if Quest is connected"""
        return self.connected and self.latest_quest_data is not None

    def wait_for_connection(self, timeout=30):
        """Wait for Quest to connect"""
        print(f"[Quest Socket] Waiting for Quest connection (timeout: {timeout}s)...")
        start_time = time.time()

        while not self.is_connected():
            if time.time() - start_time > timeout:
                print(f"[Quest Socket] ✗ Connection timeout!")
                return False
            time.sleep(0.1)

        print(f"[Quest Socket] ✓ Quest connected!")
        return True


# Test mode - run this file directly to test Quest connection
if __name__ == "__main__":
    print("=== Quest Socket Monitor Test ===")
    print("This will receive and print Quest VR data")
    print("Connect your Quest to this PC and start sending data to port 5454")
    print("Press Ctrl+C to stop\n")

    monitor = QuestSocketMonitor()
    monitor.start()

    if not monitor.wait_for_connection(timeout=60):
        print("No Quest connection. Exiting.")
        monitor.stop()
        exit(1)

    try:
        while True:
            data = monitor.get_dual_controllers()

            if data["left"]:
                left_pos = data["left"]["position"]
                print(f"[LEFT] Pos: ({left_pos['x']:.3f}, {left_pos['y']:.3f}, {left_pos['z']:.3f}) "
                      f"Trigger: {data['left']['trigger']:.2f}")

            if data["right"]:
                right_pos = data["right"]["position"]
                print(f"[RIGHT] Pos: ({right_pos['x']:.3f}, {right_pos['y']:.3f}, {right_pos['z']:.3f}) "
                      f"Trigger: {data['right']['trigger']:.2f}")

            if data["head"]:
                head_euler = data["head"]["euler"]
                print(f"[HEAD] Euler: (x={head_euler['x']:.1f}°, y={head_euler['y']:.1f}°, z={head_euler['z']:.1f}°)")

            if data["agv"]:
                print(f"[AGV] x={data['agv']['x']:.2f}, y={data['agv']['y']:.2f}")

            print("-" * 60)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        monitor.stop()
