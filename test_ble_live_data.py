#!/usr/bin/env python3
"""
ESP32 Sensor Shield Data Retrieval Tool
Comprehensive tool for retrieving sensor data via multiple methods:
- Flash/LittleFS data dump
- Serial monitoring
- WiFi Access Point
- BLE live streaming
- BLE dump mode
"""

import asyncio
import subprocess
import time
import json
import sys
import argparse
try:
    import serial  # type: ignore
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False

try:
    import requests  # type: ignore
    REQUESTS_AVAILABLE = True
except ImportError:
    requests = None
    REQUESTS_AVAILABLE = False
from datetime import datetime
from pathlib import Path
from rich.console import Console
from rich.tree import Tree
from rich.live import Live
from rich.panel import Panel
import signal

try:
    from bleak import BleakScanner, BleakClient
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False

class SensorDataRetriever:
    def __init__(self, args):
        self.args = args
        self.console = Console()
        self.running = True
        self.data_log = []
        self.connected_device = None
        self.live_data = {}

        # Default addresses/ports
        self.default_serial_port = "/dev/ttyUSB0"
        self.default_ap_ip = "192.168.4.1"

        # ESP32 BLE configuration
        self.ble_service_uuid = "12345678-1234-5678-9abc-def123456789"
        self.ble_characteristic_uuid = "87654321-4321-8765-cba9-9876543210fe"
        self.target_device_name = "SensorShield_ESP32"

        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        self.running = False
        self.console.print("\n🛑 Stopping data retrieval...")
        sys.exit(0)

    def create_status_tree(self, mode):
        """Create Rich tree for status display"""
        tree = Tree(f"📊 Sensor Data Retrieval - {mode.upper()} Mode")

        # System Info
        system_branch = tree.add("🖥️ System Status")
        if mode in ['serial', 'flash']:
            system_branch.add("✅ Serial tools available" if self.check_serial_tools() else "❌ Serial tools not available")
        if mode in ['ble-live', 'ble-dump']:
            system_branch.add("✅ BLE tools available" if BLEAK_AVAILABLE else "❌ BLE tools not available")
        if mode == 'access-point':
            system_branch.add("✅ Network tools available" if self.check_network_tools() else "❌ Network tools not available")

        # Connection Status
        if mode == 'ble-live' and self.connected_device:
            device_branch = tree.add(f"📱 Connected: {self.connected_device}")
            if self.live_data:
                for key, value in self.live_data.items():
                    if isinstance(value, (int, float)):
                        device_branch.add(f"📊 {key}: {value}")
                    else:
                        device_branch.add(f"📊 {key}: {str(value)[:50]}")

        # Data Status
        if self.data_log:
            log_branch = tree.add(f"📝 Data Retrieved ({len(self.data_log)} entries)")
            recent_entries = self.data_log[-3:] if len(self.data_log) > 3 else self.data_log
            for entry in recent_entries:
                timestamp = entry.get('timestamp', 'Unknown')
                log_branch.add(f"⏰ {timestamp}")

        return tree

    def check_serial_tools(self):
        """Check if serial tools are available"""
        return SERIAL_AVAILABLE

    def check_network_tools(self):
        """Check if network tools are available"""
        return REQUESTS_AVAILABLE

    async def mode_flash(self):
        """Flash/LittleFS data retrieval mode"""
        if not SERIAL_AVAILABLE:
            self.console.print("❌ Serial tools not available")
            return False

        self.console.print("💾 [bold blue]Flash/LittleFS Data Retrieval Mode[/bold blue]")

        port = self.args.addr if self.args.addr else self.default_serial_port

        # First try to connect and send command to dump all flash data
        try:
            ser = serial.Serial(port, 115200, timeout=self.args.timeout)
            self.console.print(f"🔌 Connected to {port}")

            # Send command to request full data dump
            ser.write(b"DUMP_ALL\n")
            time.sleep(1)

            self.console.print("📤 Sent DUMP_ALL command, waiting for response...")

            start_time = time.time()
            json_buffer = ""
            dump_started = False
            dump_ended = False

            while time.time() - start_time < self.args.timeout and not dump_ended:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    json_buffer += data

                    # Look for complete lines
                    lines = json_buffer.split('\n')
                    for line in lines[:-1]:  # Process all complete lines
                        line = line.strip()

                        # Handle protocol markers
                        if line == "DUMP_START":
                            dump_started = True
                            self.console.print("📡 Receiving data dump...")
                            continue
                        elif line == "DUMP_END":
                            dump_ended = True
                            self.console.print("✅ Data dump completed")
                            break

                        # Process JSON data between markers
                        if dump_started and line.startswith('{') and line.endswith('}'):
                            try:
                                entry = json.loads(line)
                                self.data_log.append(entry)
                                self.console.print(f"📊 [dim]Entry {len(self.data_log)}: {entry.get('timestamp', 'Unknown')}[/dim]")
                            except json.JSONDecodeError as e:
                                self.console.print(f"⚠️ JSON decode error: {e}")

                    json_buffer = lines[-1]  # Keep the incomplete line

                time.sleep(0.1)

            ser.close()

            if not dump_started:
                self.console.print("⚠️ No DUMP_START received - device may not support DUMP_ALL command")
            elif len(self.data_log) == 0:
                self.console.print("ℹ️ No stored data found on device")
            else:
                self.console.print(f"✅ Retrieved {len(self.data_log)} entries from flash storage")

        except Exception as e:
            self.console.print(f"❌ Flash mode failed: {e}")
            # Fallback to BLE dump if available
            if BLEAK_AVAILABLE:
                self.console.print("🔄 Falling back to BLE dump mode...")
                return await self.mode_ble_dump()
            return False

        return len(self.data_log) > 0

    async def mode_serial(self):
        """Serial monitoring mode"""
        if not SERIAL_AVAILABLE:
            self.console.print("❌ Serial tools not available")
            return False

        self.console.print("🔗 [bold blue]Serial Monitoring Mode[/bold blue]")

        port = self.args.addr if self.args.addr else self.default_serial_port

        try:
            ser = serial.Serial(port, 115200, timeout=1)
            self.console.print(f"🔌 Connected to {port}")
            self.console.print(f"📡 [bold cyan]Monitoring real-time data for {self.args.timeout}s (Press Ctrl+C to stop)...[/bold cyan]")

            start_time = time.time()
            with Live(self.create_status_tree("serial"), refresh_per_second=2) as live:
                while self.running and (time.time() - start_time < self.args.timeout):
                    if ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                                # Parse JSON data if present
                                if line.startswith('Data logged:'):
                                    json_part = line.split('Data logged:', 1)[1].strip()
                                    try:
                                        data = json.loads(json_part)
                                        data['_timestamp'] = timestamp
                                        self.data_log.append(data)
                                        self.console.print(f"[green][{timestamp}] Data: {json_part}[/green]")
                                    except json.JSONDecodeError:
                                        pass
                                else:
                                    # Print all other serial output with Rich Tree structure
                                    if any(keyword in line for keyword in ['SENSOR', 'Temperature', 'BLE', 'Storage']):
                                        tree = Tree(f"📊 [{timestamp}] ESP32 Output")
                                        tree.add(line)
                                        self.console.print(tree)
                                    else:
                                        self.console.print(f"[dim][{timestamp}] {line}[/dim]")

                        except Exception as e:
                            self.console.print(f"⚠️ Decode error: {e}")

                    live.update(self.create_status_tree("serial"))
                    await asyncio.sleep(0.1)

            if not self.running:
                self.console.print("🛑 Stopped by user")
            else:
                self.console.print(f"⏰ Timeout reached ({self.args.timeout}s)")

            ser.close()

        except Exception as e:
            self.console.print(f"❌ Serial mode failed: {e}")
            return False

        return True

    async def mode_access_point(self):
        """WiFi Access Point mode"""
        if not REQUESTS_AVAILABLE:
            self.console.print("❌ Network tools not available")
            return False

        self.console.print("📶 [bold blue]WiFi Access Point Mode[/bold blue]")

        ip = self.args.addr if self.args.addr else self.default_ap_ip

        try:
            # Test connection to AP
            self.console.print(f"🔗 Connecting to http://{ip}/data...")

            response = requests.get(f"http://{ip}/data", timeout=self.args.timeout)

            if response.status_code == 200:
                data = response.json()
                self.data_log.extend(data)
                self.console.print(f"✅ Retrieved {len(data)} entries from access point")

                # Also get device status
                try:
                    status_response = requests.get(f"http://{ip}/status", timeout=10)
                    if status_response.status_code == 200:
                        status = status_response.json()

                        # Display device status using Rich
                        status_tree = Tree("📊 Device Status")
                        status_tree.add(f"Boot count: {status.get('boot_count', 'Unknown')}")
                        status_tree.add(f"Uptime: {status.get('uptime_seconds', 'Unknown')} seconds")
                        status_tree.add(f"Free memory: {status.get('free_memory', 'Unknown')} bytes")
                        status_tree.add(f"SPIFFS used: {status.get('spiffs_used', 'Unknown')}/{status.get('spiffs_total', 'Unknown')} bytes")
                        status_tree.add(f"BLE connected: {status.get('ble_connected', 'Unknown')}")

                        self.console.print(status_tree)

                except Exception as e:
                    self.console.print(f"⚠️ Could not get device status: {e}")

                return True
            else:
                self.console.print(f"❌ HTTP error: {response.status_code}")
                return False

        except Exception as e:
            self.console.print(f"❌ Access point mode failed: {e}")
            return False

    async def mode_ble_live(self):
        """BLE live streaming mode"""
        if not BLEAK_AVAILABLE:
            self.console.print("❌ BLE mode requires python-bleak library")
            return False

        self.console.print("🔵 [bold blue]BLE Live Streaming Mode[/bold blue]")

        # Scan for target device
        device = await self.find_target_device()
        if not device:
            return False

        try:
            async with BleakClient(device.address) as client:
                self.connected_device = f"{device.name} ({device.address})"
                self.console.print(f"✅ Connected to {self.connected_device}")

                # Set up notification handler
                def notification_handler(sender, data):
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    try:
                        decoded = data.decode('utf-8')
                        json_data = json.loads(decoded)
                        json_data['_timestamp'] = timestamp
                        self.live_data.update(json_data)
                        self.data_log.append(json_data)

                        # Use Rich Tree for structured display
                        tree = Tree(f"🔔 [{timestamp}] Live BLE Data")
                        for key, value in json_data.items():
                            if key != '_timestamp':
                                tree.add(f"{key}: {value}")
                        self.console.print(tree)

                    except Exception as e:
                        self.console.print(f"⚠️ Data decode error: {e}")

                # Start notifications
                await client.start_notify(self.ble_characteristic_uuid, notification_handler)
                self.console.print("🔔 Started BLE notifications")

                # Monitor live data with timeout
                self.console.print(f"📊 [bold cyan]Monitoring live BLE data for {self.args.timeout}s (Press Ctrl+C to stop)...[/bold cyan]")

                start_time = time.time()
                with Live(self.create_status_tree("ble-live"), refresh_per_second=2) as live:
                    while self.running and (time.time() - start_time < self.args.timeout):
                        live.update(self.create_status_tree("ble-live"))
                        await asyncio.sleep(1)

                if not self.running:
                    self.console.print("🛑 Stopped by user")
                else:
                    self.console.print(f"⏰ Timeout reached ({self.args.timeout}s)")

                return True

        except Exception as e:
            self.console.print(f"❌ BLE live mode failed: {e}")
            return False

    async def mode_ble_dump(self):
        """BLE dump mode"""
        if not BLEAK_AVAILABLE:
            self.console.print("❌ BLE mode requires python-bleak library")
            return False

        self.console.print("🔵 [bold blue]BLE Dump Mode[/bold blue]")

        # Find target device
        device = await self.find_target_device()
        if not device:
            return False

        try:
            async with BleakClient(device.address) as client:
                self.connected_device = f"{device.name} ({device.address})"
                self.console.print(f"✅ Connected to {self.connected_device}")

                # Send DUMP_ALL command via BLE
                dump_command = "DUMP_ALL"
                await client.write_gatt_char(self.ble_characteristic_uuid, dump_command.encode())
                self.console.print("📤 Sent DUMP_ALL command via BLE")

                # Set up notification handler for dump response
                dump_started = False
                dump_ended = False

                def dump_handler(sender, data):
                    nonlocal dump_started, dump_ended
                    try:
                        decoded = data.decode('utf-8').strip()

                        if decoded == "DUMP_START":
                            dump_started = True
                            self.console.print("📡 Receiving BLE data dump...")
                        elif decoded == "DUMP_END":
                            dump_ended = True
                            self.console.print("✅ BLE data dump completed")
                        elif dump_started and decoded.startswith('{'):
                            try:
                                entry = json.loads(decoded)
                                entry['_timestamp'] = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                self.data_log.append(entry)
                                self.console.print(f"📊 [dim]Entry {len(self.data_log)}: {entry.get('timestamp', 'Unknown')}[/dim]")
                            except json.JSONDecodeError:
                                pass
                    except Exception as e:
                        self.console.print(f"⚠️ BLE data decode error: {e}")

                # Start notifications for dump
                await client.start_notify(self.ble_characteristic_uuid, dump_handler)

                # Wait for dump completion with timeout
                start_time = time.time()
                while not dump_ended and (time.time() - start_time < self.args.timeout):
                    await asyncio.sleep(0.5)

                if not dump_started:
                    self.console.print("⚠️ No DUMP_START received - device may not support BLE DUMP_ALL command")
                    self.console.print("🔄 Falling back to BLE live mode...")
                    return await self.mode_ble_live()
                elif not dump_ended:
                    self.console.print(f"⏰ Dump timeout reached ({self.args.timeout}s)")

                return len(self.data_log) > 0

        except Exception as e:
            self.console.print(f"❌ BLE dump mode failed: {e}")
            self.console.print("🔄 Falling back to BLE live mode...")
            return await self.mode_ble_live()

    async def find_target_device(self):
        """Find the target BLE device"""
        self.console.print("🔍 Scanning for BLE devices...")

        devices = await BleakScanner.discover(timeout=10)

        for device in devices:
            if device.name and self.target_device_name.lower() in device.name.lower():
                self.console.print(f"✅ Found target device: {device.name} ({device.address})")
                return device

        self.console.print("❌ Target device not found")
        return None

    def save_output(self):
        """Save collected data to output file"""
        if not self.data_log:
            self.console.print("⚠️ No data to save")
            return

        output_file = self.args.output if self.args.output else f"sensor_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

        try:
            with open(output_file, 'w') as f:
                json.dump(self.data_log, f, indent=2)

            self.console.print(f"💾 Saved {len(self.data_log)} entries to {output_file}")

            # Generate summary using Rich Tree
            summary_tree = Tree("📊 Data Collection Summary")
            summary_tree.add(f"Total entries: {len(self.data_log)}")
            summary_tree.add(f"Output file: {output_file}")
            summary_tree.add(f"File size: {Path(output_file).stat().st_size} bytes")

            if self.data_log:
                first_entry = self.data_log[0]
                last_entry = self.data_log[-1]
                summary_tree.add(f"Time span: {first_entry.get('_timestamp', 'Unknown')} to {last_entry.get('_timestamp', 'Unknown')}")

                # Show data fields
                fields_branch = summary_tree.add("Data fields:")
                for key in first_entry.keys():
                    if key != '_timestamp':
                        fields_branch.add(key)

            self.console.print(summary_tree)

        except Exception as e:
            self.console.print(f"❌ Failed to save data: {e}")

async def main():
    parser = argparse.ArgumentParser(
        description="ESP32 Sensor Shield Data Retrieval Tool",
        epilog="""
Examples:
  %(prog)s -f                          # Pull full LittleFS data
  %(prog)s -s                          # Monitor serial stream
  %(prog)s -s /dev/ttyACM0             # Monitor specific serial port
  %(prog)s -ap                         # Download from WiFi AP
  %(prog)s -bl                         # BLE live streaming
  %(prog)s -bd                         # BLE dump mode
  %(prog)s -ap --addr 192.168.1.100    # Custom AP address
  %(prog)s -bl -o live_data.json       # Save BLE data to file
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # Mode selection (mutually exclusive)
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('-f', '--flash', action='store_true',
                          help='Pull full JSON log from LittleFS on 16MB SPI NOR chip')
    mode_group.add_argument('-s', '--serial', nargs='?', const=True,
                          help='Mirror real-time Serial.println() stream (default: /dev/ttyUSB0)')
    mode_group.add_argument('-ap', '--access-point', action='store_true',
                          help='Download /data from WiFi AP (192.168.4.1)')
    mode_group.add_argument('-bl', '--ble-live', action='store_true',
                          help='Subscribe to BLE LogData notifications for live readings')
    mode_group.add_argument('-bd', '--ble-dump', action='store_true',
                          help='Send DUMP over BLE LogCtrl, save full history until EOF')

    # Optional arguments
    parser.add_argument('--addr',
                       help='Override IP (AP), BLE MAC, or Serial port (e.g., --addr AA:BB:CC:DD:EE:FF)')
    parser.add_argument('-o', '--output',
                       help='Write output to file instead of stdout')
    parser.add_argument('--timeout', type=int, default=30,
                       help='Network/BLE timeout in seconds (default: 30)')

    args = parser.parse_args()

    # Handle serial port specification
    if args.serial and isinstance(args.serial, str):
        args.addr = args.serial

    # Create retriever instance
    retriever = SensorDataRetriever(args)

    # Header
    retriever.console.print(Panel.fit(
        "📊 ESP32 Sensor Shield Data Retrieval Tool",
        style="bold blue"
    ))

    success = False

    try:
        # Route to appropriate mode
        if args.flash:
            success = await retriever.mode_flash()
        elif args.serial:
            success = await retriever.mode_serial()
        elif args.access_point:
            success = await retriever.mode_access_point()
        elif args.ble_live:
            success = await retriever.mode_ble_live()
        elif args.ble_dump:
            success = await retriever.mode_ble_dump()

        # Save output if requested and data was collected
        if success and (args.output or retriever.data_log):
            retriever.save_output()

        if success:
            retriever.console.print("\n🎉 [bold green]Data retrieval completed successfully![/bold green]")
            return 0
        else:
            retriever.console.print("\n⚠️ [bold yellow]Data retrieval completed with issues[/bold yellow]")
            return 1

    except KeyboardInterrupt:
        retriever.console.print("\n🛑 Data retrieval interrupted by user")
        if retriever.data_log and args.output:
            retriever.save_output()
        return 0
    except Exception as e:
        retriever.console.print(f"\n❌ [bold red]Data retrieval failed: {e}[/bold red]")
        return 1

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
