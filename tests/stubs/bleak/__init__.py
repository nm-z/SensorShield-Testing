import asyncio

class _Device:
    def __init__(self, name='SensorShield_ESP32', address='00:11:22:33:44:55'):
        self.name = name
        self.address = address

class BleakScanner:
    @staticmethod
    async def discover(timeout=10):
        await asyncio.sleep(0)
        return [_Device()]

class BleakClient:
    def __init__(self, address):
        self.address = address

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        pass

    async def write_gatt_char(self, uuid, data):
        return None

    async def start_notify(self, uuid, callback):
        callback(uuid, b'DUMP_START')
        callback(uuid, b'{"temp":25,"humidity":60}')
        callback(uuid, b'DUMP_END')
