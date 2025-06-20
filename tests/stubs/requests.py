class _Response:
    def __init__(self, data, status=200):
        self._data = data
        self.status_code = status

    def json(self):
        return self._data


def get(url, timeout=0):
    if url.endswith('/data'):
        return _Response([{"temp": 25, "humidity": 60}])
    if url.endswith('/status'):
        return _Response({
            'boot_count': 1,
            'uptime_seconds': 42,
            'free_memory': 1024,
            'spiffs_used': 10,
            'spiffs_total': 20,
            'ble_connected': False,
        })
    return _Response({}, status=404)
