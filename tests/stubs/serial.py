class Serial:
    """Simple stub of ``pyserial.Serial`` supporting dump and live modes."""

    def __init__(self, *_, **__):
        self._index = 0
        self._set_live_data()

    def _set_live_data(self):
        self._data = (
            b"Booting...\n"
            b"Data logged: {\"temp\":25,\"humidity\":60}\n"
            b"Data logged: {\"temp\":26,\"humidity\":61}\n"
        )

    def _set_dump_data(self):
        self._data = b"DUMP_START\n{\"temp\":25,\"humidity\":60}\nDUMP_END\n"
        self._index = 0

    @property
    def in_waiting(self):
        return len(self._data) - self._index

    def write(self, data):
        if b"DUMP_ALL" in data:
            self._set_dump_data()

    def read(self, size=1):
        start = self._index
        end = min(start + size, len(self._data))
        chunk = self._data[start:end]
        self._index = end
        return chunk

    def readline(self):
        newline = self._data.find(b"\n", self._index)
        if newline == -1:
            newline = len(self._data)
            end = newline
        else:
            end = newline + 1
        chunk = self._data[self._index:end]
        self._index = end
        return chunk

    def close(self):
        pass

class SerialException(Exception):
    pass

