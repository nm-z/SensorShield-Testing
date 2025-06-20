class Serial:
    def __init__(self, *_, **__):
        self._data = b"DUMP_START\n{\"temp\":25,\"humidity\":60}\nDUMP_END\n"
        self._index = 0

    @property
    def in_waiting(self):
        return len(self._data) - self._index

    def write(self, _):
        pass

    def read(self, size=1):
        start = self._index
        end = min(start + size, len(self._data))
        self._index = end
        return self._data[start:end]

    def readline(self):
        newline = self._data.find(b"\n", self._index)
        if newline == -1:
            newline = len(self._data)
        end = newline + 1
        chunk = self._data[self._index:end]
        self._index = end
        return chunk

    def close(self):
        pass

class SerialException(Exception):
    pass
