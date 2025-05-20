import hashlib

data = b"u566UZQnQPC8+L/hillMsA==258EAFA5-E914-47DA-95CA-C5AB0DC85B11\0\0\0"

print(hashlib.sha1(data).hexdigest())
