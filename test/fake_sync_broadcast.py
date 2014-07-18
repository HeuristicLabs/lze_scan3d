import socket
import sys
import time
import struct
import numpy as np

dt = 0.1

#UDP_HOST_OUT = 'localhost'
UDP_HOST_OUT = '127.0.0.1'
#UDP_HOST_OUT = '172.25.89.186'
UDP_PORT_OUT = 6000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

out_addr = (UDP_HOST_OUT,UDP_PORT_OUT)



while True:

    data = struct.pack('!hhhhhh', np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand())

    sent = sock.sendto(data, out_addr)
    print >>sys.stderr, 'sent %s bytes to %s' % (sent, out_addr)

    time.sleep(dt)
