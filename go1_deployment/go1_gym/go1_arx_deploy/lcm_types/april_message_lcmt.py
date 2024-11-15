"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class april_message_lcmt(object):
    __slots__ = ["ee_pose"]

    __typenames__ = ["float"]

    __dimensions__ = [[6]]

    def __init__(self):
        self.ee_pose = [ 0.0 for dim0 in range(6) ]

    def encode(self):
        buf = BytesIO()
        buf.write(april_message_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>6f', *self.ee_pose[:6]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != april_message_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return april_message_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = april_message_lcmt()
        self.ee_pose = struct.unpack('>6f', buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if april_message_lcmt in parents: return 0
        tmphash = (0x372e753cc0aa2998) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if april_message_lcmt._packed_fingerprint is None:
            april_message_lcmt._packed_fingerprint = struct.pack(">Q", april_message_lcmt._get_hash_recursive([]))
        return april_message_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

