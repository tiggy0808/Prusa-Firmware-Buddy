#!/usr/bin/env python3
"""Compile a .po into a .mo, dropping msgids absent from the firmware binary."""

import argparse
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

import polib

MO_MAGIC = 0x950412de
MO_HEADER_SIZE = 28


def hashpjw(data: bytes) -> int:
    """GNU gettext string hash; must match gettext_hash_table::hash_string."""
    hval = 0
    for byte in data:
        if byte == 0:
            break
        hval = ((hval << 4) + byte) & 0xffffffff
        g = hval & 0xf0000000
        if g:
            hval ^= g >> 24
            hval ^= g
    return hval


def is_prime(n: int) -> bool:
    if n < 2:
        return False
    return all(n % d for d in range(2, int(n**0.5) + 1))


def next_prime(n: int) -> int:
    while not is_prime(n):
        n += 1
    return n


def write_mo_with_hash_table(pofile: polib.POFile, output: Path):
    """Write a GNU .mo including the hash table the firmware looks strings up by.

    polib omits the hash table, so this is the fallback for hosts without
    msgfmt (e.g. Windows). Only plain entries are supported, which is all the
    firmware catalogs use.
    """
    entries = {'': pofile.metadata_as_entry().msgstr}
    for entry in pofile:
        if entry.msgctxt or entry.msgid_plural:
            raise ValueError(
                f'Unsupported entry without msgfmt: {entry.msgid!r}')
        if entry.translated():
            entries[entry.msgid] = entry.msgstr

    items = sorted((k.encode(), v.encode()) for k, v in entries.items())
    count = len(items)
    hash_size = max(3, next_prime(count * 4 // 3))

    hash_table = [0] * hash_size
    for index, (msgid, _) in enumerate(items):
        hval = hashpjw(msgid)
        pos = hval % hash_size
        incr = 1 + hval % (hash_size - 2)
        while hash_table[pos]:
            pos = pos - (hash_size -
                         incr) if pos >= hash_size - incr else pos + incr
        hash_table[pos] = index + 1

    orig_offset = MO_HEADER_SIZE
    trans_offset = orig_offset + 8 * count
    hash_offset = trans_offset + 8 * count
    strings_offset = hash_offset + 4 * hash_size

    orig_table, trans_table, blob = [], [], bytearray()
    for msgid, _ in items:
        orig_table.append((len(msgid), strings_offset + len(blob)))
        blob += msgid + b'\0'
    for _, msgstr in items:
        trans_table.append((len(msgstr), strings_offset + len(blob)))
        blob += msgstr + b'\0'

    with open(output, 'wb') as f:
        f.write(
            struct.pack('<7I', MO_MAGIC, 0, count, orig_offset, trans_offset,
                        hash_size, hash_offset))
        for length, offset in orig_table + trans_table:
            f.write(struct.pack('<2I', length, offset))
        f.write(struct.pack(f'<{hash_size}I', *hash_table))
        f.write(blob)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path, help='input .po')
    parser.add_argument('--output',
                        required=True,
                        type=Path,
                        help='output .mo')
    parser.add_argument('--binary',
                        type=Path,
                        help='binary gating which strings are kept')
    args = parser.parse_args()

    pofile = polib.pofile(str(args.input))
    if args.binary:
        blob = args.binary.read_bytes()
        pofile[:] = [e for e in pofile if e.msgid.encode() in blob]

    if shutil.which('msgfmt') is None:
        write_mo_with_hash_table(pofile, args.output)
        return

    # polib doesn't build a hash table so we need to call msgfmt here.
    # A directory rather than a named temp file, so msgfmt can open it on Windows.
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_po = Path(tmp_dir) / args.input.name
        pofile.save(str(tmp_po))
        subprocess.run(
            ['msgfmt', str(tmp_po), '-o',
             str(args.output)], check=True)


if __name__ == '__main__':
    main()
