"""Build the assets folder the port reads instead of the disc.

Usage: python make_assets.py <disc.cue> <assets dir>

- the data track's files go to the assets folder; Form 2 files (.STR video, .XA audio) stay
  in raw 2352-byte sectors so they can be streamed like from the disc
- CD audio tracks go to music/trackNN.wav (44.1 kHz 16-bit stereo, exactly the CD data), from a .bin per
  track (Redump) or one .bin for the whole disc
- trailing dots are dropped from names ("DIRINFO." -> "DIRINFO"); Windows can't keep them
- the empty DD2_MUS placeholders (directory entries pointing at the audio tracks) are skipped
"""
import os
import re
import struct
import sys

RAW = 2352
DATA_OFF = 24
XA_FORM2 = 0x1000
XA_INTERLEAVED = 0x2000
PREGAP = 150  # INDEX 00 to INDEX 01 of each audio track: 2 seconds of silence in the file


def parse_cue(path):
    """[{file, number, audio, index: {n: sector in its file}}] in disc order."""
    base = os.path.dirname(path)
    tracks, current = [], None
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.match(r'\s*FILE "(.+)"', line)
        if m:
            current = os.path.join(base, m.group(1))
        m = re.match(r"\s*TRACK (\d+) (\S+)", line)
        if m:
            tracks.append({"file": current, "number": int(m.group(1)), "audio": m.group(2) == "AUDIO", "index": {}})
        m = re.match(r"\s*INDEX (\d+) (\d+):(\d+):(\d+)", line)
        if m and tracks:
            mm, ss, ff = int(m.group(2)), int(m.group(3)), int(m.group(4))
            tracks[-1]["index"][int(m.group(1))] = (mm * 60 + ss) * 75 + ff
    return tracks


def track_span(tracks, i):
    """(first, end) sectors of track i's audio (INDEX 01 on) in its file; end None = to the end of it."""
    t = tracks[i]
    first = t["index"].get(1, PREGAP)
    end = None
    if i + 1 < len(tracks) and tracks[i + 1]["file"] == t["file"]:
        nxt = tracks[i + 1]["index"]
        end = nxt.get(0, nxt.get(1))
    return first, end


def read_sector(f, lba):
    f.seek(lba * RAW)
    return f.read(RAW)


def read_data(f, lba, size):
    out = bytearray()
    for i in range((size + 2047) // 2048):
        out += read_sector(f, lba + i)[DATA_OFF:DATA_OFF + 2048]
    return bytes(out[:size])


def walk(f, lba, size, out_dir, rel, written):
    data = read_data(f, lba, size)
    pos = 0
    while pos < len(data):
        length = data[pos]
        if length == 0:
            pos = (pos // 2048 + 1) * 2048
            continue
        rec = data[pos:pos + length]
        pos += length
        ext_lba, ext_size = struct.unpack_from("<I", rec, 2)[0], struct.unpack_from("<I", rec, 10)[0]
        name_len = rec[32]
        name = rec[33:33 + name_len]
        if name in (b"\x00", b"\x01"):
            continue
        name = name.decode("ascii").split(";")[0].rstrip(".")
        su = 33 + name_len + (1 - name_len % 2)
        xa = struct.unpack_from(">H", rec, su + 4)[0] if rec[su + 6:su + 8] == b"XA" else 0
        path = os.path.join(out_dir, name)
        if rec[25] & 0x02:
            if name == "DD2_MUS":
                continue
            os.makedirs(path, exist_ok=True)
            walk(f, ext_lba, ext_size, path, rel + name + "/", written)
            continue
        with open(path, "wb") as o:
            if xa & (XA_FORM2 | XA_INTERLEAVED):
                for i in range((ext_size + 2047) // 2048):
                    o.write(read_sector(f, ext_lba + i))
            else:
                o.write(read_data(f, ext_lba, ext_size))
        written.append(rel + name)


def write_wav(src, first, end, dst):
    with open(src, "rb") as f:
        f.seek(first * RAW)
        pcm = f.read() if end is None else f.read((end - first) * RAW)
    with open(dst, "wb") as o:
        o.write(b"RIFF" + struct.pack("<I", 36 + len(pcm)) + b"WAVE")
        o.write(b"fmt " + struct.pack("<IHHIIHH", 16, 1, 2, 44100, 44100 * 4, 4, 16))
        o.write(b"data" + struct.pack("<I", len(pcm)) + pcm)
    return len(pcm) // (44100 * 4)


def main():
    cue, out_dir = sys.argv[1], sys.argv[2]
    tracks = parse_cue(cue)
    os.makedirs(os.path.join(out_dir, "music"), exist_ok=True)
    written = []
    data_file = tracks[0]["file"]
    with open(data_file, "rb") as f:
        pvd = read_sector(f, 16)[DATA_OFF:DATA_OFF + 2048]
        root = pvd[156:156 + 34]
        walk(f, struct.unpack_from("<I", root, 2)[0], struct.unpack_from("<I", root, 10)[0], out_dir, "", written)
    print("files:", " ".join(written))
    for i, t in enumerate(tracks):
        if t["audio"]:
            first, end = track_span(tracks, i)
            number = t["number"]
            seconds = write_wav(t["file"], first, end, os.path.join(out_dir, "music", f"track{number:02d}.wav"))
            print(f"music/track{number:02d}.wav  {seconds // 60}:{seconds % 60:02d}")


if __name__ == "__main__":
    main()
