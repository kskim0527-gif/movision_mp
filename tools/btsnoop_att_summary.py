import argparse
import collections
import struct
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List, Iterable


BT_SNOOP_MAGIC = b"btsnoop\x00"


@dataclass
class AttEvent:
    idx: int
    direction: str  # "out" or "in" relative to phone host
    handle: Optional[int]
    opcode: int
    cid: int
    acl_handle: int
    payload_len: int
    att: bytes  # full ATT payload (reassembled)


def _be_u32(b: bytes, off: int) -> int:
    return struct.unpack_from(">I", b, off)[0]


def _be_u64(b: bytes, off: int) -> int:
    return struct.unpack_from(">Q", b, off)[0]


def _uuid_str_le(uuid_bytes: bytes) -> str:
    """
    ATT/GATT transmits UUIDs little-endian on the wire for 16-bit and 128-bit UUIDs.
    """
    if len(uuid_bytes) == 2:
        u16 = struct.unpack_from("<H", uuid_bytes, 0)[0]
        return f"0x{u16:04X}"
    if len(uuid_bytes) == 16:
        b = bytes(uuid_bytes[::-1])
        return f"{b[0:4].hex()}-{b[4:6].hex()}-{b[6:8].hex()}-{b[8:10].hex()}-{b[10:16].hex()}"
    return uuid_bytes.hex()


def _uuid_short(uuid_bytes: bytes) -> str:
    s = _uuid_str_le(uuid_bytes)
    # collapse Bluetooth Base UUIDs for readability
    if len(uuid_bytes) == 16 and s.endswith("-0000-1000-8000-00805f9b34fb"):
        # 0000xxxx-0000-1000-8000-00805f9b34fb -> 0xXXXX
        try:
            u16 = int(s[4:8], 16)
            return f"0x{u16:04X}"
        except Exception:
            return s
    return s


def _att_opcode_name(op: int) -> str:
    op_name = {
        0x01: "ErrRsp",
        0x02: "ExchMtuReq",
        0x03: "ExchMtuRsp",
        0x04: "FindInfoReq",
        0x05: "FindInfoRsp",
        0x06: "FindByTypeValReq",
        0x07: "FindByTypeValRsp",
        0x08: "ReadByTypeReq",
        0x09: "ReadByTypeRsp",
        0x0A: "ReadReq",
        0x0B: "ReadRsp",
        0x0C: "ReadBlobReq",
        0x0D: "ReadBlobRsp",
        0x10: "ReadByGrpTypeReq",
        0x11: "ReadByGrpTypeRsp",
        0x12: "WriteReq",
        0x13: "WriteRsp",
        0x1B: "Notif",
        0x1D: "Ind",
        0x1E: "Conf",
        0x52: "WriteCmd",
    }
    return op_name.get(op, "?")


def _decode_att_brief(e: AttEvent) -> str:
    """
    Best-effort decode of common discovery primitives, so you don't need Wireshark.
    Returns a short human-readable string (may be empty).
    """
    a = e.att
    if not a:
        return ""
    op = a[0]

    # Requests
    if op == 0x10 and len(a) >= 7:  # Read By Group Type Req
        sh, eh = struct.unpack_from("<HH", a, 1)
        group_type = a[5:]
        return f"start=0x{sh:04X} end=0x{eh:04X} group_type={_uuid_short(group_type)}"
    if op == 0x08 and len(a) in (7, 21):  # Read By Type Req (2B or 16B UUID)
        sh, eh = struct.unpack_from("<HH", a, 1)
        attr_type = a[5:]
        return f"start=0x{sh:04X} end=0x{eh:04X} type={_uuid_short(attr_type)}"
    if op == 0x04 and len(a) >= 5:  # Find Information Req
        sh, eh = struct.unpack_from("<HH", a, 1)
        return f"start=0x{sh:04X} end=0x{eh:04X}"
    if op in (0x0A, 0x0C) and len(a) >= 3:  # Read Req / Read Blob Req
        h = struct.unpack_from("<H", a, 1)[0]
        if op == 0x0C and len(a) >= 5:
            off = struct.unpack_from("<H", a, 3)[0]
            return f"handle=0x{h:04X} off={off}"
        return f"handle=0x{h:04X}"

    # Responses (length varies; we only summarize)
    if op == 0x11 and len(a) >= 2:  # Read By Group Type Rsp
        ent_len = a[1]
        n = 0
        if ent_len >= 4:
            n = (len(a) - 2) // ent_len
        return f"entry_len={ent_len} entries={n}"
    if op == 0x09 and len(a) >= 2:  # Read By Type Rsp
        ent_len = a[1]
        n = 0
        if ent_len >= 2:
            n = (len(a) - 2) // ent_len
        return f"entry_len={ent_len} entries={n}"
    if op == 0x05 and len(a) >= 2:  # Find Info Rsp
        fmt = a[1]
        ent_len = 4 if fmt == 0x01 else 18 if fmt == 0x02 else 0
        n = 0
        if ent_len:
            n = (len(a) - 2) // ent_len
        return f"fmt=0x{fmt:02X} entries={n}"

    return ""


@dataclass
class DiscoveryDB:
    services: List[Tuple[int, int, str]]
    characteristics: List[Tuple[int, int, int, str]]  # (decl_handle, props, value_handle, uuid)
    descriptors: Dict[int, str]  # handle -> uuid


def _extract_discovery(events: List[AttEvent]) -> DiscoveryDB:
    """
    Extract best-effort service/characteristic/descriptor mapping from ATT discovery traffic.
    This is heuristic (it does not track request/response pairing), but works well in practice.
    """
    services: List[Tuple[int, int, str]] = []
    characteristics: List[Tuple[int, int, int, str]] = []
    descriptors: Dict[int, str] = {}

    for e in events:
        a = e.att
        if not a:
            continue
        op = a[0]

        # Read By Group Type Response: often used for Primary Service discovery (group type 0x2800)
        if op == 0x11 and len(a) >= 2:
            ent_len = a[1]
            if ent_len >= 6 and (len(a) - 2) % ent_len == 0:
                # entry: start(2) end(2) value(ent_len-4) where value is service UUID for 0x2800
                for off in range(2, len(a), ent_len):
                    sh, eh = struct.unpack_from("<HH", a, off)
                    val = a[off + 4 : off + ent_len]
                    if len(val) in (2, 16):
                        services.append((sh, eh, _uuid_short(val)))

        # Read By Type Response: often used with type 0x2803 (Characteristic Declaration)
        if op == 0x09 and len(a) >= 2:
            ent_len = a[1]
            if ent_len in (7, 21) and (len(a) - 2) % ent_len == 0:
                # entry: decl_handle(2) + props(1) + value_handle(2) + uuid(2 or 16)
                for off in range(2, len(a), ent_len):
                    decl_handle = struct.unpack_from("<H", a, off)[0]
                    props = a[off + 2]
                    value_handle = struct.unpack_from("<H", a, off + 3)[0]
                    uuid_b = a[off + 5 : off + ent_len]
                    if len(uuid_b) in (2, 16):
                        characteristics.append((decl_handle, props, value_handle, _uuid_short(uuid_b)))

        # Find Information Response: often used for descriptors (incl. CCCD 0x2902, Report Ref 0x2908)
        if op == 0x05 and len(a) >= 2:
            fmt = a[1]
            if fmt == 0x01:  # 16-bit UUID
                ent_len = 4  # handle(2) + uuid(2)
                if (len(a) - 2) % ent_len == 0:
                    for off in range(2, len(a), ent_len):
                        h = struct.unpack_from("<H", a, off)[0]
                        u = a[off + 2 : off + 4]
                        descriptors[h] = _uuid_short(u)
            elif fmt == 0x02:  # 128-bit UUID
                ent_len = 18  # handle(2) + uuid(16)
                if (len(a) - 2) % ent_len == 0:
                    for off in range(2, len(a), ent_len):
                        h = struct.unpack_from("<H", a, off)[0]
                        u = a[off + 2 : off + 18]
                        descriptors[h] = _uuid_short(u)

    # De-duplicate (keep order of first appearance)
    seen_s = set()
    services2: List[Tuple[int, int, str]] = []
    for sh, eh, u in services:
        key = (sh, eh, u)
        if key in seen_s:
            continue
        seen_s.add(key)
        services2.append((sh, eh, u))

    seen_c = set()
    chars2: List[Tuple[int, int, int, str]] = []
    for dh, p, vh, u in characteristics:
        key = (dh, vh, u, p)
        if key in seen_c:
            continue
        seen_c.add(key)
        chars2.append((dh, p, vh, u))

    return DiscoveryDB(services=services2, characteristics=chars2, descriptors=descriptors)


def parse_btsnoop(path: str) -> List[bytes]:
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 16 or data[:8] != BT_SNOOP_MAGIC:
        raise SystemExit("Not a btsnoop file (missing magic).")
    # version = _be_u32(data, 8)
    # datalink = _be_u32(data, 12)
    off = 16
    packets: List[bytes] = []
    while off + 24 <= len(data):
        orig_len = _be_u32(data, off + 0)
        incl_len = _be_u32(data, off + 4)
        _flags = _be_u32(data, off + 8)
        # drops = _be_u32(data, off + 12)
        # ts = _be_u64(data, off + 16)
        off += 24
        if off + incl_len > len(data):
            break
        pkt = data[off : off + incl_len]
        off += incl_len
        # Some files may have padding; be conservative
        if orig_len == 0 or incl_len == 0:
            continue
        packets.append(struct.pack(">I", _flags) + pkt)  # prefix flags for direction
    return packets


def direction_from_flags(flags: int) -> str:
    # btsnoop flags: bit0 indicates direction (0=sent, 1=received) for most stacks
    # We'll label relative to PHONE: out=phone->air, in=air->phone
    return "in" if (flags & 0x1) else "out"


def parse_att_from_btsnoop_packets(packets_with_flags: List[bytes]) -> List[AttEvent]:
    events: List[AttEvent] = []

    # Reassembly state per ACL handle (LE-U / BR/EDR doesn't matter; btsnoop is HCI level)
    # We only care about CID 0x0004 (ATT), but need L2CAP reassembly first.
    InProg = Tuple[int, int, bytearray]  # (cid, expected_len, buf)
    inprog: Dict[int, InProg] = {}

    idx = 0
    for rec in packets_with_flags:
        if len(rec) < 5:
            continue
        flags = struct.unpack_from(">I", rec, 0)[0]
        pkt = rec[4:]
        if not pkt:
            continue
        ptype = pkt[0]
        if ptype != 0x02:  # HCI ACL Data Packet
            continue
        if len(pkt) < 1 + 4:
            continue

        # ACL header
        handle_pb_bc = struct.unpack_from("<H", pkt, 1)[0]
        acl_handle = handle_pb_bc & 0x0FFF
        pb = (handle_pb_bc >> 12) & 0x3
        # bc = (handle_pb_bc >> 14) & 0x3
        data_total_len = struct.unpack_from("<H", pkt, 3)[0]
        payload = pkt[5:]
        if len(payload) < data_total_len:
            # truncated in file; skip
            continue
        payload = payload[:data_total_len]

        # PB flags:
        # Common interpretations:
        # 0b10 = first automatically-flushable packet of a higher layer PDU
        # 0b01 = continuing fragment
        # Some captures (esp. LE) also show small PDUs with pb==0x0 or pb==0x3.
        if pb in (0x0, 0x2, 0x3):
            if len(payload) < 4:
                continue
            l2_len = struct.unpack_from("<H", payload, 0)[0]
            cid = struct.unpack_from("<H", payload, 2)[0]
            l2_payload = payload[4:]
            buf = bytearray(l2_payload)
            inprog[acl_handle] = (cid, l2_len, buf)
        elif pb == 0x1:
            if acl_handle not in inprog:
                continue
            cid, exp_len, buf = inprog[acl_handle]
            buf.extend(payload)
            inprog[acl_handle] = (cid, exp_len, buf)
        else:
            # Unknown / unhandled
            continue

        cid, exp_len, buf = inprog.get(acl_handle, (0, 0, bytearray()))
        if exp_len == 0:
            continue
        if len(buf) < exp_len:
            continue

        full = bytes(buf[:exp_len])
        # remove consumed pdu
        del inprog[acl_handle]

        if cid != 0x0004 or len(full) < 1:
            continue

        opcode = full[0]
        handle = None
        if opcode in (0x08, 0x0A):  # Read By Type Req / Read Req
            if len(full) >= 3:
                handle = struct.unpack_from("<H", full, 1)[0]
        elif opcode in (0x12, 0x52):  # Write Request / Write Command
            if len(full) >= 3:
                handle = struct.unpack_from("<H", full, 1)[0]
        elif opcode in (0x1B, 0x1D):  # Notification / Indication
            if len(full) >= 3:
                handle = struct.unpack_from("<H", full, 1)[0]
        # 0x1E Confirmation has no handle

        events.append(
            AttEvent(
                idx=idx,
                direction=direction_from_flags(flags),
                handle=handle,
                opcode=opcode,
                cid=cid,
                acl_handle=acl_handle,
                payload_len=len(full),
                att=full,
            )
        )
        idx += 1

    return events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("btsnoop", help="Path to btsnoop_hci.log")
    ap.add_argument("--limit", type=int, default=200, help="Max number of events to print")
    ap.add_argument(
        "--show-discovery",
        action="store_true",
        help="Print decoded service/characteristic/descriptor map inferred from discovery traffic.",
    )
    ap.add_argument(
        "--tail",
        type=int,
        default=0,
        help="Only keep the last N ATT events (0 = keep all). Useful when btsnoop contains multiple sessions.",
    )
    ap.add_argument(
        "--min-handle",
        default="",
        help="Filter: only include events with handle >= this value (hex like 0x0020 or decimal).",
    )
    ap.add_argument(
        "--max-handle",
        default="",
        help="Filter: only include events with handle <= this value (hex like 0x0048 or decimal).",
    )
    ap.add_argument(
        "--acl",
        action="append",
        default=[],
        help="Filter: only include events for specific ACL handle(s) (hex like 0x001E or decimal). Can be provided multiple times.",
    )
    ap.add_argument(
        "--watch",
        action="append",
        default=[],
        help="Watch specific ATT handles (hex like 0x002D or decimal). Can be provided multiple times.",
    )
    args = ap.parse_args()

    packets = parse_btsnoop(args.btsnoop)
    events = parse_att_from_btsnoop_packets(packets)

    def parse_handle_list(vals: Iterable[str]) -> List[int]:
        out: List[int] = []
        for v in vals:
            v = v.strip().lower()
            if not v:
                continue
            if v.startswith("0x"):
                out.append(int(v, 16))
            else:
                out.append(int(v, 10))
        return out

    def parse_int_opt(v: str) -> Optional[int]:
        if v is None:
            return None
        s = str(v).strip().lower()
        if not s:
            return None
        return int(s, 16) if s.startswith("0x") else int(s, 10)

    # Optional tail filter (helps when bugreport includes multiple sessions/devices)
    if args.tail and args.tail > 0 and len(events) > args.tail:
        events = events[-args.tail :]

    # Optional ACL-handle filter
    acl_filter = set(parse_handle_list(args.acl)) if args.acl else set()
    if acl_filter:
        events = [e for e in events if e.acl_handle in acl_filter]

    # Optional handle-range filter (note: some discovery ops don't carry a single handle; we keep those)
    min_h = parse_int_opt(args.min_handle)
    max_h = parse_int_opt(args.max_handle)
    if min_h is not None or max_h is not None:
        filtered = []
        for e in events:
            if e.handle is None:
                filtered.append(e)
                continue
            if min_h is not None and e.handle < min_h:
                continue
            if max_h is not None and e.handle > max_h:
                continue
            filtered.append(e)
        events = filtered

    by_opcode = collections.Counter(e.opcode for e in events)
    by_handle_write = collections.Counter(e.handle for e in events if e.opcode in (0x12, 0x52) and e.handle is not None)
    by_handle_read = collections.Counter(e.handle for e in events if e.opcode in (0x0A,) and e.handle is not None)
    by_handle_ind = collections.Counter(e.handle for e in events if e.opcode == 0x1D and e.handle is not None)
    by_acl = collections.Counter(e.acl_handle for e in events)

    print(f"ATT events: {len(events)}")
    if len(by_acl) > 1:
        print("ACL handle counts (multiple connections likely):")
        for ah, cnt in sorted(by_acl.items(), key=lambda x: (-x[1], x[0]))[:20]:
            print(f"  acl_handle=0x{ah:04X} ({ah}): {cnt}")
    print("Opcode counts:")
    for op, cnt in sorted(by_opcode.items(), key=lambda x: (-x[1], x[0])):
        print(f"  0x{op:02X}: {cnt}")

    def top(counter, name):
        items = [(k, v) for k, v in counter.items() if k is not None]
        items.sort(key=lambda x: (-x[1], x[0]))
        print(f"{name} (top 20):")
        for k, v in items[:20]:
            print(f"  handle 0x{k:04X} ({k}): {v}")

    top(by_handle_write, "WRITE handles (0x12/0x52)")
    top(by_handle_read, "READ handles (0x0A)")
    top(by_handle_ind, "INDICATION handles (0x1D)")

    # Show likely CCCD writes (2-byte payloads) and any short writes
    cccd_like = []
    short_writes = []
    for e in events:
        if e.opcode not in (0x12, 0x52) or e.handle is None:
            continue
        val = e.att[3:]
        if len(val) == 2:
            cccd_like.append((e.handle, struct.unpack_from("<H", val, 0)[0]))
        if len(val) <= 8:
            short_writes.append((e.idx, e.direction, e.handle, e.opcode, val))

    if cccd_like:
        cnt = collections.Counter(cccd_like)
        print("\nCCCD-like writes (handle,value16) top:")
        for (h, v), c in sorted(cnt.items(), key=lambda x: (-x[1], x[0][0], x[0][1]))[:30]:
            print(f"  handle 0x{h:04X} val=0x{v:04X} ({v}) : {c}")
    else:
        print("\nCCCD-like writes: none detected (no 2-byte writes found).")

    if short_writes:
        print("\nShort writes (<=8 bytes), first 50:")
        for idx0, d, h, op, val in short_writes[:50]:
            print(f"  ev={idx0:05d} {d:3s} op=0x{op:02X} handle=0x{h:04X} val={val.hex(':')}")
    else:
        print("\nShort writes: none.")

    watch = parse_handle_list(args.watch)
    if watch:
        watch_set = set(watch)
        print("\nWatched handles:")
        for h in sorted(watch_set):
            print(f"  0x{h:04X} ({h})")

        print("\nEvents for watched handles (first 400):")
        shown = 0
        for e in events:
            if e.handle is None or e.handle not in watch_set:
                continue
            val_hex = ""
            if e.opcode in (0x12, 0x52, 0x1B, 0x1D) and len(e.att) > 3:
                val_hex = " val=" + e.att[3: min(len(e.att), 3 + 20)].hex(":")
                if len(e.att) > 3 + 20:
                    val_hex += "..."
            brief = _decode_att_brief(e)
            if brief:
                brief = " " + brief
            print(
                f"  ev={e.idx:05d} {e.direction:3s} op=0x{e.opcode:02X}({_att_opcode_name(e.opcode)}) "
                f"handle=0x{e.handle:04X} len={e.payload_len}{val_hex}{brief}"
            )
            shown += 1
            if shown >= 400:
                print("  ... truncated (use Wireshark for full view, or increase script limits if needed).")
                break

    if args.show_discovery:
        db = _extract_discovery(events)
        print("\nDiscovery summary (best-effort):")
        if db.services:
            print("  Services (start..end uuid):")
            for sh, eh, u in db.services:
                print(f"    0x{sh:04X}..0x{eh:04X} {u}")
        else:
            print("  Services: (none decoded)")
        if db.characteristics:
            print("  Characteristics (decl props value uuid):")
            for dh, p, vh, u in db.characteristics:
                print(f"    decl=0x{dh:04X} props=0x{p:02X} value=0x{vh:04X} uuid={u}")
        else:
            print("  Characteristics: (none decoded)")
        if db.descriptors:
            print("  Descriptors (handle uuid):")
            for h in sorted(db.descriptors.keys()):
                print(f"    0x{h:04X} {db.descriptors[h]}")
        else:
            print("  Descriptors: (none decoded)")

    print("\nFirst events:")
    for e in events[: args.limit]:
        h = f"0x{e.handle:04X}" if e.handle is not None else "-"
        brief = _decode_att_brief(e)
        if brief:
            brief = " " + brief
        print(
            f"{e.idx:05d} {e.direction:3s} op=0x{e.opcode:02X}({_att_opcode_name(e.opcode)}) "
            f"handle={h} len={e.payload_len}{brief}"
        )


if __name__ == "__main__":
    main()


