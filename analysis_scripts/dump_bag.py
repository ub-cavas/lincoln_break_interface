#!/usr/bin/env python3
import sys, csv, os
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)

db = [f for f in os.listdir(BAG) if f.endswith('.db3')][0]
con = sqlite3.connect(os.path.join(BAG, db))
cur = con.cursor()

topics = {}
for tid, name, typ in cur.execute("SELECT id, name, type FROM topics"):
    topics[tid] = (name, typ)

def fields(msg, prefix=''):
    out = {}
    for f in msg.get_fields_and_field_types():
        v = getattr(msg, f)
        key = prefix + f
        if hasattr(v, 'get_fields_and_field_types'):
            out.update(fields(v, key + '.'))
        elif isinstance(v, (int, float, bool, str)):
            out[key] = v
    return out

t0 = None
for tid, (name, typ) in topics.items():
    rows = []
    cls = get_message(typ)
    for ts, data in cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        m = deserialize_message(data, cls)
        r = {'t': ts / 1e9}
        r.update(fields(m))
        rows.append(r)
    if not rows:
        continue
    if t0 is None or rows[0]['t'] < t0:
        t0 = rows[0]['t']
    safe = name.strip('/').replace('/', '_')
    with open(os.path.join(OUT, safe + '.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"{name:45s} {len(rows):5d} rows -> {safe}.csv")
    print(f"   fields: {', '.join(list(rows[0].keys())[:24])}")
print(f"\nt0 = {t0}")
