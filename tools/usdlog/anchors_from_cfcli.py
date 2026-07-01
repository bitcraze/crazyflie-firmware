"""
Convert `cfcli loco display --csv` output into the anchors YAML consumed by the
replay harness (bindings/util/loco_utils.read_loco_anchor_positions), i.e.::

    0: {x: 0.0, y: 0.0, z: 0.0}
    1: {x: 4.0, y: 0.0, z: 0.0}
    ...

Usage:
    cfcli -u <uri> --csv loco display | python3 -m tools.usdlog.anchors_from_cfcli > anchors.yaml

The parser is intentionally tolerant about column naming: it looks for an 'id'
column and x/y/z columns by header name, and falls back to positional parsing
(first integer = id, last three floats = x, y, z) if there is no usable header.
Only anchors reported as valid (if a 'valid' column exists) are emitted.
"""
import csv
import sys


def _to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def parse(rows):
    rows = [r for r in rows if r and any(c.strip() for c in r)]
    if not rows:
        return {}

    header = [c.strip().lower() for c in rows[0]]
    has_header = any(h in header for h in ('id', 'anchor', 'x', 'y', 'z'))

    def col(*names):
        for n in names:
            if n in header:
                return header.index(n)
        return None

    id_i = col('id', 'anchor', 'anchor_id') if has_header else None
    x_i = col('x', 'pos_x', 'position_x') if has_header else None
    y_i = col('y', 'pos_y', 'position_y') if has_header else None
    z_i = col('z', 'pos_z', 'position_z') if has_header else None
    valid_i = col('valid') if has_header else None

    data_rows = rows[1:] if has_header else rows
    result = {}
    for r in data_rows:
        r = [c.strip() for c in r]
        if has_header and None not in (id_i, x_i, y_i, z_i):
            if valid_i is not None and r[valid_i].lower() in ('no', 'false', '0'):
                continue
            try:
                aid = int(r[id_i])
            except (ValueError, IndexError):
                continue
            x, y, z = _to_float(r[x_i]), _to_float(r[y_i]), _to_float(r[z_i])
        else:
            # Positional fallback: first int is id, last three floats are x/y/z.
            ints = [int(c) for c in r if c.lstrip('-').isdigit()]
            floats = [f for f in (_to_float(c) for c in r) if f is not None]
            if not ints or len(floats) < 3:
                continue
            aid = ints[0]
            x, y, z = floats[-3], floats[-2], floats[-1]
        if None in (x, y, z):
            continue
        result[aid] = (x, y, z)
    return result


def main():
    reader = csv.reader(sys.stdin)
    anchors = parse(list(reader))
    if not anchors:
        sys.stderr.write('No anchors parsed from input. Check `cfcli loco display --csv` output.\n')
        sys.exit(1)
    out = sys.stdout
    for aid in sorted(anchors):
        x, y, z = anchors[aid]
        out.write(f'{aid}: {{x: {x}, y: {y}, z: {z}}}\n')


if __name__ == '__main__':
    main()
