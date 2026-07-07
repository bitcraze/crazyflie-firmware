"""
Convert `cfcli loco display` output into the anchors YAML consumed by the
replay harness (bindings/util/loco_utils.read_loco_anchor_positions), i.e.::

    0: {x: 0.0, y: 0.0, z: 0.0}
    1: {x: 4.0, y: 0.0, z: 0.0}
    ...

Usage:
    cfcli -u <uri> loco display | python3 -m tools.usdlog.anchors_from_cfcli > anchors.yaml

The parser is intentionally tolerant about the input format: it accepts both
the human-readable table cfcli prints (`ID Active Valid Position (x, y, z)`,
verified against cfcli as of 2026-07) and CSV. Lines are tokenized on
whitespace, commas and parentheses; a header line maps columns by name, any
other line falls back to positional parsing (first integer = id, last three
floats = x, y, z). Rows containing a 'no' token (inactive/invalid anchors)
are skipped.
"""
import re
import sys


def _to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def tokenize(lines):
    """Split each line on whitespace, commas and parentheses."""
    rows = []
    for line in lines:
        tokens = [t for t in re.split(r'[\s,()]+', line) if t]
        if tokens:
            rows.append(tokens)
    return rows


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
        # Header-index mapping is only trustworthy when the row has as many
        # tokens as the header (the human-readable table's "Position (x, y, z)"
        # header tokenizes wider than its data rows).
        if has_header and None not in (id_i, x_i, y_i, z_i) and len(r) == len(header):
            if valid_i is not None and r[valid_i].lower() in ('no', 'false', '0'):
                continue
            try:
                aid = int(r[id_i])
            except (ValueError, IndexError):
                continue
            x, y, z = _to_float(r[x_i]), _to_float(r[y_i]), _to_float(r[z_i])
        else:
            # Positional fallback: first int is id, last three floats are x/y/z.
            if any(c.lower() in ('no', 'false') for c in r):
                continue  # inactive or invalid anchor
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
    anchors = parse(tokenize(sys.stdin))
    if not anchors:
        sys.stderr.write('No anchors parsed from input. Check `cfcli loco display` output.\n')
        sys.exit(1)
    out = sys.stdout
    for aid in sorted(anchors):
        x, y, z = anchors[aid]
        out.write(f'{aid}: {{x: {x}, y: {y}, z: {z}}}\n')


if __name__ == '__main__':
    main()
