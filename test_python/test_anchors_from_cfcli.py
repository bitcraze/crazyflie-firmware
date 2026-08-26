# Test the cfcli anchor-listing parser against the output formats it must
# survive. The human-readable table is verbatim `cfcli loco display` output
# captured from hardware (2026-07); the script's documented usage pipes it in.
from tools.usdlog.anchors_from_cfcli import parse, tokenize

CFCLI_TABLE = """\
Loco Positioning System - Anchor Data:
   ID  Active  Valid  Position (x, y, z)
    2     yes    yes  (-3.000, -4.080, 0.200)
    5     yes    yes  (3.000, -3.730, 3.180)
    6     yes    yes  (-3.000, -3.730, 3.180)
    8     yes    yes  (0.020, -0.020, 3.180)
    0     yes    yes  (3.000, 4.080, 0.200)
    4     yes    yes  (3.000, 3.710, 3.180)
    3     yes    yes  (-3.000, 4.080, 0.200)
    7     yes    yes  (-3.000, 3.710, 3.180)
    1     yes    yes  (3.000, -4.080, 0.200)
"""


def test_parses_cfcli_human_readable_table():
    anchors = parse(tokenize(CFCLI_TABLE.splitlines()))
    assert sorted(anchors) == list(range(9))
    assert anchors[2] == (-3.0, -4.08, 0.2)
    assert anchors[8] == (0.02, -0.02, 3.18)
    assert anchors[4] == (3.0, 3.71, 3.18)


def test_skips_inactive_or_invalid_rows():
    table = CFCLI_TABLE.replace("    5     yes    yes", "    5     yes     no")
    anchors = parse(tokenize(table.splitlines()))
    assert 5 not in anchors
    assert len(anchors) == 8


def test_parses_csv_with_header():
    csv_text = "id,x,y,z,valid\n0,1.0,2.0,3.0,yes\n1,-1.5,0.0,2.25,yes\n2,9.0,9.0,9.0,no\n"
    anchors = parse(tokenize(csv_text.splitlines()))
    assert anchors == {0: (1.0, 2.0, 3.0), 1: (-1.5, 0.0, 2.25)}


def test_empty_input_yields_no_anchors():
    assert parse(tokenize([])) == {}
