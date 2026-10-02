"""Tests for logparam_doc. Run with: uv run --with pytest pytest tools/logparam-doc"""

from pathlib import Path

import pytest

import json
import subprocess

from logparam_doc import (Condition, DocPart, anchor, default_ref, merge, parse_doc, read_kbuild, scan, scan_file,
                          write_json, write_markdown)


def write(root: Path, files: dict[str, str]) -> None:
    for name, text in files.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


def scan_c(tmp_path: Path, source: str, kbuild: tuple[str, ...] = ()):
    write(tmp_path, {"src/test.c": source})
    return scan_file(tmp_path, Path("src/test.c"), kbuild)


# Kbuild ----------------------------------------------------------------------

def test_kbuild_maps_files_to_symbols(tmp_path):
    write(tmp_path, {
        "src/Kbuild": "obj-y += a.o\nobj-$(CONFIG_B) += b.o\nobj-$(CONFIG_SUB) += sub/\n",
        "src/a.c": "", "src/b.c": "", "src/dead.c": "",
        "src/sub/Kbuild": "obj-y += c.o\nobj-$(CONFIG_D) += d.o\n",
        "src/sub/c.c": "", "src/sub/d.c": "",
    })
    assert read_kbuild(tmp_path, Path("src")) == {
        Path("src/a.c"): (),
        Path("src/b.c"): ("CONFIG_B",),
        Path("src/sub/c.c"): ("CONFIG_SUB",),
        Path("src/sub/d.c"): ("CONFIG_SUB", "CONFIG_D"),
    }


def test_kbuild_uses_vtpl_for_generated_files(tmp_path):
    write(tmp_path, {"src/Kbuild": "obj-y += version.o\n", "src/version.vtpl": ""})
    assert read_kbuild(tmp_path, Path("src")) == {Path("src/version.vtpl"): ()}


# Doc comments ----------------------------------------------------------------

def test_doc_brief_and_details():
    doc = parse_doc([
        "/**",
        " * @brief Motor thrust to set at idle",
        " * (default: 0)",
        " *",
        " * This is often needed for brushless motors.",
        " */",
    ])
    assert doc.brief == "Motor thrust to set at idle (default: 0)"
    assert doc.details == (DocPart("text", ("This is often needed for brushless motors.",)),)


def test_doc_tables_keep_their_rows_and_drop_doxygen_line_breaks():
    doc = parse_doc([
        "/**",
        " * @brief Angle [rad]",
        " *",
        " * | Base station | primary |\\n",
        " * | Sweep | 1 |\\n\\n",
        " *",
        " * Converted for V2.",
        " */",
    ])
    assert doc.details == (
        DocPart("table", ("| Base station | primary |", "| Sweep | 1 |")),
        DocPart("text", ("Converted for V2.",)),
    )


def test_doc_text_and_table_in_one_paragraph_are_split():
    doc = parse_doc(["/**", " * Effect", " *", " * Available effects:", " * | 0 | Off |", " */"])
    assert [p.kind for p in doc.details] == ["text", "table"]


def test_doc_single_line_and_addtogroup():
    assert parse_doc(["/** Power distribution parameters */"]).brief == "Power distribution parameters"
    assert parse_doc(["/** @addtogroup deck */"]) is None


# Scanner ---------------------------------------------------------------------

def test_scan_group_with_variables(tmp_path):
    groups, diagnostics = scan_c(tmp_path, """
/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, undocumented, &x)
PARAM_GROUP_STOP(powerDist)
""", kbuild=("CONFIG_POWER",))
    assert diagnostics == []
    [group] = groups
    assert (group.kind, group.name, group.doc.brief) == ("param", "powerDist", "Power distribution parameters")
    idle, undocumented = group.variables
    assert (idle.name, idle.type, idle.flags) == ("idleThrust", "uint32", {"core", "persistent"})
    assert idle.doc.brief == "Motor thrust to set at idle"
    assert idle.condition == Condition(("CONFIG_POWER",), ())
    assert idle.location.line == 9
    assert (undocumented.flags, undocumented.doc) == ({"read-only"}, None)


def test_scan_records_conditions_as_text(tmp_path):
    groups, _ = scan_c(tmp_path, """
#ifdef CRAZYFLIE_FW
#ifdef ENABLE_LOG
LOG_GROUP_START(ranging)
LOG_ADD(LOG_FLOAT, distance0, &d[0])
#if NR_OF_ANCHORS > 4
LOG_ADD(LOG_FLOAT, distance4, &d[4])
#else
LOG_ADD(LOG_FLOAT, few, &f)
#endif
#ifndef NO_EXTRA
LOG_ADD(LOG_FLOAT, extra, &e)
#endif
#if defined(A) && \\
    defined(B)
LOG_ADD(LOG_FLOAT, both, &b)
#endif
LOG_ADD_DEBUG(LOG_FLOAT, dbg, &x)
STATS_CNT_RATE_LOG_ADD(rate, &r)
LOG_GROUP_STOP(ranging)
#endif
#endif
""")
    [group] = groups
    assert group.condition.preproc == ("ENABLE_LOG",)  # CRAZYFLIE_FW is always defined
    conditions = {v.name: v.condition.preproc for v in group.variables}
    assert conditions == {
        "distance0": (),
        "distance4": ("NR_OF_ANCHORS > 4",),
        "few": ("!(NR_OF_ANCHORS > 4)",),
        "extra": ("!NO_EXTRA",),
        "both": ("defined(A) && defined(B)",),
        "dbg": ("CONFIG_DEBUG_LOG_ENABLE",),
        "rate": (),
    }
    assert {v.name: v.type for v in group.variables}["rate"] == "float"


def test_doc_is_not_taken_across_other_lines(tmp_path):
    groups, _ = scan_c(tmp_path, """
PARAM_GROUP_START(ctrl)
/* not a doc comment */
PARAM_ADD(PARAM_FLOAT, a, &a)
/** @brief Doc for b */
// Attitude I
PARAM_ADD(PARAM_FLOAT, b, &b)
PARAM_GROUP_STOP(ctrl)
""")
    assert [v.doc for v in groups[0].variables] == [None, None]


def test_name_is_the_whole_argument(tmp_path):
    groups, _ = scan_c(tmp_path, "LOG_GROUP_START(g)\nLOG_ADD(LOG_FLOAT, rate_d[0], &r[0])\nLOG_GROUP_STOP(g)\n")
    assert groups[0].variables[0].name == "rate_d[0]"


def test_prints_outside_groups_are_ignored(tmp_path):
    _, diagnostics = scan_c(tmp_path, 'void f() {\n  LOG_DEBUG("x");\n}\n')
    assert diagnostics == []


@pytest.mark.parametrize("source, message", [
    ("LOG_GROUP_START(a)\nLOG_ADD(LOG_FLOAT, x, &x)\n", "log group 'a' is never stopped"),
    ("LOG_GROUP_STOP(a)\n", "log group 'a' stopped without being started"),
    ("LOG_GROUP_START(a)\nLOG_GROUP_STOP(b)\n", "log group 'b' stopped, but the open group is log group 'a'"),
    ("LOG_GROUP_START(a)\nLOG_GROUP_START(b)\n", "log group 'b' starts before group 'a' is stopped"),
    ("LOG_GROUP_START(a)\nLOG_ADD_NEW(LOG_FLOAT, x, &x)\nLOG_GROUP_STOP(a)\n", "unknown macro LOG_ADD_NEW in log group 'a'"),
    ("LOG_GROUP_START(a)\nPARAM_ADD(PARAM_FLOAT, x, &x)\nLOG_GROUP_STOP(a)\n", "PARAM_ADD in log group 'a'"),
])
def test_structure_errors(tmp_path, source, message):
    _, diagnostics = scan_c(tmp_path, source)
    assert [(d.severity, d.message) for d in diagnostics][0] == ("error", message)


# Merge and validate ----------------------------------------------------------

def merge_files(tmp_path: Path, kbuild: str, files: dict[str, str]):
    write(tmp_path, {"src/Kbuild": kbuild, **{f"src/{name}": text for name, text in files.items()}})
    blocks, diagnostics = scan(tmp_path, Path("src"))
    assert diagnostics == []
    return merge(blocks)


def messages(diagnostics, severity="error"):
    return [d.message for d in diagnostics if d.severity == severity]


DOCUMENTED = "/** @brief Thrust at idle */\nPARAM_ADD(PARAM_UINT32, idleThrust, &t)\n"


def test_split_group_is_merged(tmp_path):
    groups, diagnostics = merge_files(tmp_path, "obj-y += a.o b.o\n", {
        "a.c": "/** System parameters */\nPARAM_GROUP_START(system)\n/** @brief A */\nPARAM_ADD(PARAM_UINT8, a, &a)\nPARAM_GROUP_STOP(system)\n",
        "b.c": "/** @addtogroup system */\nPARAM_GROUP_START(system)\n/** @brief B */\nPARAM_ADD(PARAM_UINT8, b, &b)\nPARAM_GROUP_STOP(system)\n",
    })
    assert diagnostics == []
    [group] = groups
    assert group.doc.brief == "System parameters"
    assert list(group.variables) == ["a", "b"]
    assert len(group.blocks) == 2


def test_group_descriptions_must_not_conflict(tmp_path):
    _, diagnostics = merge_files(tmp_path, "obj-y += a.o b.o\n", {
        "a.c": "/** Current sensor */\nPARAM_GROUP_START(flapper)\nPARAM_GROUP_STOP(flapper)\n",
        "b.c": "/** Flapper configuration */\nPARAM_GROUP_START(flapper)\nPARAM_GROUP_STOP(flapper)\n",
    })
    assert messages(diagnostics) == ["param group 'flapper' has different descriptions in src/a.c:2, src/b.c:2, keep one"]


def test_identical_group_descriptions_are_fine(tmp_path):
    same = "/** System state */\nLOG_GROUP_START(sys)\nLOG_GROUP_STOP(sys)\n"
    _, diagnostics = merge_files(tmp_path, "obj-y += a.o b.o\n", {"a.c": same, "b.c": same})
    assert diagnostics == []


def test_alternatives_under_different_kbuild_symbols(tmp_path):
    group = "PARAM_GROUP_START(powerDist)\n" + DOCUMENTED + "PARAM_GROUP_STOP(powerDist)\n"
    groups, diagnostics = merge_files(tmp_path, "obj-$(CONFIG_QUAD) += quad.o\nobj-$(CONFIG_FLAPPER) += flapper.o\n",
                                      {"quad.c": group, "flapper.c": group})
    assert diagnostics == []
    assert len(groups[0].variables["idleThrust"].definitions) == 2


def test_alternatives_must_have_identical_descriptions(tmp_path):
    _, diagnostics = merge_files(tmp_path, "obj-$(CONFIG_QUAD) += quad.o\nobj-$(CONFIG_FLAPPER) += flapper.o\n", {
        "quad.c": "PARAM_GROUP_START(powerDist)\n" + DOCUMENTED + "PARAM_GROUP_STOP(powerDist)\n",
        "flapper.c": "PARAM_GROUP_START(powerDist)\n/** @brief Other */\nPARAM_ADD(PARAM_UINT32, idleThrust, &t)\nPARAM_GROUP_STOP(powerDist)\n",
    })
    assert messages(diagnostics) == ["param powerDist.idleThrust has different descriptions in src/flapper.c:3, src/quad.c:3, make them identical"]


@pytest.mark.parametrize("kbuild, files", [
    # same Kbuild symbol
    ("obj-$(CONFIG_A) += a.o b.o\n", {"a.c": "x", "b.c": "x"}),
    # same file, two blocks
    ("obj-y += a.o\n", {"a.c": "xx"}),
])
def test_duplicates_always_built_together(tmp_path, kbuild, files):
    block = "PARAM_GROUP_START(g)\n" + DOCUMENTED + "PARAM_GROUP_STOP(g)\n"
    _, diagnostics = merge_files(tmp_path, kbuild, {name: block * len(text) for name, text in files.items()})
    assert len(messages(diagnostics)) == 1
    assert "param g.idleThrust is already defined at" in messages(diagnostics)[0]


def test_duplicates_inside_if_are_alternatives(tmp_path):
    _, diagnostics = merge_files(tmp_path, "obj-y += a.o b.o\n", {
        "a.c": "#ifdef RAW_LOG\nPARAM_GROUP_START(g)\n" + DOCUMENTED + "PARAM_GROUP_STOP(g)\n#endif\n",
        "b.c": "PARAM_GROUP_START(g)\n" + DOCUMENTED + "PARAM_GROUP_STOP(g)\n",
    })
    assert diagnostics == []


def test_missing_descriptions(tmp_path):
    _, diagnostics = merge_files(tmp_path, "obj-y += a.o\n", {
        "a.c": "PARAM_GROUP_START(g)\nPARAM_ADD_CORE(PARAM_UINT8, c, &c)\nPARAM_ADD(PARAM_UINT8, n, &n)\nPARAM_GROUP_STOP(g)\n",
    })
    assert messages(diagnostics) == ["core param g.c has no description"]
    assert messages(diagnostics, "warning") == ["param g.n has no description"]


# Writers ---------------------------------------------------------------------

POWER_DIST = """
/** Power distribution parameters */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * Needed for brushless motors.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &t)
PARAM_GROUP_STOP(powerDist)
"""

RANGING = """
/** Distances to anchors, see [docs](%https://example.com/loco) */
LOG_GROUP_START(ranging)
LOG_ADD(LOG_UINT16, state, &s)
#if NR_OF_ANCHORS > 4
/** @brief Distance to anchor 4 [m] */
LOG_ADD(LOG_FLOAT, distance4, &d[4])
#endif
/**
 * @brief Mode
 *
 * | Id | Mode |
 * | -  | -    |\\n
 * | 0  | Auto |\\n
 *
 * Set by the client.
 */
LOG_ADD(LOG_UINT8, mode, &m)
LOG_GROUP_STOP(ranging)
"""


def generate(tmp_path: Path, kbuild: str, files: dict[str, str]):
    groups, diagnostics = merge_files(tmp_path, kbuild, files)
    assert messages(diagnostics) == []
    return groups


def test_anchor_matches_kramdown_ids():
    assert anchor("ranging.distance0") == "rangingdistance0"
    assert anchor("activeMarker") == "activemarker"
    assert anchor("lighthouse.angle1y_1 details") == "lighthouseangle1y_1-details"


def test_markdown_group(tmp_path):
    groups = generate(tmp_path, "obj-$(CONFIG_DECK_LOCO) += loco.o\n", {"loco.c": RANGING})
    md = write_markdown(groups, "log", ref="2026.04")
    assert "* [ranging](#ranging)" in md
    assert "## ranging\n\nDistances to anchors, see [docs](https://example.com/loco)\n" in md
    assert ("Requires `CONFIG_DECK_LOCO` "
            "([loco.c](https://github.com/bitcraze/crazyflie-firmware/blob/2026.04/src/loco.c#L3))") in md
    assert '| <span id="rangingstate"></span>ranging.state | uint16 |  | *No description* |  |' in md
    assert ('| <span id="rangingdistance4"></span>ranging.distance4 | float |  | Distance to anchor 4 [m] '
            '| `NR_OF_ANCHORS > 4` |') in md
    assert "| Mode [details below](#rangingmode-details) |" in md
    assert "#### ranging.mode details\n\nMode\n\n| Id | Mode |\n| -  | -    |\n| 0  | Auto |\n\nSet by the client.\n" in md


def test_markdown_flags_details_and_alternatives(tmp_path):
    groups = generate(tmp_path, "obj-$(CONFIG_QUAD) += quad.o\nobj-$(CONFIG_FLAPPER) += flapper.o\n",
                      {"quad.c": POWER_DIST, "flapper.c": POWER_DIST})
    md = write_markdown(groups, "param", ref="master")
    assert "Requires `CONFIG_" not in md  # nothing shared, each entry shows its own condition
    assert ("| uint32 | core, persistent | Motor thrust to set at idle (default: 0)"
            "<br><small>Needed for brushless motors.</small> | `CONFIG_FLAPPER` ([flapper.c]("
            "https://github.com/bitcraze/crazyflie-firmware/blob/master/src/flapper.c#L9)) or `CONFIG_QUAD` ([quad.c](") in md


def test_markdown_entries_with_their_own_conditions(tmp_path):
    # Like the deck group: every driver adds its own entry under its own Kbuild symbol
    deck = "/** @brief Nonzero if the {0} deck is attached */\nPARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bc{0}, &i)\n"
    groups = generate(tmp_path, "obj-$(CONFIG_DECKS) += decks/\n", {
        "decks/Kbuild": "obj-$(CONFIG_DECK_AI) += ai.o\nobj-$(CONFIG_DECK_FLOW) += flow.o\n",
        "decks/ai.c": "/** Attached decks */\nPARAM_GROUP_START(deck)\n" + deck.format("AI") + "PARAM_GROUP_STOP(deck)\n",
        "decks/flow.c": "PARAM_GROUP_START(deck)\n" + deck.format("Flow") + "PARAM_GROUP_STOP(deck)\n",
    })
    md = write_markdown(groups, "param", ref="master")
    assert "\nRequires `CONFIG_DECKS`\n" in md  # shared by all entries, no files
    assert "deck.bcAI | uint8 | core, read-only | Nonzero if the AI deck is attached | `CONFIG_DECK_AI` ([ai.c](" in md
    assert "deck.bcFlow | uint8 | core, read-only | Nonzero if the Flow deck is attached | `CONFIG_DECK_FLOW` ([flow.c](" in md


def test_markdown_unconditional_group(tmp_path):
    groups = generate(tmp_path, "obj-y += a.o\n", {"a.c": POWER_DIST})
    assert "Defined in [a.c](" in write_markdown(groups, "param", ref="master")


def test_group_details_and_links(tmp_path):
    source = """
/**
 * Sensor fusion parameters, see [docs](%https://example.com/fusion)
 *
 * Uses the accelerometer and the gyro.
 */
PARAM_GROUP_START(sensfusion6)
/** @brief Gain, see [docs](%https://example.com/gain) and %https://example.com/pull/903 */
PARAM_ADD(PARAM_FLOAT, kp, &kp)
PARAM_GROUP_STOP(sensfusion6)
"""
    groups = generate(tmp_path, "obj-y += a.o\n", {"a.c": source})
    md = write_markdown(groups, "param", ref="master")
    assert ("## sensfusion6\n\nSensor fusion parameters, see [docs](https://example.com/fusion)\n\n"
            "Uses the accelerometer and the gyro.\n") in md
    group = json.loads(write_json(groups))["params"]["sensfusion6"]
    assert group["desc"] == "Sensor fusion parameters, see [docs](https://example.com/fusion)\n\nUses the accelerometer and the gyro."
    assert group["variables"]["kp"]["short_desc"] == "Gain, see [docs](https://example.com/gain) and https://example.com/pull/903"
    assert "%http" not in write_json(groups)


def test_groups_without_entries_are_left_out(tmp_path):
    groups = generate(tmp_path, "obj-y += a.o\n", {
        "a.c": "LOG_GROUP_START(empty)\n  //LOG_ADD(LOG_FLOAT, ox, &x)\nLOG_GROUP_STOP(empty)\n" + RANGING,
    })
    assert "empty" not in write_markdown(groups, "log", ref="master")
    assert "empty" not in json.loads(write_json(groups))["logs"]


def test_default_ref(tmp_path):
    def git(*args):
        return subprocess.run(["git", "-C", str(tmp_path), *args], check=True, capture_output=True, text=True).stdout.strip()

    assert default_ref(tmp_path) == "master"
    git("init", "-q")
    git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "first")
    assert default_ref(tmp_path) == git("rev-parse", "HEAD")
    git("tag", "2026.04")
    assert default_ref(tmp_path) == "2026.04"


def test_json_keeps_the_structure_the_client_reads(tmp_path):
    groups = generate(tmp_path, "obj-y += a.o b.o\n", {"a.c": POWER_DIST, "b.c": RANGING})
    data = json.loads(write_json(groups))
    assert data["params"]["powerDist"] == {
        "desc": "Power distribution parameters",
        "variables": {"idleThrust": {"core": True, "short_desc": "Motor thrust to set at idle (default: 0)",
                                     "type": "PARAM_UINT32, PARAM_PERSISTENT", "desc": "Needed for brushless motors."}},
    }
    assert data["logs"]["ranging"]["variables"]["state"] == {"core": False, "short_desc": "", "type": "LOG_UINT16", "desc": ""}
    assert data["logs"]["ranging"]["variables"]["mode"]["desc"] == "| Id | Mode |\n| -  | -    |\n| 0  | Auto |\n\nSet by the client."


# Firmware --------------------------------------------------------------------

def test_firmware_source_has_no_errors():
    root = Path(__file__).resolve().parents[2]
    blocks, diagnostics = scan(root, Path("src"))
    groups, merge_diagnostics = merge(blocks)
    assert [str(d) for d in diagnostics + merge_diagnostics if d.severity == "error"] == []
    assert len(groups) > 100
