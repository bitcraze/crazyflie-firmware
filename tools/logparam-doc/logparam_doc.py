#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Generate the log and parameter documentation from the firmware source.

Every log and parameter entry in the source is documented, whatever the build
configuration. Build conditions are never evaluated: the Kbuild symbol that
builds a file and the #if lines around an entry are shown as text.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

Kind = Literal["log", "param"]
Severity = Literal["error", "warning"]


# ---------------------------------------------------------------------------
# Data model, layer 1: what the source says

@dataclass(frozen=True)
class Location:
    file: Path
    line: int

    def __str__(self) -> str:
        return f"{self.file}:{self.line}"


@dataclass(frozen=True)
class DocPart:
    kind: Literal["text", "table"]
    lines: tuple[str, ...]  # table: one row per line, text: one paragraph


@dataclass(frozen=True)
class Doc:
    brief: str
    details: tuple[DocPart, ...]


@dataclass(frozen=True)
class Condition:
    kbuild: tuple[str, ...]   # Kbuild symbols that build the file, empty for obj-y
    preproc: tuple[str, ...]  # #if lines around the entry, as text


@dataclass(frozen=True)
class RawVariable:
    name: str
    type: str                # "float", "uint8", ...
    flags: frozenset[str]    # core, persistent, read-only
    doc: Doc | None
    condition: Condition
    location: Location


@dataclass(frozen=True)
class RawGroup:
    """One START...STOP block."""
    kind: Kind
    name: str
    doc: Doc | None
    variables: tuple[RawVariable, ...]
    condition: Condition
    location: Location


# ---------------------------------------------------------------------------
# Data model, layer 2: the merged view that gets documented

@dataclass(frozen=True)
class Definition:
    """One place where a variable is defined, with the block it is in."""
    block: RawGroup
    variable: RawVariable

    @property
    def inside_if(self) -> bool:
        return bool(self.block.condition.preproc or self.variable.condition.preproc)


@dataclass
class Variable:
    group: str
    name: str
    definitions: list[Definition]  # more than one means alternatives

    @property
    def first(self) -> RawVariable:
        return self.definitions[0].variable


@dataclass
class Group:
    kind: Kind
    name: str
    doc: Doc | None
    blocks: list[RawGroup]
    variables: dict[str, Variable]


@dataclass(frozen=True)
class Diagnostic:
    severity: Severity
    location: Location
    message: str

    def __str__(self) -> str:
        return f"{self.location}: {self.severity}: {self.message}"


# ---------------------------------------------------------------------------
# Kbuild

KBUILD_LINE = re.compile(r"^obj-(?:y|\$\((CONFIG_\w+)\))\s*\+?=\s*(.*)$")


def read_kbuild(root: Path, directory: Path, symbols: tuple[str, ...] = ()) -> dict[Path, tuple[str, ...]]:
    """Map each built source file below `directory` to the Kbuild symbols that build it.

    Make conditionals (ifeq/ifneq) are ignored. An object is built from foo.c,
    or from foo.vtpl for the generated version file.
    """
    files: dict[Path, tuple[str, ...]] = {}
    kbuild = root / directory / "Kbuild"
    if not kbuild.exists():
        return files
    for line in kbuild.read_text().splitlines():
        match = KBUILD_LINE.match(line.strip())
        if not match:
            continue
        symbol, objects = match.groups()
        line_symbols = symbols + ((symbol,) if symbol else ())
        for obj in objects.split():
            if obj.endswith("/"):
                files.update(read_kbuild(root, directory / obj, line_symbols))
            elif obj.endswith(".o"):
                for suffix in (".c", ".vtpl"):
                    source = directory / (obj[:-2] + suffix)
                    if (root / source).exists():
                        files[source] = line_symbols
                        break
    return files


# ---------------------------------------------------------------------------
# Scanner

GROUP_START = re.compile(r"^(LOG|PARAM)_GROUP_START\((\w+)\)")
GROUP_STOP = re.compile(r"^(LOG|PARAM)_GROUP_STOP\((\w+)\)")
# Anything that looks like a registry macro inside a group
REGISTRY_CALL = re.compile(r"^((?:LOG|PARAM|STATS_CNT_RATE_LOG)_\w+)\((.*)\)\s*;?\s*(?://.*|/\*.*\*/)?$")
PREPROC = re.compile(r"^#\s*(\w+)\s*(.*)$")

# Macro -> (kind, flags given by the macro, index of the type argument, index of the name argument)
MACROS: dict[str, tuple[Kind, frozenset[str], int | None, int]] = {
    "LOG_ADD": ("log", frozenset(), 0, 1),
    "LOG_ADD_CORE": ("log", frozenset({"core"}), 0, 1),
    "LOG_ADD_BY_FUNCTION": ("log", frozenset(), 0, 1),
    "LOG_ADD_DEBUG": ("log", frozenset(), 0, 1),
    "STATS_CNT_RATE_LOG_ADD": ("log", frozenset(), None, 0),
    "STATS_CNT_RATE_LOG_ADD_DEBUG": ("log", frozenset(), None, 0),
    "PARAM_ADD": ("param", frozenset(), 0, 1),
    "PARAM_ADD_CORE": ("param", frozenset({"core"}), 0, 1),
    "PARAM_ADD_WITH_CALLBACK": ("param", frozenset(), 0, 1),
    "PARAM_ADD_CORE_WITH_CALLBACK": ("param", frozenset({"core"}), 0, 1),
}

TYPES = {"UINT8", "UINT16", "UINT32", "INT8", "INT16", "INT32", "FLOAT", "FP16"}
TYPE_FLAGS = {"PERSISTENT": "persistent", "RONLY": "read-only", "CORE": "core"}

# Always defined in firmware builds (tools/kbuild/Makefile.kbuild)
ALWAYS_DEFINED = {"CRAZYFLIE_FW"}


def strip_comment(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text)
    return text.split("//")[0].strip()


def split_args(text: str) -> list[str]:
    """Split macro arguments on top level commas."""
    args, depth, current = [], 0, ""
    for char in text:
        if char == "," and depth == 0:
            args.append(current.strip())
            current = ""
            continue
        depth += char in "([{"
        depth -= char in ")]}"
        current += char
    args.append(current.strip())
    return args


class PreprocStack:
    """The #if lines around the current line, as text."""

    def __init__(self) -> None:
        self._stack: list[list[str]] = []  # per #if: the conditions of all branches so far

    def handle(self, directive: str, argument: str) -> None:
        argument = " ".join(strip_comment(argument).split())
        if directive in ("if", "ifdef", "ifndef"):
            condition = {"if": argument, "ifdef": argument, "ifndef": f"!{argument}"}[directive]
            self._stack.append([condition])
        elif directive in ("elif", "else") and self._stack:
            self._stack[-1].append(argument if directive == "elif" else "")
        elif directive == "endif" and self._stack:
            self._stack.pop()

    def conditions(self) -> tuple[str, ...]:
        result = []
        for branches in self._stack:
            *previous, current = branches
            parts = [negate(c) for c in previous] + ([current] if current else [])
            parts = [p for p in parts if p not in ALWAYS_DEFINED]
            result.extend(parts)
        return tuple(result)


def negate(condition: str) -> str:
    if condition.startswith("!") and re.fullmatch(r"!\w+", condition):
        return condition[1:]
    if re.fullmatch(r"\w+", condition):
        return f"!{condition}"
    return f"!({condition})"


def parse_doc(lines: list[str]) -> Doc | None:
    """Parse the lines of a /** ... */ comment."""
    text = []
    for line in lines:
        line = re.sub(r"^\s*/\*\*+", "", line)
        line = re.sub(r"\*+/\s*$", "", line)
        line = re.sub(r"^\s*\*(?!/)\s?", "", line)
        line = re.sub(r"(\s*\\n)+\s*$", "", line)  # Doxygen line break command
        text.append(line.rstrip())
    if any(re.match(r"\s*[@\\]addtogroup\b", line) for line in text):
        return None

    paragraphs: list[list[str]] = [[]]
    for line in text:
        if line.strip():
            paragraphs[-1].append(line.strip())
        elif paragraphs[-1]:
            paragraphs.append([])
    paragraphs = [p for p in paragraphs if p]
    if not paragraphs:
        return None

    brief = re.sub(r"^[@\\]brief\s*", "", " ".join(" ".join(paragraphs[0]).split()))
    details: list[DocPart] = []
    for paragraph in paragraphs[1:]:
        # A paragraph can hold text and table rows, split it into runs
        run: list[str] = []
        for line in paragraph:
            is_table = line.startswith("|")
            if run and run[0].startswith("|") != is_table:
                details.append(make_part(run))
                run = []
            run.append(line)
        details.append(make_part(run))
    return Doc(brief=brief, details=tuple(details))


def make_part(lines: list[str]) -> DocPart:
    if lines[0].startswith("|"):
        return DocPart("table", tuple(lines))
    return DocPart("text", (" ".join(" ".join(lines).split()),))


def doc_above(lines: list[str], index: int) -> Doc | None:
    """The doc comment directly above lines[index], skipping blank and preprocessor lines."""
    end = index - 1
    while end >= 0 and (not lines[end].strip() or lines[end].strip().startswith("#")):
        end -= 1
    if end < 0 or not lines[end].rstrip().endswith("*/"):
        return None
    start = end
    while start > 0 and "/*" not in lines[start]:
        start -= 1
    if not lines[start].lstrip().startswith("/**"):
        return None
    return parse_doc(lines[start:end + 1])


def scan_file(root: Path, path: Path, kbuild: tuple[str, ...]) -> tuple[list[RawGroup], list[Diagnostic]]:
    """Find all log and parameter groups in one source file."""
    lines = (root / path).read_text(errors="replace").splitlines()
    groups: list[RawGroup] = []
    diagnostics: list[Diagnostic] = []
    preproc = PreprocStack()

    current: dict | None = None  # the open group
    index = -1
    while index + 1 < len(lines):
        index += 1
        line = lines[index].strip()
        location = Location(path, index + 1)

        if match := PREPROC.match(line):
            argument = match.group(2)
            while argument.endswith("\\") and index + 1 < len(lines):
                index += 1
                argument = argument[:-1] + " " + lines[index].strip()
            preproc.handle(match.group(1), argument)
            continue

        if match := GROUP_START.match(line):
            kind, name = match.group(1).lower(), match.group(2)
            if current:
                diagnostics.append(Diagnostic("error", location, f"{kind} group '{name}' starts before group '{current['name']}' is stopped"))
            current = {"kind": kind, "name": name, "doc": doc_above(lines, index), "variables": [],
                       "condition": Condition(kbuild, preproc.conditions()), "location": location}
            continue

        if match := GROUP_STOP.match(line):
            kind, name = match.group(1).lower(), match.group(2)
            if not current:
                diagnostics.append(Diagnostic("error", location, f"{kind} group '{name}' stopped without being started"))
            elif (kind, name) != (current["kind"], current["name"]):
                diagnostics.append(Diagnostic("error", location, f"{kind} group '{name}' stopped, but the open group is {current['kind']} group '{current['name']}'"))
                current = None
            else:
                groups.append(RawGroup(current["kind"], name, current["doc"], tuple(current["variables"]),
                                       current["condition"], current["location"]))
                current = None
            continue

        if current and (match := REGISTRY_CALL.match(line)):
            macro, arguments = match.groups()
            if macro not in MACROS:
                diagnostics.append(Diagnostic("error", location, f"unknown macro {macro} in {current['kind']} group '{current['name']}'"))
                continue
            kind, flags, type_index, name_index = MACROS[macro]
            if kind != current["kind"]:
                diagnostics.append(Diagnostic("error", location, f"{macro} in {current['kind']} group '{current['name']}'"))
                continue
            args = split_args(arguments)
            var_type, type_flags = parse_type(args[type_index]) if type_index is not None else ("float", frozenset())
            conditions = preproc.conditions()[len(current["condition"].preproc):]
            if macro.endswith("_DEBUG"):  # only built with debug logging, see log.h and statsCnt.h
                conditions += ("CONFIG_DEBUG_LOG_ENABLE",)
            current["variables"].append(RawVariable(
                name=args[name_index], type=var_type, flags=flags | type_flags, doc=doc_above(lines, index),
                condition=Condition(kbuild, conditions), location=location))

    if current:
        diagnostics.append(Diagnostic("error", current["location"], f"{current['kind']} group '{current['name']}' is never stopped"))
    return groups, diagnostics


def parse_type(text: str) -> tuple[str, frozenset[str]]:
    """'PARAM_FLOAT | PARAM_PERSISTENT' -> ('float', {'persistent'})"""
    var_type, flags = "unknown", set()
    for token in (t.strip() for t in text.split("|")):
        suffix = token.split("_", 1)[-1]
        if suffix in TYPES:
            var_type = suffix.lower()
        elif suffix in TYPE_FLAGS:
            flags.add(TYPE_FLAGS[suffix])
    return var_type, frozenset(flags)


def scan(root: Path, src: Path) -> tuple[list[RawGroup], list[Diagnostic]]:
    """Scan every built source file below `src`."""
    groups: list[RawGroup] = []
    diagnostics: list[Diagnostic] = []
    for path, kbuild in sorted(read_kbuild(root, src).items()):
        file_groups, file_diagnostics = scan_file(root, path, kbuild)
        groups.extend(file_groups)
        diagnostics.extend(file_diagnostics)
    return groups, diagnostics


# ---------------------------------------------------------------------------
# Merge and validate

def merge(blocks: list[RawGroup]) -> tuple[list[Group], list[Diagnostic]]:
    """Merge the blocks of each group and check the result.

    Checks that docs exist and that there is one unambiguous description for
    every group and variable, never what the docs say.
    """
    groups: dict[tuple[Kind, str], Group] = {}
    for block in blocks:
        group = groups.setdefault((block.kind, block.name), Group(block.kind, block.name, None, [], {}))
        group.blocks.append(block)
        for raw in block.variables:
            variable = group.variables.setdefault(raw.name, Variable(block.name, raw.name, []))
            variable.definitions.append(Definition(block, raw))

    diagnostics: list[Diagnostic] = []
    for group in groups.values():
        diagnostics += check_group_doc(group)
        for variable in group.variables.values():
            diagnostics += check_duplicates(group, variable)
            diagnostics += check_variable_doc(group, variable)
    return list(groups.values()), diagnostics


def check_group_doc(group: Group) -> list[Diagnostic]:
    """A group may be split over blocks, but has at most one distinct description."""
    documented = [b for b in group.blocks if b.doc]
    distinct = list(dict.fromkeys(b.doc for b in documented))
    group.doc = distinct[0] if distinct else None
    if len(distinct) > 1:
        places = ", ".join(str(b.location) for b in documented)
        return [Diagnostic("error", documented[0].location,
                           f"{group.kind} group '{group.name}' has different descriptions in {places}, keep one")]
    return []


def check_duplicates(group: Group, variable: Variable) -> list[Diagnostic]:
    """The same group.name twice is a conflict when both are always built together.

    Definitions in different files with different Kbuild symbols, or inside an
    #if, are alternatives. Whether alternatives can still end up in the same
    build is not decided here.
    """
    diagnostics = []
    definitions = variable.definitions
    for i, a in enumerate(definitions):
        for b in definitions[i + 1:]:
            if a.inside_if or b.inside_if:
                continue
            same_file = a.variable.location.file == b.variable.location.file
            same_kbuild = a.variable.condition.kbuild == b.variable.condition.kbuild
            if same_file or same_kbuild:
                diagnostics.append(Diagnostic("error", b.variable.location,
                                              f"{group.kind} {group.name}.{variable.name} is already defined at {a.variable.location}"))
    if len(definitions) > 1 and len({d.variable.doc for d in definitions}) > 1:
        places = ", ".join(str(d.variable.location) for d in definitions)
        diagnostics.append(Diagnostic("error", definitions[0].variable.location,
                                      f"{group.kind} {group.name}.{variable.name} has different descriptions in {places}, make them identical"))
    return diagnostics


def check_variable_doc(group: Group, variable: Variable) -> list[Diagnostic]:
    if variable.first.doc:
        return []
    if any("core" in d.variable.flags for d in variable.definitions):
        return [Diagnostic("error", variable.first.location, f"core {group.kind} {group.name}.{variable.name} has no description")]
    return [Diagnostic("warning", variable.first.location, f"{group.kind} {group.name}.{variable.name} has no description")]


# ---------------------------------------------------------------------------
# Writers

SOURCE_URL = "https://github.com/bitcraze/crazyflie-firmware/blob/{ref}/{path}#L{line}"
MD_FILES = {"log": "logs.md_raw", "param": "params.md_raw"}
JSON_FILE = "log_param_doc.json"


def anchor(text: str) -> str:
    """The id kramdown gives a heading with this text."""
    text = re.sub(r"[^a-z0-9 _-]", "", text.lower())
    return text.replace(" ", "-")


def inline(text: str) -> str:
    """Comment text for use in markdown, links written as (%https://...) work as normal links."""
    return re.sub(r"\(%(https?://)", r"(\1", text)


def cell(text: str) -> str:
    return inline(text).replace("|", "\\|")


def default_ref(root: Path) -> str:
    """The release tag at HEAD, else the commit hash, else master when not in a git checkout."""
    for command in (["describe", "--tags", "--exact-match", "HEAD"], ["rev-parse", "HEAD"]):
        result = subprocess.run(["git", "-C", str(root), *command], capture_output=True, text=True)
        if result.returncode == 0:
            return result.stdout.strip()
    return "master"


def source_link(location: Location, ref: str) -> str:
    url = SOURCE_URL.format(ref=ref, path=location.file.as_posix(), line=location.line)
    return f"[{location.file.name}]({url})"


def condition_text(conditions: tuple[str, ...]) -> str:
    return " and ".join(f"`{c}`" for c in conditions)


def block_conditions(block: RawGroup) -> tuple[str, ...]:
    return block.condition.kbuild + block.condition.preproc


def shared_conditions(group: Group) -> tuple[str, ...]:
    """The conditions every block of the group has."""
    first, *rest = (block_conditions(b) for b in group.blocks)
    return tuple(c for c in first if all(c in other for other in rest))


def per_entry_conditions(group: Group) -> bool:
    """True when the blocks of a split group are built under different conditions.

    Then each entry shows its own condition, e.g. the deck group where every
    deck driver adds its own entry.
    """
    return len({block_conditions(b) for b in group.blocks}) > 1


def group_requires(group: Group, ref: str) -> str | None:
    """A line with the conditions that build the group, and the files when they are the same for every entry."""
    files = ", ".join(source_link(b.location, ref) for b in group.blocks)
    shared = shared_conditions(group)
    if per_entry_conditions(group):
        return f"Requires {condition_text(shared)}" if shared else None
    return f"Requires {condition_text(shared)} ({files})" if shared else f"Defined in {files}"


def entry_requires(group: Group, variable: Variable, ref: str) -> str:
    """The conditions an entry needs beyond the group line, alternatives joined with 'or'."""
    shared = shared_conditions(group)
    parts = []
    for definition in variable.definitions:
        conditions = definition.variable.condition.preproc
        if per_entry_conditions(group):
            own = tuple(c for c in block_conditions(definition.block) if c not in shared)
            text = condition_text(own + conditions)
            link = source_link(definition.variable.location, ref)
            parts.append(f"{text} ({link})" if text else f"always ({link})")
        elif conditions:
            parts.append(condition_text(conditions))
    return " or ".join(dict.fromkeys(parts))


def has_table(doc: Doc | None) -> bool:
    return bool(doc) and any(part.kind == "table" for part in doc.details)


def write_group(group: Group, ref: str) -> list[str]:
    out = ["", "---", "[back to group index](#index)", "", f"## {group.name}", ""]
    out += [inline(group.doc.brief) if group.doc else "*No description*", ""]
    if requires := group_requires(group, ref):
        out += [requires, ""]
    out += ["| Name | Type | Flags | Description | Requires |", "| --- | --- | --- | --- | --- |"]

    sections = []
    for variable in group.variables.values():
        raw = variable.first
        full_name = f"{group.name}.{variable.name}"
        flags = ", ".join(f for f in ("core", "persistent", "read-only")
                          if any(f in d.variable.flags for d in variable.definitions))
        if raw.doc:
            description = cell(raw.doc.brief)
            if has_table(raw.doc):
                description += f" [details below](#{anchor(full_name + ' details')})"
                sections.append((full_name, raw.doc))
            else:
                details = "<br>".join(cell(p.lines[0]) for p in raw.doc.details)
                description += f"<br><small>{details}</small>" if details else ""
        else:
            description = "*No description*"
        requires = entry_requires(group, variable, ref)
        out.append(f'| <span id="{anchor(full_name)}"></span>{full_name} | {raw.type} | {flags} | {description} | {requires} |')

    for full_name, doc in sections:
        out += ["", f"#### {full_name} details", "", inline(doc.brief)]
        for part in doc.details:
            out += [""] + [inline(line) for line in part.lines]
    return out


def documented(groups: list[Group]) -> list[Group]:
    """Groups sorted by name, without groups that have no entries (clients never see those)."""
    return sorted((g for g in groups if g.variables), key=lambda g: g.name.lower())


def write_markdown(groups: list[Group], kind: Kind, ref: str) -> str:
    groups = [g for g in documented(groups) if g.kind == kind]
    out = ["## Index", ""]
    letter = None
    for group in groups:
        if group.name[0].lower() != letter:
            letter = group.name[0].lower()
            out += ["", f"### {letter.upper()}"]
        out.append(f"* [{group.name}](#{anchor(group.name)})")
    for group in groups:
        out += write_group(group, ref)
    return "\n".join(out) + "\n"


def json_type(kind: Kind, raw: RawVariable) -> str:
    prefix = kind.upper()
    names = [f"{prefix}_{raw.type.upper()}"]
    names += [f"{prefix}_{name}" for flag, name in (("persistent", "PERSISTENT"), ("read-only", "RONLY")) if flag in raw.flags]
    return ", ".join(names)


def json_desc(doc: Doc | None) -> str:
    if not doc:
        return ""
    return "\n\n".join("\n".join(part.lines) for part in doc.details)


def write_json(groups: list[Group]) -> str:
    """Same structure as the Doxygen based generator, the client reads desc and short_desc."""
    result: dict[str, dict] = {"params": {}, "logs": {}}
    for group in documented(groups):
        result[group.kind + "s"][group.name] = {
            "desc": group.doc.brief if group.doc else "",
            "variables": {
                name: {
                    "core": any("core" in d.variable.flags for d in variable.definitions),
                    "short_desc": variable.first.doc.brief if variable.first.doc else "",
                    "type": json_type(group.kind, variable.first),
                    "desc": json_desc(variable.first.doc),
                }
                for name, variable in group.variables.items()
            },
        }
    return json.dumps(result)


def write_output(groups: list[Group], out: Path, ref: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for kind, file_name in MD_FILES.items():
        (out / file_name).write_text(write_markdown(groups, kind, ref))
    (out / JSON_FILE).write_text(write_json(groups))


# ---------------------------------------------------------------------------
# Command line

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("src", type=Path, help="firmware source directory, e.g. src")
    parser.add_argument("out", type=Path, help="output directory, e.g. docs/api")
    parser.add_argument("--ref", help="git ref used in source links (default: the tag at HEAD, else the commit hash)")
    parser.add_argument("--root", type=Path, default=Path("."), help="firmware repository root (default: .)")
    parser.add_argument("--verbose", action="store_true", help="list every warning, not just the count")
    args = parser.parse_args(argv)

    blocks, diagnostics = scan(args.root, args.src)
    groups, merge_diagnostics = merge(blocks)
    diagnostics += merge_diagnostics

    errors = [d for d in diagnostics if d.severity == "error"]
    warnings = [d for d in diagnostics if d.severity == "warning"]
    for diagnostic in errors + (warnings if args.verbose else []):
        print(diagnostic, file=sys.stderr)
    entries = sum(len(g.variables) for g in groups)
    print(f"{len(groups)} groups, {entries} entries, {len(errors)} errors, "
          f"{len(warnings)} entries without description" + ("" if args.verbose or not warnings else " (--verbose lists them)"),
          file=sys.stderr)
    if errors:
        return 1
    write_output(groups, args.out, args.ref or default_ref(args.root))
    return 0


if __name__ == "__main__":
    sys.exit(main())
