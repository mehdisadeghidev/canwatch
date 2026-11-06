#!/usr/bin/env python3
"""Arduino IDE inserts generated prototypes right before the first function definition.
Any struct used in a function signature must therefore be declared above that line,
otherwise the IDE build fails while a plain g++ compile of the file succeeds.
Run from the sketch folder: python3 test/prototype_order.py
"""
import re
import sys

SRC = "canwatch.ino"
SIG = re.compile(
    r"^((?:static\s+)?(?:inline\s+)?[A-Za-z_][\w:<>*&,\s]*?\s+\**[A-Za-z_]\w*)"
    r"\s*\(([^;{]*)\)\s*\{"
)
HEAD = re.compile(r"^[A-Za-z_][\w:<>*&,\s]*?\s+\**[A-Za-z_]\w*\s*\([^;]*$")  # wrapped signature
WATCHED = ("Entry", "Out", "LogLine", "Raw", "Health", "FrameStats")

with open(SRC, encoding="utf-8") as handle:
    lines = handle.read().split("\n")

start = next(i for i, line in enumerate(lines) if 'R"rawliteral(' in line)
end = next(i for i, line in enumerate(lines) if line.startswith(')rawliteral"'))


def is_code(index):
    return not start < index < end


first_fn = next(
    (i for i, line in enumerate(lines) if is_code(i) and (SIG.match(line) or HEAD.match(line))),
    None,
)
if first_fn is None:
    sys.exit("no function definition found")

signatures = " ".join(
    match.group(2)
    for i, line in enumerate(lines)
    if is_code(i) and i > first_fn and (match := SIG.match(line))
)
used = set(re.findall(r"\b([A-Z]\w+)\b", signatures))

above = "\n".join(lines[:first_fn])
missing = [
    name
    for name in WATCHED
    if name in used and not re.search(rf"^(?:struct|class)\s+{name}\b|^\s*{name};", above, re.M)
]
if missing:
    print(
        f"FAIL: used in a signature but declared below the first function "
        f"(line {first_fn + 1}): {', '.join(missing)}"
    )
    sys.exit(1)
print(
    f"prototype order ok: first function at line {first_fn + 1}, "
    f"signature types declared above it"
)
