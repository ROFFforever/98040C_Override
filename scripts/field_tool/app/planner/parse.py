"""C++ -> Plan: reads back what codegen.py writes, including hand tweaks of it (change a number, add a line).
Anything it doesn't recognise is kept as a Custom code step and written back out as-is, so nothing is lost.
Recognised: setPose / the wall-sensor start, chassis->moveToPoint(...) etc. (await or ->schedule()),
pros::delay, new Sequence({...}) groups, and `// note` comments on a step's line."""
import re

from app.planner.plan import Group, Plan, Step
from app.planner.steps import BY_ID, STEP_TYPES

BY_CALL = {s.call: s for s in STEP_TYPES if s.call}
DEFAULT_TIME_LIMIT = 15.0
#includes codegen adds by itself, everything else in a file goes into the plan's extra includes
BASE_INCLUDES = {"Subsystems/drivetrain.h", "CommandScheduler/commandScheduler.h", "util/mathUtils.h",
                 "Commands/WaitCommand.h", "Commands/Localization/WallLocalization.h"} \
    | {inc for s in STEP_TYPES for inc in s.includes}

_TOKEN = re.compile(r'''(?P<cm>//[^\n]*|/\*.*?\*/)|(?P<str>"(?:\\.|[^"\\\n])*"|'(?:\\.|[^'\\\n])*')|(?P<ws>\s+)|(?P<ch>.)''',
                    re.S)
_BLOCK_START = re.compile(r"^(if|for|while|switch|else|do)\b|^\{")


class ParseError(ValueError):
    pass


def _lex(text):
    """[(kind, text)] with kind cm (comment), ws, or ch (a character, or a whole string literal)."""
    out = []
    for m in _TOKEN.finditer(text):
        kind = m.lastgroup
        out.append(("ch" if kind == "str" else kind, m.group()))
    return out


def _comment_text(raw):
    raw = raw.strip()
    if raw.startswith("//"):
        return raw[2:].strip()
    return raw[2:-2].strip() if raw.startswith("/*") else raw


def _squash(s):
    return re.sub(r"\s+", " ", s).strip()


def _else_follows(tokens, i):
    rest = [t for k, t in tokens[i + 1:i + 40] if k != "ws" and k != "cm"][:4]
    return "".join(rest) == "else"


def _split(tokens, mode):
    """Cuts tokens into top-level pieces. mode 'stmt': statements ending in ';' (or a closing '}' of an if/for
    block). mode 'list': comma separated elements. Each piece: raw text, code (no comments, whitespace squashed),
    lead (own-line comments above it), trail (comment after it on its line), indent (of its first line)."""
    out, lead = [], []
    cur = None
    depth = 0
    newline = True      #a newline came since the last piece ended
    prev_ws = ""

    def finish():
        nonlocal cur, newline
        cur["raw"] = "".join(cur.pop("toks")).strip()
        cur["code"] = _squash("".join(cur.pop("code")))
        out.append(cur)
        cur = None
        newline = False

    for i, (kind, text) in enumerate(tokens):
        if cur is None:
            if kind == "ws":
                prev_ws = text
                newline = newline or "\n" in text
                continue
            if kind == "cm":
                if out and not newline and not out[-1]["trail"]:
                    out[-1]["trail"] = _comment_text(text)
                else:
                    lead.append(_comment_text(text))
                prev_ws = ""
                continue
            if mode == "list" and text == ",":
                continue
            cur = {"toks": [], "code": [], "lead": lead, "trail": "", "indent": prev_ws.rsplit("\n", 1)[-1]}
            lead, depth, prev_ws = [], 0, ""
        if kind == "ws":
            cur["toks"].append(text)
            cur["code"].append(" ")
            continue
        if kind == "cm":
            cur["toks"].append(text)
            if mode == "list" and depth == 0:
                cur["trail"] = _comment_text(text)
            continue
        if mode == "list" and depth == 0 and text == ",":
            finish()
            continue
        cur["toks"].append(text)
        cur["code"].append(text)
        if text in "([{":
            depth += 1
        elif text in ")]}":
            depth -= 1
        if mode == "stmt" and depth == 0 and (
                text == ";" or (text == "}" and _BLOCK_START.match(_squash("".join(cur["code"])))
                                and not _else_follows(tokens, i))):
            finish()
    if cur is not None:
        if "".join(cur["code"]).strip():
            finish()
        else:
            cur = None
    for text in lead:   #comments left over at the end
        out.append({"raw": "", "code": "", "lead": [text], "trail": "", "indent": ""})
    return out


def _dedent(raw, indent):
    lines = raw.splitlines()
    return "\n".join(lines[:1] + [ln[len(indent):] if indent and ln.startswith(indent) else ln for ln in lines[1:]])


def _function(text):
    """-> (return type, name, params text, body tokens)."""
    m = re.search(r"(?m)^(void|Sequence\s*\*)\s*(\w+)\s*\(([^)]*)\)\s*\{", text)
    if m is None:
        raise ParseError("couldn't find the auton function (void name(drivetrain* chassis){ ... })")
    tokens, depth, body = _lex(text[m.end():]), 1, []
    for kind, tok in tokens:
        if kind == "ch":
            depth += (tok in "([{") - (tok in ")]}")
            if depth == 0:
                break
        body.append((kind, tok))
    else:
        raise ParseError("missing the closing } of the function")
    return re.sub(r"\s+", "", m.group(1)), m.group(2), m.group(3), body


# ---- calls -> steps

def _split_args(s):
    args, depth, cur = [], 0, []
    for kind, tok in _lex(s):
        if kind == "ch" and depth == 0 and tok == ",":
            args.append("".join(cur).strip())
            cur = []
            continue
        if kind == "ch":
            depth += (tok in "([{") - (tok in ")]}")
        cur.append(tok)
    if "".join(cur).strip() or args:
        args.append("".join(cur).strip())
    return args


def _balanced(s):
    depth = 0
    for kind, tok in _lex(s):
        if kind == "ch":
            depth += (tok in "([{") - (tok in ")]}")
            if depth < 0:
                return False
    return depth == 0


def _float(s):
    return float(re.sub(r"(?<=\d)[fF]$", "", s.strip()))


def _param_value(prm, s):
    s = s.strip()
    if prm.kind == "bool":
        if s not in ("true", "false"):
            raise ValueError(s)
        return s == "true"
    if prm.kind == "choice":
        value = s.rsplit("::", 1)[-1]
        if value not in prm.choices or ("::" in s and s.rsplit("::", 1)[0] != prm.cpp_enum):
            raise ValueError(s)
        return value
    if prm.kind == "auto":
        return None if s == prm.cpp_auto else _float(s)
    if prm.kind == "int":
        return int(round(_float(s)))
    return _float(s)


def _step_from_call(code, note="", is_async=False):
    """'chassis->moveToPoint(1, 2)' -> Step, or None if it isn't a call the planner knows (or has odd arguments)."""
    m = re.fullmatch(r"(\w+->\w+)\s*\((.*)\)", code)
    if m is None or m.group(1) not in BY_CALL or not _balanced(m.group(2)):
        return None
    kind = BY_CALL[m.group(1)]
    args = _split_args(m.group(2))
    if len(args) > len(kind.params):
        return None
    params = {prm.name: prm.default for prm in kind.params}
    try:
        for prm, arg in zip(kind.params, args):
            params[prm.name] = _param_value(prm, arg)
    except ValueError:
        return None
    if len(args) < sum(1 for prm in kind.params if not prm.optional):
        return None
    return Step(kind.id, params, is_async, note)


def _wait(ms, note=""):
    return Step("wait", {"ms": int(ms)}, False, note)


def _code_step(text, note=""):
    return Step("code", {"text": text}, False, note)


def _element_step(el):
    """One element of a new Sequence({...}) list."""
    note = "; ".join(el["lead"] + ([el["trail"]] if el["trail"] else []))
    m = re.fullmatch(r"new WaitCommand\(\s*(\d+)\s*\)", el["code"])
    if m:
        return _wait(m.group(1), note)
    step = _step_from_call(el["code"], note)
    return step or _code_step(el["code"], note)


def _sequence_elements(raw):
    """'new Sequence({ ... })' found in raw -> (comment on its first line, pieces inside), or None."""
    marker = "new Sequence({"
    at = raw.find(marker)
    if at < 0:
        return None
    inner, depth = [], 1
    for kind, tok in _lex(raw[at + len(marker):]):
        if kind == "ch":
            depth += (tok in "([{") - (tok in ")]}")
            if depth == 0:
                break
        inner.append((kind, tok))
    else:
        return None
    note, i = "", 0
    while i < len(inner) and inner[i][0] == "ws" and "\n" not in inner[i][1]:
        i += 1
    if i < len(inner) and inner[i][0] == "cm":  #the group's own note sits after the opening {
        note = _comment_text(inner[i][1])
        inner = inner[i + 1:]
    return note, _split(inner, "list")


def _group_from(st, inner_code, is_async):
    """inner_code is the squashed 'new Sequence({...})' text; -> Group or None."""
    if not re.fullmatch(r"new Sequence\(\{.*\}\)", inner_code):
        return None
    found = _sequence_elements(st["raw"])
    if found is None:
        return None
    note, elements = found
    return Group([_element_step(el) for el in elements if el["code"] or el["lead"]], is_async,
                 note or st["trail"])


def _convert(st):
    """One statement -> Step | Group, or None for 'keep as code'."""
    code, note = st["code"], st["trail"]
    m = re.fullmatch(r"pros::delay\(\s*(\d+)\s*\);", code)
    if m:
        return _wait(m.group(1), note)
    m = re.fullmatch(r"await\((.*)\);", code)
    if m and _balanced(m.group(1)):
        return _group_from(st, m.group(1), False) or _step_from_call(m.group(1), note)
    m = re.fullmatch(r"(.*)->schedule\(\s*\);", code)
    if m and _balanced(m.group(1)):
        inner = m.group(1)
        if inner.startswith("(") and inner.endswith(")") and _balanced(inner[1:-1]):
            return _group_from(st, inner[1:-1].strip(), True)
        return _step_from_call(inner, note, True)
    return None


def _convert_all(statements):
    """Statements -> plan items, with unrecognised ones (and stray comments) merged into code steps."""
    items = []

    def add_code(lines):
        if items and isinstance(items[-1], Step) and items[-1].type == "code" and not items[-1].note:
            items[-1].params["text"] += "\n" + "\n".join(lines)
        else:
            items.append(_code_step("\n".join(lines)))

    for st in statements:
        if st["lead"]:
            add_code([f"// {c}" for c in st["lead"]])
        if not st["code"]:
            continue
        item = _convert(st)
        if item is not None:
            items.append(item)
            continue
        raw = _dedent(st["raw"], st["indent"])
        add_code([raw + (f" // {st['trail']}" if st["trail"] else "")])
    return items


# ---- start pose

_POSE = r"chassis->setPose\(([^,]+),([^,]+),degToRad\(([^()]+)\)\);"
_WALL_POSE = re.compile(r"if\(!\w+->set_initial_pose\(([^,]+),WallLocalization::Quadrant::\w+\)\)\{" + _POSE + r"\}")
_PLAIN_POSE = re.compile(_POSE)


def _start_pose(st):
    """-> (x, y, heading, wall_pose) when the statement sets the start pose, else None."""
    compact = re.sub(r"\s+", "", st["code"])
    for pattern, wall in ((_WALL_POSE, True), (_PLAIN_POSE, False)):
        m = pattern.fullmatch(compact)
        if m:
            g = m.groups()[-3:]
            try:  #with the wall start, its heading argument is the one the robot uses
                return _float(g[0]), _float(g[1]), _float(m.group(1) if wall else g[2]), wall
            except ValueError:
                return None
    return None


# ---- whole file

def _extra_params(params):
    keep = [p.strip() for p in params.split(",") if p.strip()]
    keep = [p for p in keep if not re.fullmatch(r"WallLocalization\s*\*\s*\w+", p)]
    if keep and re.fullmatch(r"drivetrain\s*\*\s*\w+", keep[0]):
        keep = keep[1:]
    return ", ".join(keep)


def _includes(text):
    return re.findall(r'(?m)^\s*#include\s*"([^"]+)"', text)


def parse_plan(name, cpp_text, h_text=""):
    """-> (Plan, warnings). Raises ParseError when the function can't be found at all."""
    ret, _fn, params, body = _function(cpp_text)
    warnings = []
    plan = Plan(name=name, style="sequence" if ret.startswith("Sequence") else "lines", set_pose=False)
    plan.extra_params = _extra_params(params)

    first = cpp_text.split("\n", 1)[0]
    m = re.search(r"time_limit=([\d.]+)", first)
    plan.time_limit = float(m.group(1)) if m else DEFAULT_TIME_LIMIT
    m = re.search(r"start=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)", first)
    if m:
        plan.start_x, plan.start_y, plan.start_heading = (float(g) for g in m.groups())

    extras = []
    for inc in _includes(h_text) + _includes(cpp_text):
        if inc not in BASE_INCLUDES and inc != f"GeneratedAutons/{name}.h" and inc not in extras:
            extras.append(inc)
    plan.extra_includes = ", ".join(extras)

    statements = _split(body, "stmt")
    pose_at = next((i for i, st in enumerate(statements) if st["code"] and _start_pose(st)), None)
    if pose_at is not None:
        plan.start_x, plan.start_y, plan.start_heading, plan.wall_pose = _start_pose(statements[pose_at])
        plan.set_pose = True
        pose_stmt = statements.pop(pose_at)
        if pose_stmt["lead"]:  #keep the comments that were above it
            statements.insert(pose_at, {"raw": "", "code": "", "lead": pose_stmt["lead"], "trail": "", "indent": ""})

    if plan.style == "sequence":
        ret_at = next((i for i, st in enumerate(statements) if st["code"].startswith("return new Sequence({")), None)
        if ret_at is None:
            raise ParseError("sequence style needs 'return new Sequence({ ... });'")
        st = statements[ret_at]
        elements = (_sequence_elements(st["raw"]) or ("", []))[1]
        plan.items = [_element_step(el) for el in elements if el["code"] or el["lead"]]
        for i, other in enumerate(statements):
            if i != ret_at and (other["code"] or other["lead"]):
                warnings.append(f"ignored a line outside the return in sequence style: {other['code'] or other['lead'][0]}")
    else:
        plan.items = _convert_all(statements)
    return plan, warnings
