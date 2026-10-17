"""Where plans live: the exported C++ (src/GeneratedAutons/<name>.cpp + include/GeneratedAutons/<name>.h).
The tool reads the .cpp back with parse.py, so editing it by hand (or on the robot laptop) edits the plan.
Plans that only exist as scripts/field_tool/autons/<name>.json (the old format) are loaded from there once and
become .cpp files on their first save. Files this tool didn't write (RedHard.cpp...) are never listed or touched."""
import json

from app import field_config as cfg
from app.planner import codegen, parse
from app.planner.plan import Plan, identifier


def cpp_path(name):
    return cfg.EXPORT_SRC_DIR / f"{identifier(name)}.cpp"


def h_path(name):
    return cfg.EXPORT_INC_DIR / f"{identifier(name)}.h"


def _json_path(name):
    return cfg.PLANS_DIR / f"{identifier(name)}.json"


def read(path):
    """Text of a file, or None. Bytes that aren't utf-8 survive a read/write round trip."""
    try:
        return path.read_text(encoding="utf-8", errors="surrogateescape")
    except OSError:
        return None


def _write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", errors="surrogateescape", newline="\n")


def list_plans():
    names = set()
    if cfg.EXPORT_SRC_DIR.exists():
        names |= {p.stem for p in cfg.EXPORT_SRC_DIR.glob("*.cpp") if codegen.is_plan_file(p)}
    if cfg.PLANS_DIR.exists():
        names |= {p.stem for p in cfg.PLANS_DIR.glob("*.json")}
    return sorted(names)


def taken_names():
    """Plan names plus any other .cpp in GeneratedAutons (hand-written ones must not be overwritten)."""
    others = {p.stem for p in cfg.EXPORT_SRC_DIR.glob("*.cpp")} if cfg.EXPORT_SRC_DIR.exists() else set()
    return set(list_plans()) | others


def load_plan(name):
    """-> (Plan, warnings, cpp text it was read from or None for a legacy json)."""
    cpp, js = cpp_path(name), _json_path(name)
    if js.exists() and (not cpp.exists() or (codegen.is_legacy(cpp) and not codegen.hand_edited(cpp))):
        return Plan.from_dict(json.loads(js.read_text())), [], None  #old format, cpp is derived from it
    text = read(cpp)
    if text is None:
        raise parse.ParseError(f"{cpp.name} is missing")
    plan, warnings = parse.parse_plan(identifier(name), text, read(h_path(name)) or "")
    return plan, warnings, text


def save_plan(plan):
    """Writes the .h and .cpp (only the ones that changed). -> the cpp text now on disk."""
    cpp = cpp_path(plan.name)
    if cpp.exists() and not codegen.is_plan_file(cpp):
        raise OSError(f"{cpp.name} exists and wasn't made by the field tool - not overwriting it")
    header, cpp_text = codegen.generate(plan)
    for path, text in ((h_path(plan.name), header), (cpp, cpp_text)):
        if read(path) != text:
            _write(path, text)
    return cpp_text


def delete_plan(name):
    """The .cpp/.h are the plan, so they go with it (and the old json, if there is one)."""
    for path in (cpp_path(name), h_path(name), _json_path(name)):
        if path.exists():
            path.unlink()
