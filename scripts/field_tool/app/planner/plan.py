"""The auton plan: start pose + an ordered list of steps, saved as JSON in scripts/field_tool/autons/.
The JSON is the source of truth - the C++ is generated from it (codegen.py), never parsed back."""
import copy
import json
import re
from dataclasses import dataclass, field

from app import field_config as cfg
from app.planner.steps import BY_ID, default_params

STYLES = ("lines", "sequence")  #lines: await()/->schedule() statements, sequence: return new Sequence({...})


@dataclass
class Step:
    type: str
    params: dict
    is_async: bool = False  #lines style: ->schedule() instead of await()
    note: str = ""          #becomes a // comment on the generated line

    @property
    def kind(self):
        return BY_ID[self.type]


@dataclass
class Group:
    """Steps chained into one `new Sequence({...})` inside a lines-style auton."""
    steps: list
    is_async: bool = False
    note: str = ""


@dataclass
class Plan:
    name: str = "new_auton"
    style: str = "lines"
    start_x: float = 0.0
    start_y: float = 0.0
    start_heading: float = 0.0
    set_pose: bool = True        #emit chassis->setPose(start) at the top
    extra_params: str = ""       #e.g. "Lift* lift, piston* claw" for custom code steps
    extra_includes: str = ""     #comma separated headers for the generated .h
    time_limit: float = 15.0     #15 = match auton, 60 = skills
    items: list = field(default_factory=list)  #Step | Group

    # ---- addressing: a path is (item index,) or (item index, step index inside a group)

    def get(self, path):
        if path is None:
            return None
        item = self.items[path[0]] if 0 <= path[0] < len(self.items) else None
        if len(path) == 1 or item is None:
            return item
        return item.steps[path[1]] if isinstance(item, Group) and 0 <= path[1] < len(item.steps) else None

    def flat(self):
        """[(path, Step)] in run order - what the step numbers on the field count."""
        out = []
        for i, item in enumerate(self.items):
            if isinstance(item, Group):
                out += [((i, j), s) for j, s in enumerate(item.steps)]
            else:
                out.append(((i,), item))
        return out

    # ---- json

    def to_dict(self):
        def step(s):
            return {"type": s.type, "params": s.params, "async": s.is_async, "note": s.note}

        items = []
        for it in self.items:
            if isinstance(it, Group):
                items.append({"type": "sequence", "async": it.is_async, "note": it.note,
                              "steps": [step(s) for s in it.steps]})
            else:
                items.append(step(it))
        return {"version": 1, "name": self.name, "style": self.style,
                "start": {"x": self.start_x, "y": self.start_y, "heading": self.start_heading},
                "set_pose": self.set_pose, "extra_params": self.extra_params,
                "extra_includes": self.extra_includes, "time_limit": self.time_limit, "items": items}

    @classmethod
    def from_dict(cls, d):
        def step(s):
            if s.get("type") not in BY_ID:
                raise ValueError(f"unknown step type {s.get('type')!r}")
            params = default_params(s["type"])  #fills in params added since the plan was saved
            params.update({k: v for k, v in s.get("params", {}).items() if k in params})
            return Step(s["type"], params, bool(s.get("async", False)), s.get("note", ""))

        items = []
        for it in d.get("items", []):
            if it.get("type") == "sequence":
                items.append(Group([step(s) for s in it.get("steps", [])], bool(it.get("async", False)),
                                   it.get("note", "")))
            else:
                items.append(step(it))
        start = d.get("start", {})
        return cls(name=d.get("name", "new_auton"), style=d.get("style", "lines") if d.get("style") in STYLES
                   else "lines", start_x=start.get("x", 0.0), start_y=start.get("y", 0.0),
                   start_heading=start.get("heading", 0.0), set_pose=d.get("set_pose", True),
                   extra_params=d.get("extra_params", ""), extra_includes=d.get("extra_includes", ""),
                   time_limit=d.get("time_limit", 15.0), items=items)

    def copy(self):
        return copy.deepcopy(self)


def identifier(name):
    """Plan name -> valid C++ function name (also the file name)."""
    name = re.sub(r"\W", "_", name.strip()) or "auton"
    return "_" + name if name[0].isdigit() else name


def plan_path(name):
    return cfg.PLANS_DIR / f"{identifier(name)}.json"


def list_plans():
    if not cfg.PLANS_DIR.exists():
        return []
    return sorted(p.stem for p in cfg.PLANS_DIR.glob("*.json"))


def load_plan(name):
    return Plan.from_dict(json.loads(plan_path(name).read_text()))


def save_plan(plan):
    cfg.PLANS_DIR.mkdir(parents=True, exist_ok=True)
    plan_path(plan.name).write_text(json.dumps(plan.to_dict(), indent=2) + "\n")
