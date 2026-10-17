"""Auton planner state: the open plan, selection, undo/redo, placing steps by clicking, autosave.
The panel and field overlay only read from here and call these methods - neither edits the plan directly."""
import math
import time

from PySide6.QtCore import QObject, QTimer, Signal

from app import settings
from app.planner import codegen, parse, store
from app.planner.motion import wrap
from app.planner.parse import ParseError
from app.planner.plan import Group, Plan, Step, identifier
from app.planner.simulate import can_async, simulate_plan
from app.planner.steps import BY_ID, default_params

STATE_FILE = "planner.json"  #remembers the last open plan
UNDO_LIMIT = 200
COALESCE_S = 1.0             #edits to the same field within this long are one undo step
POLL_MS = 1000               #how often the open plan's .cpp is checked for edits made outside the tool
POINT_STEP_TYPES = ("move_to_point", "drive_to_point", "rotate_to_point")


def snap(v, fine, coarse, use_coarse):
    q = coarse if use_coarse else fine
    return round(round(v / q) * q, 3)


class Planner(QObject):
    changed = Signal()          #plan contents, sim or selection changed
    plan_switched = Signal()    #a different plan was opened / renamed / undo restored plan-level fields
    placing_changed = Signal()

    def __init__(self, profile):
        super().__init__()
        self.profile = profile
        self.plan = None
        self.sim = None
        self.code = ("", "")    #(header, cpp) for the current plan
        self.selected = None    #path into the plan, see Plan.get()
        self.placing = None     #step type id waiting for a click on the field
        self._undo, self._redo = [], []
        self._last_key, self._last_time = None, 0.0
        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.setInterval(400)
        self._save_timer.timeout.connect(self.save)
        #The plan is the .cpp on disk: _disk_text is what we last wrote/read, so a different file means it was
        #edited outside the tool (VS Code on the laptop at a match...) and the file wins.
        self._disk_text = None
        self._dirty = False     #in-tool edits not written yet
        self.notice = ""        #problem reading/writing the .cpp, shown in the panel
        self._poll = QTimer(self)
        self._poll.setInterval(POLL_MS)
        self._poll.timeout.connect(self._check_external)
        self._poll.start()

    # ---- plans

    def ensure_plan(self):
        """Open something the first time the planner is shown: last plan, else any plan, else a new one."""
        if self.plan is not None:
            return
        last = settings.load(STATE_FILE, {}).get("last")
        plans = store.list_plans()
        order = ([last] if last in plans else []) + [p for p in plans if p != last]
        if not any(self.open_plan(name) for name in order):
            self.new_plan()

    def open_plan(self, name):
        """-> True if it opened (a .cpp the parser can't read stays closed and sets self.notice)."""
        self.save()
        try:
            plan, warnings, text = store.load_plan(name)
        except (ParseError, ValueError, OSError) as e:
            self.notice = f"Couldn't open {name}: {e}"
            self.changed.emit()
            return False
        legacy = text is None
        self._disk_text = store.read(store.cpp_path(name)) if legacy else text
        self.notice = "\n".join(warnings)
        self._set_plan(plan)
        if legacy:  #first open of an old json plan: write it out as the .cpp
            self._dirty = True
            self.save()
        return True

    def new_plan(self, base="auton"):
        self.save()
        taken = store.taken_names()
        n = 1
        while f"{base}_{n}" in taken:
            n += 1
        self._disk_text, self.notice = None, ""
        self._set_plan(Plan(name=f"{base}_{n}"))
        self._dirty = True
        self.save()

    def duplicate_plan(self):
        self.save()
        copy = self.plan.copy()
        taken = store.taken_names()
        name, n = f"{copy.name}_copy", 2
        while name in taken:
            name, n = f"{copy.name}_copy{n}", n + 1
        copy.name = name
        self._disk_text, self.notice = None, ""
        self._set_plan(copy)
        self._dirty = True
        self.save()

    def delete_plan(self):
        store.delete_plan(self.plan.name)
        self.plan, self._dirty, self._disk_text = None, False, None
        self.ensure_plan()

    def rename(self, new_name):
        """-> error text, or None if it worked."""
        new_id = identifier(new_name)
        if new_id == self.plan.name:
            return None
        if new_id in store.taken_names():
            return f"There's already a file called {new_id}.cpp."
        old = self.plan.name
        self.checkpoint()
        self.plan.name = new_id
        self._disk_text = None
        self._dirty = True
        self.save()
        store.delete_plan(old)
        self._remember()
        self.plan_switched.emit()
        self._commit(save=False)
        return None

    def _set_plan(self, plan):
        self.plan = plan
        self.selected = None
        self.placing = None
        self._undo, self._redo = [], []
        self._last_key = None
        self._remember()
        self.plan_switched.emit()
        self.placing_changed.emit()
        self._commit(save=False)

    def _remember(self):
        settings.save(STATE_FILE, {"last": self.plan.name})

    def save(self):
        """Writes the plan to its .cpp/.h - unless the .cpp was changed outside the tool, then that wins."""
        self._save_timer.stop()
        if self.plan is None or not self._dirty:
            return
        on_disk = store.read(store.cpp_path(self.plan.name))
        if on_disk is not None and on_disk != self._disk_text:
            self._dirty = False
            self._reload(on_disk)
            return
        try:
            self._disk_text = store.save_plan(self.plan)
            self._dirty = False
            if self.notice.startswith("Couldn't save"):
                self.notice = ""
        except OSError as e:
            self.notice = f"Couldn't save: {e}"
            self.changed.emit()

    def _check_external(self):
        if self.plan is None or self._dirty:
            return
        on_disk = store.read(store.cpp_path(self.plan.name))
        if on_disk is not None and on_disk != self._disk_text:
            self._reload(on_disk)

    def _reload(self, text):
        """Re-read the plan from edited .cpp text. Ctrl+Z goes back to what the tool had."""
        try:
            plan, warnings = parse.parse_plan(self.plan.name, text, store.read(store.h_path(self.plan.name)) or "")
        except ParseError as e:  #probably mid-edit: keep the plan, try again when the file changes
            notice = f"Can't read {self.plan.name}.cpp yet: {e}"
            if notice != self.notice:
                self.notice = notice
                self.changed.emit()
            return
        self.checkpoint()
        self.plan = plan
        self._disk_text = text
        self.notice = "\n".join(warnings)
        self.selected = self.selected if self.plan.get(self.selected) is not None else None
        self._last_key = None
        self.plan_switched.emit()
        self._commit(save=False)

    def set_profile(self, profile):
        self.profile = profile
        if self.plan is not None:
            self._commit(save=False)

    # ---- undo

    def checkpoint(self, key=None):
        """Call before changing the plan. Repeated edits with the same key (e.g. typing in one spin box)
        within COALESCE_S share one undo step."""
        now = time.monotonic()
        if key is not None and key == self._last_key and now - self._last_time < COALESCE_S:
            self._last_time = now
            return
        self._undo.append((self.plan.to_dict(), self.selected))
        del self._undo[:-UNDO_LIMIT]
        self._redo.clear()
        self._last_key, self._last_time = key, now

    def undo(self):
        self._swap(self._undo, self._redo)

    def redo(self):
        self._swap(self._redo, self._undo)

    def _swap(self, src, dst):
        if not src:
            return
        dst.append((self.plan.to_dict(), self.selected))
        data, sel = src.pop()
        old_name = self.plan.name
        self.plan = Plan.from_dict(data)
        if self.plan.name != old_name:  #undoing a rename: move the file back too
            store.delete_plan(old_name)
            self._disk_text = None
            self._remember()
        self.selected = sel if self.plan.get(sel) is not None else None
        self._last_key = None
        self.plan_switched.emit()
        self._commit()

    def _commit(self, save=True):
        self.sim = simulate_plan(self.plan, self.profile)
        self.code = codegen.generate(self.plan)
        if save:
            self._dirty = True
            self._save_timer.start()
        self.changed.emit()

    # ---- selection

    def select(self, path):
        if path is not None and self.plan.get(path) is None:
            path = None
        if path != self.selected:
            self.selected = path
            self.changed.emit()

    def select_relative(self, n):
        flat = [p for p, _ in self.plan.flat()]
        if not flat:
            return
        if self.selected in flat:
            i = flat.index(self.selected) + n
        elif self.selected is not None:  #a group: go from its first step
            i = next((k for k, p in enumerate(flat) if p[0] == self.selected[0]), 0) + (n if n < 0 else 0)
        else:
            i = 0 if n > 0 else len(flat) - 1
        self.select(flat[min(max(i, 0), len(flat) - 1)])

    def selected_step(self):
        item = self.plan.get(self.selected)
        return item if isinstance(item, Step) else None

    # ---- adding steps

    def insert_pose(self):
        """Pose a new step would start from: the end of the selection (or of the whole plan)."""
        sims = self.sim.steps
        if self.selected is not None:
            mine = [s for s in sims if s.path[:len(self.selected)] == self.selected]
            if mine:
                return mine[-1].end
        if sims and self.selected is None:
            return sims[-1].end
        return self.plan.start_x, self.plan.start_y, self.plan.start_heading

    def _insert(self, step):
        sel = self.selected
        if sel is None:
            self.plan.items.append(step)
            path = (len(self.plan.items) - 1,)
        elif len(sel) == 2:  #inside a group: stays in the group
            self.plan.items[sel[0]].steps.insert(sel[1] + 1, step)
            path = (sel[0], sel[1] + 1)
        else:
            self.plan.items.insert(sel[0] + 1, step)
            path = (sel[0] + 1,)
        self.selected = path

    def add_step(self, type_id, x=None, y=None, use_coarse=False, backwards=False):
        params = default_params(type_id)
        if x is not None:
            self._place_params(type_id, params, self.insert_pose(), x, y, use_coarse)
            if backwards and "backwards" in params:
                params["backwards"] = True
        self.checkpoint()
        self._insert(Step(type_id, params))
        self._commit()

    def start_placing(self, type_id):
        """Steps with a field handle get placed by clicking; the rest go straight in."""
        if not BY_ID[type_id].handle:
            self.cancel_placing()
            self.add_step(type_id)
            return
        self.placing = type_id
        self.placing_changed.emit()

    def cancel_placing(self):
        if self.placing is not None:
            self.placing = None
            self.placing_changed.emit()
            return True
        return False

    def place_at(self, x, y, use_coarse, backwards):
        type_id = self.placing
        self.add_step(type_id, x, y, use_coarse, backwards)
        if not BY_ID[type_id].repeat_place:
            self.cancel_placing()

    def _place_params(self, type_id, params, pose, x, y, use_coarse):
        """Fill a step's field-placed params from a field point (click or handle drag)."""
        handle = BY_ID[type_id].handle
        if handle == "point":
            params["x"] = snap(x, 0.1, 1.0, use_coarse)
            params["y"] = snap(y, 0.1, 1.0, use_coarse)
        elif handle == "distance":  #project onto the heading line: in front = forward, behind = backwards
            h = math.radians(pose[2])
            d = (x - pose[0]) * math.cos(h) + (y - pose[1]) * math.sin(h)
            params["distance"] = snap(abs(d), 0.1, 1.0, use_coarse)
            params["backwards"] = d < 0
        elif handle == "angle":
            ang = math.degrees(math.atan2(y - pose[1], x - pose[0])) % 360
            params["target"] = snap(ang, 1.0, 15.0, use_coarse) % 360
            if type_id == "swing" and "side" in params and not self.dragging:
                #pick the side that turns toward the click: CCW = lock left, CW = lock right
                params["side"] = "LEFT" if wrap(params["target"] - pose[2]) >= 0 else "RIGHT"

    dragging = False

    # ---- editing the selection

    def set_param(self, path, name, value):
        step = self.plan.get(path)
        if step is None or step.params.get(name) == value:
            return
        self.checkpoint(("param", path, name))
        step.params[name] = value
        self._commit()

    def set_item_field(self, path, name, value):
        """async / note on a step or group."""
        item = self.plan.get(path)
        if item is None or getattr(item, name) == value:
            return
        self.checkpoint(("item", path, name))
        setattr(item, name, value)
        self._commit()

    def set_plan_field(self, name, value):
        if getattr(self.plan, name) == value:
            return
        self.checkpoint(("plan", name))
        setattr(self.plan, name, value)
        self._commit()

    def delete_selected(self):
        sel = self.selected
        if self.plan.get(sel) is None:
            return
        self.checkpoint()
        if len(sel) == 2:
            group = self.plan.items[sel[0]]
            del group.steps[sel[1]]
            if not group.steps:
                del self.plan.items[sel[0]]
                self.selected = (sel[0] - 1,) if sel[0] > 0 else None
            else:
                self.selected = (sel[0], sel[1] - 1) if sel[1] > 0 else (sel[0],)
        else:
            del self.plan.items[sel[0]]
            self.selected = (sel[0] - 1,) if sel[0] > 0 else None
        if self.selected is not None and self.plan.get(self.selected) is None:
            self.selected = None
        self._commit()

    def duplicate_selected(self):
        item = self.plan.get(self.selected)
        if item is None:
            return
        self.checkpoint()
        copy = Plan(items=[item]).copy().items[0]
        sel = self.selected
        if len(sel) == 2:
            self.plan.items[sel[0]].steps.insert(sel[1] + 1, copy)
            self.selected = (sel[0], sel[1] + 1)
        else:
            self.plan.items.insert(sel[0] + 1, copy)
            self.selected = (sel[0] + 1,)
        self._commit()

    def toggle_async(self):
        item = self.plan.get(self.selected)
        if item is None or (isinstance(item, Step) and (len(self.selected) == 2 or not can_async(item))):
            return  #steps inside a sequence can't be async on their own, waits/code can't be at all
        self.set_item_field(self.selected, "is_async", not item.is_async)

    def toggle_group(self):
        """Top-level step -> wrapped in a new Sequence. Step in a sequence, or a sequence -> sequence dissolved."""
        sel = self.selected
        item = self.plan.get(sel)
        if item is None:
            return
        self.checkpoint()
        if len(sel) == 2 or isinstance(item, Group):
            group = self.plan.items[sel[0]]
            self.plan.items[sel[0]:sel[0] + 1] = group.steps
            self.selected = (sel[0] + (sel[1] if len(sel) == 2 else 0),)
        else:
            self.plan.items[sel[0]] = Group([item], item.is_async)
            item.is_async = False
            self.selected = (sel[0], 0)
        self._commit()

    def move_selected(self, direction):
        """Up/down one slot. Steps move into / out of sequences at their edges."""
        sel = self.selected
        item = self.plan.get(sel)
        if item is None:
            return
        items = self.plan.items
        self.checkpoint()
        if len(sel) == 2:
            group = items[sel[0]]
            j = sel[1] + direction
            if 0 <= j < len(group.steps):
                group.steps[sel[1]], group.steps[j] = group.steps[j], group.steps[sel[1]]
                self.selected = (sel[0], j)
            else:  #off the edge: leaves the sequence
                del group.steps[sel[1]]
                at = sel[0] + (1 if direction > 0 else 0)
                items.insert(at, item)
                self.selected = (at,)
                if not group.steps:
                    items.remove(group)
                    self.selected = (items.index(item),)
        else:
            i = sel[0] + direction
            if not 0 <= i < len(items):
                self._undo.pop()
                return
            neighbor = items[i]
            if isinstance(neighbor, Group) and isinstance(item, Step):  #step enters the sequence
                del items[sel[0]]
                gi = items.index(neighbor)
                if direction > 0:
                    neighbor.steps.insert(0, item)
                    self.selected = (gi, 0)
                else:
                    neighbor.steps.append(item)
                    self.selected = (gi, len(neighbor.steps) - 1)
                item.is_async = False
            else:
                items[sel[0]], items[i] = items[i], items[sel[0]]
                self.selected = (i,)
        self._commit()

    # ---- dragging handles on the field

    def begin_drag(self, key):
        self.checkpoint()
        self.dragging = True
        if key[0] in ("point", "distance", "angle"):
            self.select(key[1])
        self.changed.emit()  #outlines switch to the dragged step

    def end_drag(self):
        self.dragging = False
        self.changed.emit()

    def drag(self, key, x, y, use_coarse):
        kind = key[0]
        if kind == "start":
            self.plan.start_x = snap(x, 0.1, 1.0, use_coarse)
            self.plan.start_y = snap(y, 0.1, 1.0, use_coarse)
        elif kind == "start_heading":
            ang = math.degrees(math.atan2(y - self.plan.start_y, x - self.plan.start_x)) % 360
            ang = snap(ang, 1.0, 15.0, use_coarse) % 360
            near = round(ang / 90) * 90  #magnet to 0/90/180/270 so exact cardinals are easy to hit
            self.plan.start_heading = near % 360 if abs(ang - near) <= 4 else ang
        else:
            step = self.plan.get(key[1])
            sim = self.sim.by_path.get(key[1])
            if step is None or sim is None:
                return
            self._place_params(step.type, step.params, sim.start, x, y, use_coarse)
        self._commit()
