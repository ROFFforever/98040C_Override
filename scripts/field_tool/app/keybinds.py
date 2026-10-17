from dataclasses import dataclass

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QKeySequence
from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QHBoxLayout, QHeaderView, QKeySequenceEdit, QLabel,
                               QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout)

from app import settings

FILE = "keybinds.json"


@dataclass(frozen=True)
class Bind:
    id: str
    label: str
    default: str
    group: str


#every keyboard shortcut in the tool - add new ones here and they show up in the keybinds menu
BINDS = [
    Bind("open_run", "Open run folder", "Ctrl+O", "General"),
    Bind("keybinds", "Keybinds menu", "Ctrl+K", "General"),
    Bind("profiles", "Robot profiles", "Ctrl+P", "General"),
    Bind("fit", "Fit field to window", "F", "View"),
    Bind("grid", "Toggle tile grid", "G", "View"),
    Bind("axes", "Toggle axes", "A", "View"),
    Bind("zoom_in", "Zoom in", "=", "View"),
    Bind("zoom_out", "Zoom out", "-", "View"),
    Bind("pan_left", "Pan left", "Alt+Left", "View"),
    Bind("pan_right", "Pan right", "Alt+Right", "View"),
    Bind("pan_up", "Pan up", "Alt+Up", "View"),
    Bind("pan_down", "Pan down", "Alt+Down", "View"),
    Bind("robot_trace", "Toggle robot odom path", "R", "Run"),
    Bind("camera_trace", "Toggle camera tag path", "C", "Run"),
    Bind("ghost", "Toggle robot outline", "B", "Run"),
    Bind("motion_mode", "Position / motion data", "M", "Run"),
    Bind("frame_mode", "Time / frame number", "T", "Run"),
    Bind("next", "Next point", "Right", "Points"),
    Bind("prev", "Previous point", "Left", "Points"),
    Bind("next10", "Forward 10 points", "Shift+Right", "Points"),
    Bind("prev10", "Back 10 points", "Shift+Left", "Points"),
    Bind("jump_camera", "Camera point at the same time", "Up", "Points"),
    Bind("jump_robot", "Odom point at the same time", "Down", "Points"),
    Bind("jump_plan", "Planned point at the same time", "Shift+Up", "Points"),
    Bind("deselect", "Deselect / clear playhead", "Esc", "Points"),
    Bind("play", "Play / pause", "Space", "Playback"),
    Bind("start", "Go to start", "Home", "Playback"),
    Bind("end", "Go to end", "End", "Playback"),
    Bind("slower", "Slower", "[", "Playback"),
    Bind("faster", "Faster", "]", "Playback"),
    Bind("align_plan", "Plan starts at the playhead", "Ctrl+Home", "Playback"),
    #auton planner - add_<step id> must match the ids in planner/steps.py
    Bind("planner", "Show auton planner", "P", "Planner"),
    Bind("add_move_to_point", "Move to point", "1", "Planner"),
    Bind("add_drive_to_point", "Drive to point (no turn)", "2", "Planner"),
    Bind("add_move_forward", "Move forward", "3", "Planner"),
    Bind("add_rotate", "Rotate to heading", "4", "Planner"),
    Bind("add_rotate_to_point", "Rotate to face point", "5", "Planner"),
    Bind("add_swing", "Swing", "6", "Planner"),
    Bind("add_wait", "Wait", "7", "Planner"),
    Bind("add_code", "Custom code", "8", "Planner"),
    Bind("add_run_point", "Selected run point → move to point", "W", "Planner"),
    Bind("step_prev", "Previous step", "PgUp", "Planner"),
    Bind("step_next", "Next step", "PgDown", "Planner"),
    Bind("step_up", "Move step up", "Ctrl+Up", "Planner"),
    Bind("step_down", "Move step down", "Ctrl+Down", "Planner"),
    Bind("step_delete", "Delete step", "Del", "Planner"),
    Bind("step_duplicate", "Duplicate step", "Ctrl+D", "Planner"),
    Bind("step_async", "Toggle async (->schedule)", "Shift+A", "Planner"),
    Bind("group", "Wrap in Sequence / unwrap", "Ctrl+G", "Planner"),
    Bind("undo", "Undo", "Ctrl+Z", "Planner"),
    Bind("redo", "Redo", "Ctrl+Y", "Planner"),
    Bind("new_plan", "New plan", "Ctrl+N", "Planner"),
    Bind("dup_plan", "Copy plan", "", "Planner"),
    Bind("delete_plan", "Delete plan", "", "Planner"),
    Bind("copy_code", "Copy generated .cpp", "Ctrl+Shift+C", "Planner"),
    Bind("export", "Export to src/GeneratedAutons", "Ctrl+E", "Planner"),
]
BY_ID = {b.id: b for b in BINDS}

#not rebindable, just listed in the menu so everything is in one place
MOUSE_CONTROLS = [
    ("Left click", "Select the nearest point (planner: a step's end dot selects the step)"),
    ("Scroll / two-finger scroll", "Zoom (keyboard: = and -)"),
    ("Middle / right drag, Ctrl + left drag", "Pan (keyboard: Alt + arrows)"),
    ("Planner: click while placing", "Place the step (Shift = snap 1 in / 15°, Alt = backwards)"),
    ("Planner: drag a handle", "Move a waypoint / start pose, set a distance or angle (Shift = snap)"),
]


class Keybinds:
    def __init__(self):
        saved = settings.load(FILE, {})
        self.overrides = {k: v for k, v in saved.items() if k in BY_ID}  #ignore binds that no longer exist

    def key(self, bind_id):
        return self.overrides.get(bind_id, BY_ID[bind_id].default)

    def set_all(self, mapping):
        """mapping: bind id -> key string. Only keys that differ from the default get saved."""
        self.overrides = {k: v for k, v in mapping.items() if v != BY_ID[k].default}
        settings.save(FILE, self.overrides)


def _portable(seq):
    return seq.toString(QKeySequence.PortableText)


class KeybindDialog(QDialog):
    """Table of every action and its key. Click a key cell and press the new key; conflicts block saving."""

    def __init__(self, keybinds, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Keybinds")
        self.resize(480, 640)
        self._edits = {}  #bind id -> QKeySequenceEdit

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Action", "Key", ""])
        self.table.verticalHeader().hide()
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.Fixed)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.setColumnWidth(1, 150)

        bold = QFont()
        bold.setBold(True)
        group = None
        for bind in BINDS:
            if bind.group != group:  #group header row
                group = bind.group
                row = self.table.rowCount()
                self.table.insertRow(row)
                item = QTableWidgetItem(group)
                item.setFont(bold)
                self.table.setItem(row, 0, item)
                self.table.setSpan(row, 0, 1, 3)
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem("    " + bind.label))
            edit = QKeySequenceEdit(QKeySequence(keybinds.key(bind.id)))
            edit.setMaximumSequenceLength(1)  #single key combos only, no multi-key chords
            edit.setClearButtonEnabled(True)
            edit.keySequenceChanged.connect(self._check_conflicts)
            self.table.setCellWidget(row, 1, edit)
            reset = QPushButton("↺")
            reset.setToolTip(f"Reset to default ({bind.default})")
            reset.setFixedWidth(28)
            reset.clicked.connect(lambda _=False, e=edit, d=bind.default: e.setKeySequence(QKeySequence(d)))
            self.table.setCellWidget(row, 2, reset)
            self._edits[bind.id] = edit

        mouse = QLabel("<b>Mouse</b> (fixed)<br>" + "<br>".join(f"{k}: {v}" for k, v in MOUSE_CONTROLS))
        self.conflict_label = QLabel()
        self.conflict_label.setStyleSheet("color: #ff6060;")
        self.conflict_label.setWordWrap(True)

        reset_all = QPushButton("Reset all")
        reset_all.clicked.connect(self._reset_all)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        bottom = QHBoxLayout()
        bottom.addWidget(reset_all)
        bottom.addStretch(1)
        bottom.addWidget(self.buttons)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Click a key, then press the new combo. The ✕ clears it (no key)."))
        layout.addWidget(self.table, 1)
        layout.addWidget(mouse)
        layout.addWidget(self.conflict_label)
        layout.addLayout(bottom)
        self._check_conflicts()

    def _reset_all(self):
        for bind_id, edit in self._edits.items():
            edit.setKeySequence(QKeySequence(BY_ID[bind_id].default))

    def _check_conflicts(self):
        used = {}
        for bind_id, edit in self._edits.items():
            key = _portable(edit.keySequence())
            if key:
                used.setdefault(key, []).append(bind_id)
        clashes = {k: ids for k, ids in used.items() if len(ids) > 1}
        clashing_ids = {i for ids in clashes.values() for i in ids}
        for bind_id, edit in self._edits.items():
            edit.setStyleSheet("background: #5a1e1e;" if bind_id in clashing_ids else "")
        self.conflict_label.setText("\n".join(
            f"{k} is used by: " + ", ".join(BY_ID[i].label for i in ids) for k, ids in clashes.items()))
        self.buttons.button(QDialogButtonBox.Ok).setEnabled(not clashes)

    def result_mapping(self):
        return {bind_id: _portable(edit.keySequence()) for bind_id, edit in self._edits.items()}
