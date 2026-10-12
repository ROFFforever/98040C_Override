import copy
from dataclasses import asdict, dataclass, fields

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout, QHBoxLayout, QLabel,
                               QLineEdit, QListWidget, QMessageBox, QPushButton, QVBoxLayout, QWidget)

from app import settings

FILE = "robot_profiles.json"


@dataclass
class RobotProfile:
    name: str = "New robot"
    length_in: float = 18.0          #front to back
    width_in: float = 18.0           #side to side
    #where the robot's geometric center sits relative to the tracking center (the pose point odom/camera report).
    #e.g. forward = 1.5 means the middle of the robot is 1.5in in front of the tracking center
    center_forward_in: float = 0.0
    center_left_in: float = 0.0
    #auton planner: track width is for swing arcs, the rest only feeds the time estimates.
    #Speed values default to ROBOT_MAIN's set_speeds_lateral() in main.cpp initialize()
    track_width_in: float = 10.0     #center of left wheels to center of right wheels
    slow_vel: float = 40.0           #in/s cruise, in/s^2 accel
    slow_accel: float = 65.0
    normal_vel: float = 60.0
    normal_accel: float = 75.0
    fast_vel: float = 80.0
    fast_accel: float = 85.0
    turn_rate_dps: float = 300.0     #rough average turn speed of a rotate(), deg/s

    def lateral(self, speed):
        """(cruise in/s, accel in/s^2) for a Speed enum name: SLOW / NORMAL / FAST."""
        return {"SLOW": (self.slow_vel, self.slow_accel), "FAST": (self.fast_vel, self.fast_accel)}.get(
            speed, (self.normal_vel, self.normal_accel))


DEFAULT_PROFILES = [RobotProfile("18 x 18 (max size)")]


class ProfileStore:
    def __init__(self):
        data = settings.load(FILE, {})
        known = {f.name for f in fields(RobotProfile)}
        self.profiles = [RobotProfile(**{k: v for k, v in p.items() if k in known}) for p in data.get("profiles", [])]
        if not self.profiles:
            self.profiles = copy.deepcopy(DEFAULT_PROFILES)
        names = [p.name for p in self.profiles]
        self.active_name = data.get("active") if data.get("active") in names else names[0]

    def active(self):
        return next(p for p in self.profiles if p.name == self.active_name)

    def set_active(self, name):
        self.active_name = name
        self.save()

    def replace(self, profiles, active_name):
        self.profiles, self.active_name = profiles, active_name
        self.save()

    def save(self):
        settings.save(FILE, {"active": self.active_name, "profiles": [asdict(p) for p in self.profiles]})


#rows of the editor form: (field, label, min, max, suffix), field None = section header
PROFILE_SPINS = [
    ("length_in", "Length (front-back)", 1, 36, " in"),
    ("width_in", "Width (side-side)", 1, 36, " in"),
    ("center_forward_in", "Center forward of tracking ctr", -18, 18, " in"),
    ("center_left_in", "Center left of tracking ctr", -18, 18, " in"),
    (None, "Auton planner", 0, 0, ""),
    ("track_width_in", "Track width (for swings)", 1, 36, " in"),
    ("slow_vel", "Speed::SLOW cruise", 1, 200, " in/s"),
    ("slow_accel", "Speed::SLOW accel", 1, 1000, " in/s²"),
    ("normal_vel", "Speed::NORMAL cruise", 1, 200, " in/s"),
    ("normal_accel", "Speed::NORMAL accel", 1, 1000, " in/s²"),
    ("fast_vel", "Speed::FAST cruise", 1, 200, " in/s"),
    ("fast_accel", "Speed::FAST accel", 1, 1000, " in/s²"),
    ("turn_rate_dps", "Avg turn rate (time est.)", 10, 1000, " °/s"),
]


class ProfilePreview(QWidget):
    """Top-down sketch of the robot: outline, front edge, tracking center."""

    def __init__(self):
        super().__init__()
        self.profile = None
        self.setMinimumSize(200, 200)

    def set_profile(self, profile):
        self.profile = profile
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#1b1b1b"))
        if self.profile is None:
            return
        pr = self.profile
        #robot faces up in the preview: forward -> screen up, left -> screen left
        span = max(pr.length_in / 2 + abs(pr.center_forward_in), pr.width_in / 2 + abs(pr.center_left_in), 1) * 2.4
        s = min(self.width(), self.height()) / span
        cx, cy = self.width() / 2, self.height() / 2

        def to_px(forward, left):
            return QPointF(cx - left * s, cy - forward * s)

        top_left = to_px(pr.center_forward_in + pr.length_in / 2, pr.center_left_in + pr.width_in / 2)
        bottom_right = to_px(pr.center_forward_in - pr.length_in / 2, pr.center_left_in - pr.width_in / 2)
        body = QRectF(top_left, bottom_right)
        p.setPen(QPen(QColor("#ffb000"), 2))
        p.setBrush(QColor(255, 176, 0, 50))
        p.drawRect(body)
        p.setPen(QPen(QColor("white"), 4))
        p.drawLine(body.topLeft(), body.topRight())  #front edge

        p.setPen(QPen(QColor("#55ff7f"), 2))
        c = to_px(0, 0)
        p.drawLine(c + QPointF(-6, 0), c + QPointF(6, 0))
        p.drawLine(c + QPointF(0, -6), c + QPointF(0, 6))
        p.setPen(QColor("#cccccc"))
        p.drawText(self.rect().adjusted(6, 4, -6, -4), Qt.AlignTop | Qt.AlignHCenter, "front")
        p.drawText(self.rect().adjusted(6, 4, -6, -4), Qt.AlignBottom | Qt.AlignLeft, "+ = tracking center")


class ProfileDialog(QDialog):
    """Add / edit / delete robot profiles. The selected one becomes the active profile on OK."""

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Robot profiles")
        self.resize(640, 600)
        self.profiles = copy.deepcopy(store.profiles)  #edit a copy so Cancel throws changes away
        self._loading = False

        self.list = QListWidget()
        self.list.currentRowChanged.connect(self._show_profile)
        add = QPushButton("Add")
        dup = QPushButton("Duplicate")
        delete = QPushButton("Delete")
        add.clicked.connect(lambda: self._add(RobotProfile()))
        dup.clicked.connect(lambda: self._add(copy.deepcopy(self._current())))
        delete.clicked.connect(self._delete)
        list_buttons = QHBoxLayout()
        for b in (add, dup, delete):
            list_buttons.addWidget(b)
        left = QVBoxLayout()
        left.addWidget(self.list, 1)
        left.addLayout(list_buttons)

        self.name = QLineEdit()
        self.name.textEdited.connect(self._save_fields)
        form = QFormLayout()
        form.addRow("Name", self.name)
        self.spins = {}  #profile field name -> spin box
        for field, label, lo, hi, suffix in PROFILE_SPINS:
            if field is None:  #section header
                form.addRow(QLabel(f"<b>{label}</b>"))
                continue
            self.spins[field] = self._spin(lo, hi, suffix)
            form.addRow(label, self.spins[field])
        self.preview = ProfilePreview()
        right = QVBoxLayout()
        right.addLayout(form)
        right.addWidget(self.preview, 1)

        body = QHBoxLayout()
        body.addLayout(left, 2)
        body.addLayout(right, 3)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        layout = QVBoxLayout(self)
        layout.addLayout(body, 1)
        layout.addWidget(QLabel("The outline is drawn around each point's pose (the tracking center). "
                                "The selected profile is used after OK."))
        layout.addWidget(buttons)

        for p in self.profiles:
            self.list.addItem(p.name)
        self.list.setCurrentRow(next(i for i, p in enumerate(self.profiles) if p.name == store.active_name))

    def _spin(self, lo, hi, suffix):
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(2)
        spin.setSingleStep(0.25 if suffix == " in" else 1)
        spin.setSuffix(suffix)
        spin.valueChanged.connect(self._save_fields)
        return spin

    def _current(self):
        row = self.list.currentRow()
        return self.profiles[row] if 0 <= row < len(self.profiles) else None

    def _show_profile(self, row):
        p = self._current()
        if p is None:
            return
        self._loading = True  #don't write the fields back while filling them in
        self.name.setText(p.name)
        for field, spin in self.spins.items():
            spin.setValue(getattr(p, field))
        self._loading = False
        self.preview.set_profile(p)

    def _save_fields(self, *_):
        p = self._current()
        if self._loading or p is None:
            return
        p.name = self.name.text()
        for field, spin in self.spins.items():
            setattr(p, field, spin.value())
        self.list.currentItem().setText(p.name)
        self.preview.set_profile(p)

    def _add(self, profile):
        profile.name = self._unique_name(profile.name)
        self.profiles.append(profile)
        self.list.addItem(profile.name)
        self.list.setCurrentRow(len(self.profiles) - 1)

    def _delete(self):
        if len(self.profiles) <= 1:
            QMessageBox.information(self, "Robot profiles", "Keep at least one profile.")
            return
        row = self.list.currentRow()
        del self.profiles[row]
        self.list.takeItem(row)

    def _unique_name(self, name, skip=None):
        taken = {p.name for p in self.profiles if p is not skip}
        base, n = name, 2
        while name in taken:
            name = f"{base} ({n})"
            n += 1
        return name

    def _accept(self):
        for p in self.profiles:  #blank or duplicate names would break picking the active profile
            p.name = self._unique_name(p.name.strip() or "Robot", skip=p)
        self.active_name = self._current().name
        self.accept()
