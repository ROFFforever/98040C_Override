"""Side panel for the auton planner: plan settings, step list, the selected step's params, warnings, code preview."""
from PySide6.QtCore import QEvent, QObject, Qt, QTimer
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
                               QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QPlainTextEdit, QScrollArea,
                               QSizePolicy, QSpinBox, QSplitter, QTabWidget, QToolButton, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout, QWidget)

from app.planner.codegen import export_paths
from app.planner.plan import Group, list_plans
from app.planner.simulate import can_async
from app.planner.steps import STEP_TYPES, num

STYLE_LABELS = {"lines": "Lines: await() / ->schedule()", "sequence": "Sequence: return new Sequence({...})"}
LIMITS = [(15.0, "Match auton (15 s)"), (60.0, "Skills (60 s)")]


class _WheelGuard(QObject):
    """Spin boxes / combos ignore the scroll wheel unless focused, so scrolling the panel doesn't edit values."""

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Wheel and not obj.hasFocus():
            event.ignore()
            return True
        return False


def _tool(action, text=None):
    """Button for an action. With `text` it shows that short label instead of the action's full name
    (setDefaultAction would keep resetting it), but still follows the action's tooltip / enabled state."""
    b = QToolButton()
    b.setToolButtonStyle(Qt.ToolButtonTextOnly)
    if not text:
        b.setDefaultAction(action)
        return b
    b.setText(text)
    b.clicked.connect(action.trigger)

    def sync():
        b.setToolTip(action.toolTip())
        b.setEnabled(action.isEnabled())

    action.changed.connect(sync)
    sync()
    return b


class PlanPanel(QWidget):
    def __init__(self, planner, actions):
        super().__init__()
        self.planner = planner
        self.actions = actions
        self._guard = _WheelGuard(self)
        self._loading = False         #true while the code fills widgets in (don't echo back into the plan)
        self._editor_for = None       #(path, step type) the param editor was built for
        self._param_widgets = {}      #param name -> (param, widget)

        #three panes you can resize: steps (list + settings) / selected step's params / generated code.
        #The param editor gets its own pane so it's always on screen, not scrolled off below the list.
        steps = QWidget()
        steps_layout = QVBoxLayout(steps)
        steps_layout.setContentsMargins(6, 0, 6, 0)
        self.settings_toggle = QToolButton()
        self.settings_toggle.setCheckable(True)
        self.settings_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.settings_toggle.setStyleSheet("QToolButton { border: none; font-weight: bold; }")
        self.settings_toggle.toggled.connect(self._show_settings)
        steps_layout.addWidget(self.settings_toggle)
        self.settings_box = self._build_settings()
        steps_layout.addWidget(self.settings_box)
        steps_layout.addWidget(self._build_add_buttons())
        steps_layout.addWidget(self._build_step_list(), 1)
        steps_layout.addLayout(self._build_list_buttons())
        self._show_settings(False)

        editor = QWidget()
        editor_layout = QVBoxLayout(editor)
        editor_layout.setContentsMargins(6, 0, 6, 0)
        self.editor_box = QGroupBox("Step")
        self.editor_form = QFormLayout(self.editor_box)
        editor_layout.addWidget(self.editor_box)
        self.warn_label = QLabel()
        self.warn_label.setWordWrap(True)
        self.warn_label.setStyleSheet("color: #c05a00;")
        editor_layout.addWidget(self.warn_label)
        self.total_label = QLabel()
        editor_layout.addWidget(self.total_label)
        editor_layout.addStretch(1)

        splitter = QSplitter(Qt.Vertical)
        splitter.setChildrenCollapsible(False)
        for pane in (steps, editor):
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(pane)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            scroll.setFrameShape(QScrollArea.NoFrame)
            splitter.addWidget(scroll)
        splitter.addWidget(self._build_code())
        splitter.setSizes([420, 300, 180])
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 6, 0, 0)
        layout.addLayout(self._build_plan_row())
        layout.addWidget(splitter, 1)

        planner.changed.connect(self.refresh)
        planner.plan_switched.connect(self._load_plan_fields)

    # ---- building

    def _guarded(self, w):
        w.installEventFilter(self._guard)
        w.setFocusPolicy(Qt.StrongFocus)
        if isinstance(w, QComboBox):  #don't let long item text force the panel wider
            w.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
            w.setMinimumContentsLength(10)
        return w

    def _show_settings(self, on):
        self.settings_box.setVisible(on)
        self.settings_toggle.setArrowType(Qt.DownArrow if on else Qt.RightArrow)
        self._update_settings_summary()

    def _update_settings_summary(self):
        """Collapsed Function header still says what matters: name, style, start pose."""
        plan = self.planner.plan
        if plan is None:
            self.settings_toggle.setText("Function")
            return
        style = "Lines" if plan.style == "lines" else "Sequence"
        self.settings_toggle.setText(f"Function: {plan.name} · {style} · start ({num(plan.start_x)}, "
                                     f"{num(plan.start_y)}, {num(plan.start_heading)}°)")

    def _build_plan_row(self):
        row = QHBoxLayout()
        row.setContentsMargins(6, 0, 6, 0)
        row.addWidget(QLabel("Plan:"))
        self.plan_combo = QComboBox()
        self.plan_combo.setFocusPolicy(Qt.NoFocus)
        self.plan_combo.setMinimumWidth(80)
        self.plan_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.plan_combo.activated.connect(lambda i: self.planner.open_plan(self.plan_combo.itemText(i)))
        row.addWidget(self.plan_combo, 1)
        for bind_id, text in (("new_plan", "New"), ("dup_plan", "Copy"), ("delete_plan", "Delete")):
            row.addWidget(_tool(self.actions[bind_id], text))
        return row

    def _build_settings(self):
        box = QGroupBox()
        form = QFormLayout(box)
        self.name_edit = QLineEdit()
        self.name_edit.setToolTip("C++ function name, also the plan's file name")
        self.name_edit.editingFinished.connect(self._rename)
        form.addRow("Name", self.name_edit)

        self.style_combo = self._guarded(QComboBox())
        for key, label in STYLE_LABELS.items():
            self.style_combo.addItem(label, key)
        self.style_combo.setToolTip("Lines: one statement per step, each awaited or async - add your own lines between.\n"
                                    "Sequence: the whole auton is one Sequence* (like Autons/SoloAWP.cpp).")
        self.style_combo.currentIndexChanged.connect(
            lambda: self._plan_field("style", self.style_combo.currentData()))
        form.addRow("Style", self.style_combo)

        self.limit_combo = self._guarded(QComboBox())
        for v, label in LIMITS:
            self.limit_combo.addItem(label, v)
        self.limit_combo.currentIndexChanged.connect(
            lambda: self._plan_field("time_limit", self.limit_combo.currentData()))
        form.addRow("Time limit", self.limit_combo)

        start = QHBoxLayout()
        self.start_spins = {}
        for name, lo, hi, suffix in (("start_x", -72, 72, " in"), ("start_y", -72, 72, " in"),
                                     ("start_heading", 0, 359.9, "°")):
            spin = self._guarded(QDoubleSpinBox())
            spin.setRange(lo, hi)
            spin.setDecimals(1)
            spin.setSuffix(suffix)
            if name == "start_heading":
                spin.setWrapping(True)
            spin.setMinimumWidth(60)  #three in a row have to fit the panel
            spin.valueChanged.connect(lambda v, n=name: self._plan_field(n, round(v, 3)))
            self.start_spins[name] = spin
            start.addWidget(spin)
        form.addRow("Start x y θ", start)
        self.set_pose_check = QCheckBox("chassis->setPose(start) at the top")
        self.set_pose_check.toggled.connect(lambda on: self._plan_field("set_pose", on))
        form.addRow("", self.set_pose_check)

        self.params_edit = QLineEdit()
        self.params_edit.setPlaceholderText("e.g. Lift* lift, piston* claw")
        self.params_edit.setToolTip("Extra function parameters (after drivetrain* chassis), for custom code steps")
        self.params_edit.editingFinished.connect(lambda: self._plan_field("extra_params", self.params_edit.text()))
        form.addRow("Extra params", self.params_edit)
        self.includes_edit = QLineEdit()
        self.includes_edit.setPlaceholderText("e.g. Subsystems/Lift.h, Commands/LiftMoveToCommand.h")
        self.includes_edit.setToolTip("Comma separated headers added to the generated .h")
        self.includes_edit.editingFinished.connect(
            lambda: self._plan_field("extra_includes", self.includes_edit.text()))
        form.addRow("Extra includes", self.includes_edit)
        return box

    def _build_add_buttons(self):
        box = QGroupBox("Add step (after the selected one)")
        grid = QGridLayout(box)
        grid.setSpacing(3)
        ids = [s.id for s in STEP_TYPES] + ["run_point"]
        for k, type_id in enumerate(ids):
            b = _tool(self.actions[f"add_{type_id}"])
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            grid.addWidget(b, k // 2, k % 2)
        return box

    def _build_step_list(self):
        self.tree = QTreeWidget()
        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(["#", "Step", "t ≈"])
        self.tree.setFocusPolicy(Qt.NoFocus)  #keys stay with the window shortcuts
        self.tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tree.setRootIsDecorated(True)
        self.tree.setMinimumHeight(120)
        header = self.tree.header()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setStretchLastSection(False)
        self.tree.itemSelectionChanged.connect(self._on_tree_select)
        return self.tree

    def _build_list_buttons(self):
        row = QHBoxLayout()
        row.setSpacing(2)
        for bind_id, text in (("step_up", "↑"), ("step_down", "↓"), ("step_duplicate", "Dup"),
                              ("step_delete", "Del"), ("step_async", "Async"), ("group", "Sequence"),
                              ("undo", "Undo"), ("redo", "Redo")):
            row.addWidget(_tool(self.actions[bind_id], text))
        row.addStretch(1)
        return row

    def _build_code(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(6, 0, 6, 6)
        self.code_tabs = QTabWidget()
        mono = QFont("Consolas", 9)
        self.cpp_view, self.h_view = QPlainTextEdit(), QPlainTextEdit()
        for view, label in ((self.cpp_view, ".cpp"), (self.h_view, ".h")):
            view.setReadOnly(True)
            view.setFont(mono)
            view.setLineWrapMode(QPlainTextEdit.NoWrap)
            self.code_tabs.addTab(view, label)
        layout.addWidget(self.code_tabs, 1)
        row = QHBoxLayout()
        self.export_label = QLabel()
        self.export_label.setStyleSheet("color: #9a9a9a;")
        row.addWidget(self.export_label, 1)
        row.addWidget(_tool(self.actions["copy_code"], "Copy .cpp"))
        row.addWidget(_tool(self.actions["export"], "Export"))
        layout.addLayout(row)
        return w

    # ---- plan-level fields

    def _plan_field(self, name, value):
        if not self._loading:
            self.planner.set_plan_field(name, value)

    def _rename(self):
        if self._loading or self.planner.plan is None:
            return
        err = self.planner.rename(self.name_edit.text())
        if err:
            self.warn_label.setText("⚠ " + err)
            self._load_plan_fields()

    def _load_plan_fields(self):
        plan = self.planner.plan
        if plan is None:
            return
        self._loading = True
        self.plan_combo.clear()
        names = list_plans()
        if plan.name not in names:
            names = sorted(names + [plan.name])
        self.plan_combo.addItems(names)
        self.plan_combo.setCurrentText(plan.name)
        self.name_edit.setText(plan.name)
        self.style_combo.setCurrentIndex(self.style_combo.findData(plan.style))
        i = self.limit_combo.findData(plan.time_limit)
        if i < 0:
            self.limit_combo.addItem(f"{plan.time_limit:g} s", plan.time_limit)
            i = self.limit_combo.count() - 1
        self.limit_combo.setCurrentIndex(i)
        self.set_pose_check.setChecked(plan.set_pose)
        self.params_edit.setText(plan.extra_params)
        self.includes_edit.setText(plan.extra_includes)
        self._loading = False
        self._editor_for = None  #force the step editor to rebuild

    # ---- refresh from the planner

    def refresh(self):
        planner, plan, sim = self.planner, self.planner.plan, self.planner.sim
        if plan is None:
            return
        self._loading = True
        for name, spin in self.start_spins.items():
            if not spin.hasFocus():
                spin.setValue(getattr(plan, name))
        self._fill_tree()
        self._loading = False
        self._refresh_editor()
        self._update_settings_summary()

        warn = [text for _, text in sim.warnings]
        sel_sim = sim.by_path.get(planner.selected)
        if sel_sim is not None:
            warn += [f"step {sel_sim.number}: {n}" for n in sel_sim.notes if "past the wall" not in n]
        self.warn_label.setText("\n".join("⚠ " + w for w in warn))
        self.warn_label.setVisible(bool(warn))
        over = sim.total > plan.time_limit
        self.total_label.setText(f"<b>≈ {sim.total:.1f} s</b> of {plan.time_limit:g} s"
                                 + ("  <span style='color:#ff6060'>over</span>" if over else "")
                                 + "  <span style='color:#9a9a9a'>(rough estimate)</span>")

        header, cpp = planner.code
        for view, text in ((self.cpp_view, cpp), (self.h_view, header)):
            if view.toPlainText() != text:
                bar = view.verticalScrollBar().value()
                view.setPlainText(text)
                view.verticalScrollBar().setValue(bar)
        self.export_label.setText(f"→ src/GeneratedAutons/{export_paths(plan)[1].name}")

    def _fill_tree(self):
        planner, plan, sim = self.planner, self.planner.plan, self.planner.sim
        self.tree.blockSignals(True)
        self.tree.clear()
        selected_item = None

        def step_row(parent, path, step):
            s = sim.by_path[path]
            flags = []
            if step.is_async and len(path) == 1 and can_async(step) and plan.style == "lines":
                flags.append("⇉ async")
            if step.note.strip():
                flags.append(f"// {step.note.strip()}")
            text = step.kind.describe(step.params) + ("   " + "  ".join(flags) if flags else "")
            row = QTreeWidgetItem([str(s.number), text, f"{s.t0:.1f}"])
            row.setData(0, Qt.UserRole, path)
            row.setToolTip(1, f"{step.kind.label}: ≈{s.duration:.2f} s, starts ≈{s.t0:.2f} s"
                           + "".join(f"\n• {n}" for n in s.notes))
            if s.notes:  #hover for the notes
                row.setForeground(1, QColor("#c05a00"))
                row.setText(1, "⚠ " + text)
            parent.addChild(row) if parent is not None else self.tree.addTopLevelItem(row)
            return row

        for i, item in enumerate(plan.items):
            if isinstance(item, Group):
                label = f"Sequence ({len(item.steps)} steps)"
                if item.is_async and plan.style == "lines":
                    label += "   ⇉ async"
                if item.note.strip():
                    label += f"   // {item.note.strip()}"
                t0 = next((s.t0 for s in sim.steps if s.path[0] == i), 0.0)
                group_row = QTreeWidgetItem(["", label, f"{t0:.1f}"])
                group_row.setData(0, Qt.UserRole, (i,))
                self.tree.addTopLevelItem(group_row)
                if planner.selected == (i,):
                    selected_item = group_row
                for j, step in enumerate(item.steps):
                    row = step_row(group_row, (i, j), step)
                    if planner.selected == (i, j):
                        selected_item = row
                group_row.setExpanded(True)
            else:
                row = step_row(None, (i,), item)
                if planner.selected == (i,):
                    selected_item = row
        if selected_item is not None:
            selected_item.setSelected(True)
            self.tree.scrollToItem(selected_item)
        self.tree.blockSignals(False)

    def _on_tree_select(self):
        items = self.tree.selectedItems()
        path = items[0].data(0, Qt.UserRole) if items else None
        #selecting rebuilds the tree, so do it after the click finishes instead of inside the tree's own signal
        QTimer.singleShot(0, lambda: self.planner.select(path))

    # ---- step editor

    def _refresh_editor(self):
        planner = self.planner
        item = planner.plan.get(planner.selected)
        key = (planner.selected, getattr(item, "type", "group") if item is not None else None)
        if key != self._editor_for:
            self._build_editor(item)
            self._editor_for = key
        elif item is not None:  #same step: just update values (e.g. while dragging its handle)
            self._loading = True
            params = getattr(item, "params", {})
            for name, (prm, w) in self._param_widgets.items():
                if not w.hasFocus():
                    self._set_widget(prm, w, params.get(name))
            self._loading = False

    def _build_editor(self, item):
        while self.editor_form.rowCount():
            self.editor_form.removeRow(0)
        self._param_widgets = {}
        planner = self.planner
        path = planner.selected
        if item is None:
            self.editor_box.setTitle("Step")
            hint = QLabel("Click a step in the list (or its numbered dot on the field) to edit its "
                          "params here - timeouts, speed, wait time, etc.")
            hint.setWordWrap(True)
            self.editor_form.addRow(hint)
            return
        is_group = isinstance(item, Group)
        sim = planner.sim.by_path.get(path)
        self.editor_box.setTitle("Sequence" if is_group else f"Step {sim.number} · {item.kind.label}")
        self._loading = True

        note = QLineEdit(item.note)
        note.setPlaceholderText("comment on the generated line")
        note.textEdited.connect(lambda t: planner.set_item_field(path, "note", t))
        self.editor_form.addRow("Note", note)

        top_level = len(path) == 1
        if is_group or (top_level and can_async(item)):
            check = QCheckBox("->schedule() and move on (don't await)")
            check.setChecked(item.is_async)
            check.setEnabled(planner.plan.style == "lines")
            check.toggled.connect(lambda on: planner.set_item_field(path, "is_async", on))
            self.editor_form.addRow("Async", check)
        elif not top_level:
            self.editor_form.addRow("", QLabel("<i>runs in order inside its Sequence</i>"))

        if not is_group:
            for prm in item.kind.params:
                w = self._make_widget(prm, path)
                self._set_widget(prm, w, item.params[prm.name])
                label = prm.label + ("" if prm.optional else " *")
                self.editor_form.addRow(label, w)
                self._param_widgets[prm.name] = (prm, w)
        self._loading = False

    def _make_widget(self, prm, path):
        planner = self.planner

        def send(value):
            if not self._loading:
                planner.set_param(path, prm.name, value)

        if prm.kind == "bool":
            w = QCheckBox()
            w.toggled.connect(send)
        elif prm.kind == "choice":
            w = self._guarded(QComboBox())
            w.addItems(list(prm.choices))
            w.currentTextChanged.connect(send)
        elif prm.kind == "text":
            w = QPlainTextEdit()
            w.setFont(QFont("Consolas", 9))
            w.setFixedHeight(70)
            w.textChanged.connect(lambda: send(w.toPlainText()))
        elif prm.kind == "int":
            w = self._guarded(QSpinBox())
            w.setRange(int(prm.lo), int(prm.hi))
            w.setSingleStep(int(prm.step))
            w.setSuffix(f" {prm.unit}" if prm.unit else "")
            w.valueChanged.connect(send)
        else:  #float / auto
            w = self._guarded(QDoubleSpinBox())
            w.setRange(prm.lo, prm.hi)
            w.setDecimals(prm.decimals)
            w.setSingleStep(prm.step)
            w.setSuffix(f" {prm.unit}" if prm.unit else "")
            if prm.kind == "auto":
                w.setSpecialValueText("auto")  #shown at the minimum (0) = Units::AUTO / AUTO_TIME
                w.valueChanged.connect(lambda v: send(None if v <= prm.lo else round(v, 3)))
            else:
                w.valueChanged.connect(lambda v: send(round(v, 3)))
        return w

    def _set_widget(self, prm, w, value):
        if prm.kind == "bool":
            w.setChecked(bool(value))
        elif prm.kind == "choice":
            w.setCurrentText(value)
        elif prm.kind == "text":
            if w.toPlainText() != value:
                w.setPlainText(value)
        elif prm.kind == "auto":
            w.setValue(prm.lo if value is None else value)
        else:
            w.setValue(value)
