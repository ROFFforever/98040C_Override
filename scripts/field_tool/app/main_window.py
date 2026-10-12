from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QColor, QIcon, QKeySequence, QPixmap
from PySide6.QtWidgets import (QApplication, QComboBox, QDialog, QDockWidget, QDoubleSpinBox, QFileDialog, QLabel,
                               QMainWindow, QMessageBox, QSlider, QToolBar, QToolButton)

from app import field_config as cfg
from app.field_view import FieldView
from app.keybinds import BY_ID, KeybindDialog, Keybinds
from app.planner import codegen
from app.planner.plan import identifier
from app.planner.plan_overlay import PlanOverlay
from app.planner.plan_panel import PlanPanel
from app.planner.planner import Planner
from app.planner.playback import PLAN_KEY, auto_offset, plan_trace
from app.planner.steps import BY_ID as STEP_BY_ID, STEP_TYPES
from app.robot_profiles import ProfileDialog, ProfileStore
from app.run_data import is_run_folder, list_runs, load_run
from app.run_overlay import RunOverlay

PLANNER_WIDTH = 440


def _color_icon(color):
    pixmap = QPixmap(12, 12)
    pixmap.fill(QColor(color))
    return QIcon(pixmap)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("98040C Field Tool")
        self.keybinds = Keybinds()
        self.profiles = ProfileStore()
        self._syncing_slider = False  #true while the code (not the user) moves the playback slider

        self.view = FieldView()
        self.setCentralWidget(self.view)
        self.overlay = RunOverlay(self.view, self.profiles.active())
        self.overlay.changed.connect(self._update_playback_ui)
        self.planner = Planner(self.profiles.active())
        self.plan_overlay = PlanOverlay(self.view, self.planner, self.profiles.active())
        self.planner.placing_changed.connect(self._on_placing_changed)
        self.planner.changed.connect(self._refresh_plan_trace)
        self.view.field_clicked.connect(self._on_field_click)
        #where plan t = 0 sits on the run's clock. Auto = line up the plan's first motion with the run's
        self._plan_offset = 0.0
        self._plan_offset_auto = True

        self.actions = {}  #bind id -> QAction, every keybind goes through one of these
        self._planner_actions = []  #only active while the planner is open
        self._build_actions()
        self._build_run_toolbar()
        self.addToolBarBreak()
        self._build_view_toolbar()
        self._build_playback_toolbar()
        self._build_planner_dock()
        self._apply_keybinds()

        #live cursor position in field inches, same frame as the robot's pose
        self.coord_label = QLabel()
        self.statusBar().addPermanentWidget(self.coord_label)
        #a widget instead of showMessage(), which gets wiped whenever a toolbar button is hovered
        self.hint_label = QLabel()
        self.hint_label.setMinimumWidth(1)  #let long text get cut off instead of forcing the window wider
        self.statusBar().addWidget(self.hint_label, 1)
        self._notes = []
        self._update_hint()
        self.view.cursor_moved.connect(lambda x, y: self.coord_label.setText(f"x: {x:+7.2f} in    y: {y:+7.2f} in"))
        self.view.cursor_left.connect(lambda: self.coord_label.setText(""))

        self._update_playback_ui()
        screen = self.screen().availableGeometry()
        side = min(screen.height() - 80, 950)
        self.resize(side, side)

    # ---- actions / keybinds

    def _build_actions(self):
        ov = self.overlay

        def add(bind_id, slot, checkable=False, checked=False, text=None, icon=None):
            action = QAction(text or BY_ID[bind_id].label, self)
            if icon is not None:
                action.setIcon(icon)
            if checkable:
                action.setCheckable(True)
                action.setChecked(checked)
                action.toggled.connect(slot)
            else:
                action.triggered.connect(slot)
            self.addAction(action)  #window-wide shortcut, even if the action isn't on a toolbar
            self.actions[bind_id] = action

        add("open_run", self._browse_run, text="Open folder…")
        add("keybinds", self._edit_keybinds, text="Keybinds…")
        add("profiles", self._edit_profiles, text="Edit…")
        add("fit", self.view.fit_field, text="Fit")
        add("grid", self.view.set_grid_visible, True, True, "Grid")
        add("axes", self.view.set_axes_visible, True, True, "Axes")
        add("robot_trace", lambda on: ov.set_visible("robot", on), True, True, "Robot odom", _color_icon(cfg.ROBOT_COLOR))
        add("camera_trace", lambda on: ov.set_visible("camera", on), True, True, "Camera tag", _color_icon(cfg.CAMERA_COLOR))
        add("ghost", ov.set_ghosts_visible, True, True, "Outline")
        add("motion_mode", ov.set_motion_mode, True, False, "Motion data")
        add("frame_mode", ov.set_show_frame, True, False, "Frame #")
        add("next", lambda: ov.step(1))
        add("prev", lambda: ov.step(-1))
        add("next10", lambda: ov.step(10))
        add("prev10", lambda: ov.step(-10))
        add("jump_camera", lambda: ov.jump_to("camera"))
        add("jump_robot", lambda: ov.jump_to("robot"))
        add("jump_plan", lambda: ov.jump_to(PLAN_KEY))
        add("align_plan", self._align_plan_to_playhead, text="Start at playhead")
        add("deselect", self._escape)
        add("play", ov.toggle_play, text="▶ Play")
        add("start", ov.go_start, text="⏮")
        add("end", ov.go_end, text="⏭")
        add("slower", lambda: ov.change_speed(-1), text="−")
        add("faster", lambda: ov.change_speed(1), text="+")

        pl = self.planner
        add("planner", self._set_planner_visible, True, False, "Planner")
        planner_binds = []
        for st in STEP_TYPES:
            add(f"add_{st.id}", lambda _=False, t=st.id: pl.start_placing(t), text=st.label)
            planner_binds.append(f"add_{st.id}")
        add("add_run_point", self._add_run_point, text="Run point → waypoint")
        add("step_prev", lambda: pl.select_relative(-1))
        add("step_next", lambda: pl.select_relative(1))
        add("step_up", lambda: pl.move_selected(-1))
        add("step_down", lambda: pl.move_selected(1))
        add("step_delete", pl.delete_selected)
        add("step_duplicate", pl.duplicate_selected)
        add("step_async", pl.toggle_async)
        add("group", pl.toggle_group)
        add("undo", pl.undo)
        add("redo", pl.redo)
        add("new_plan", lambda: pl.new_plan())
        add("dup_plan", pl.duplicate_plan)
        add("delete_plan", self._delete_plan)
        add("copy_code", lambda: QApplication.clipboard().setText(pl.code[1]))
        add("export", self._export_plan)
        self._planner_actions = [self.actions[b] for b in planner_binds + [
            "jump_plan", "align_plan", "add_run_point", "step_prev", "step_next", "step_up", "step_down", "step_delete", "step_duplicate",
            "step_async", "group", "undo", "redo", "new_plan", "dup_plan", "delete_plan", "copy_code", "export"]]
        for action in self._planner_actions:
            action.setEnabled(False)

    def _apply_keybinds(self):
        for bind_id, action in self.actions.items():
            key = self.keybinds.key(bind_id)
            action.setShortcut(QKeySequence(key))
            label = BY_ID[bind_id].label
            action.setToolTip(f"{label} ({key})" if key else label)

    def _edit_keybinds(self):
        self.overlay.pause()
        dialog = KeybindDialog(self.keybinds, self)
        if dialog.exec() == QDialog.Accepted:
            self.keybinds.set_all(dialog.result_mapping())
            self._apply_keybinds()
            self._update_hint()

    def _key(self, bind_id):
        return self.keybinds.key(bind_id) or "unbound"

    def _update_hint(self):
        if self.planner.placing is not None:
            hint = (f"Placing {STEP_BY_ID[self.planner.placing].label}: click the field   "
                    f"(Shift = snap 1 in / 15°, Alt = backwards)   |   {self._key('deselect')} to stop")
        elif self.actions["planner"].isChecked():
            hint = (f"{self._key('add_move_to_point')}-{self._key('add_code')} add step   drag handles to edit   "
                    f"{self._key('step_prev')}/{self._key('step_next')} select step   {self._key('play')} play plan   "
                    f"{self._key('export')} export   |   {self._key('keybinds')} all keybinds")
        else:
            hint = (f"Click a point   |   {self._key('prev')}/{self._key('next')} step   "
                    f"{self._key('jump_camera')}/{self._key('jump_robot')} camera/odom at same time   "
                    f"{self._key('play')} play   |   {self._key('planner')} auton planner   "
                    f"{self._key('keybinds')} all keybinds")
        self.hint_label.setText(("⚠ " + "; ".join(self._notes) + "   |   " if self._notes else "") + hint)

    # ---- auton planner

    def _build_planner_dock(self):
        self.planner_dock = QDockWidget("Auton planner", self)
        self.planner_dock.setObjectName("planner_dock")
        self.planner_dock.setFeatures(QDockWidget.DockWidgetMovable)  #shown/hidden with the Planner toggle only
        self.planner_panel = PlanPanel(self.planner, self.actions)
        self.planner_dock.setWidget(self.planner_panel)
        self.planner_dock.setMinimumWidth(380)
        self.addDockWidget(Qt.RightDockWidgetArea, self.planner_dock)
        self.planner_dock.hide()

    def _set_planner_visible(self, on):
        if on:
            self.planner.ensure_plan()
            screen = self.screen().availableGeometry()
            if not self.isMaximized() and self.width() + PLANNER_WIDTH <= screen.width():
                self.resize(self.width() + PLANNER_WIDTH, self.height())  #keep the field the same size
        else:
            self.planner.cancel_placing()
            self.planner.save()
        self.planner_dock.setVisible(on)
        self.plan_overlay.set_visible(on)
        for action in self._planner_actions:
            action.setEnabled(on)
        for action in self._plan_bar_actions:
            action.setVisible(on)
        self._refresh_plan_trace()
        if on:
            self.planner_dock.resize(PLANNER_WIDTH, self.planner_dock.height())
            self.resizeDocks([self.planner_dock], [PLANNER_WIDTH], Qt.Horizontal)
        self._update_hint()

    def _on_placing_changed(self):
        self.view.set_placing(self.planner.placing is not None)
        self._update_hint()

    def _on_field_click(self, x, y, in_per_px):
        if self.actions["planner"].isChecked():
            mods = QApplication.keyboardModifiers()
            if self.planner.placing is not None:
                self.planner.place_at(x, y, bool(mods & Qt.ShiftModifier), bool(mods & Qt.AltModifier))
                return
            path = self.plan_overlay.step_at(x, y, cfg.CLICK_TOLERANCE_PX * in_per_px)
            if path is not None:
                self.planner.select(path)
                return
        self.overlay.click(x, y, in_per_px)

    def _escape(self):
        if self.planner.cancel_placing():
            return
        if self.actions["planner"].isChecked() and self.planner.selected is not None:
            self.planner.select(None)
            return
        self.overlay.clear()

    def _add_run_point(self):
        """The run point in the data box becomes a moveToPoint - drive back to somewhere the robot really was."""
        if self.overlay.selected is None:
            return
        key, i = self.overlay.selected
        tr = self.overlay.items[key].trace
        self.planner.add_step("move_to_point", float(tr.x[i]), float(tr.y[i]))

    def _delete_plan(self):
        name = self.planner.plan.name
        if QMessageBox.question(self, "Delete plan", f"Delete the plan {name}?\n\n(Exported code in "
                                f"src/GeneratedAutons is left alone.)") == QMessageBox.Yes:
            self.planner.delete_plan()

    def _export_plan(self):
        plan = self.planner.plan
        self.planner.save()
        paths = codegen.export_paths(plan)
        edited = [p for p in paths if codegen.hand_edited(p)]
        if edited and QMessageBox.question(
                self, "Overwrite?", "Changed since the last export (or not made by this tool):\n\n"
                + "\n".join(f"{p.parent.parent.name}/{p.parent.name}/{p.name}" for p in edited)
                + "\n\nOverwrite?") != QMessageBox.Yes:
            return
        codegen.write(plan)
        fn = identifier(plan.name)
        args = "&chassis" + (", ..." if plan.extra_params.strip() else "")
        call = f"{fn}({args});" if plan.style == "lines" else f"await({fn}({args}));"
        QMessageBox.information(
            self, "Exported",
            f"Wrote include/GeneratedAutons/{fn}.h and src/GeneratedAutons/{fn}.cpp\n\n"
            f"In main.cpp:\n    #include \"GeneratedAutons/{fn}.h\"\nand in autonomous():\n    {call}\n\n"
            "GeneratedAutons isn't a cold folder, so a normal build picks it up.")

    def closeEvent(self, event):
        self.planner.save()
        super().closeEvent(event)

    # ---- toolbars

    def _build_run_toolbar(self):
        bar = QToolBar("Run")
        bar.setMovable(False)
        self.addToolBar(bar)

        bar.addWidget(QLabel(" Run: "))
        self.run_combo = QComboBox()
        self.run_combo.setMinimumWidth(200)
        self.run_combo.setFocusPolicy(Qt.NoFocus)  #keeps Space / arrows going to the shortcuts
        self.run_combo.setToolTip(f"Run folders in {cfg.RUNS_DIR}")
        self.run_combo.activated.connect(self._on_run_picked)
        bar.addWidget(self.run_combo)
        self._refresh_run_list()
        bar.addAction(QAction("↻", self, toolTip="Rescan the runs folder", triggered=self._refresh_run_list))
        bar.addAction(self.actions["open_run"])

        bar.addSeparator()
        bar.addWidget(QLabel(" Robot: "))
        self.profile_combo = QComboBox()
        self.profile_combo.setFocusPolicy(Qt.NoFocus)
        self.profile_combo.activated.connect(self._on_profile_picked)
        bar.addWidget(self.profile_combo)
        bar.addAction(self.actions["profiles"])
        self._refresh_profile_combo()

        bar.addSeparator()
        bar.addAction(self.actions["keybinds"])

    def _build_view_toolbar(self):
        bar = QToolBar("View")
        bar.setMovable(False)
        bar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.addToolBar(bar)
        for bind_id in ("fit", "grid", "axes", None, "robot_trace", "camera_trace", "ghost", None,
                        "motion_mode", "frame_mode", None, "planner"):
            if bind_id is None:
                bar.addSeparator()
            else:
                bar.addAction(self.actions[bind_id])
        self.run_label = QLabel()
        bar.addWidget(self.run_label)

    def _build_playback_toolbar(self):
        bar = QToolBar("Playback")
        bar.setMovable(False)
        self.addToolBar(Qt.BottomToolBarArea, bar)
        for bind_id in ("start", "play", "end"):
            bar.addAction(self.actions[bind_id])
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.slider.valueChanged.connect(self._on_slider)
        bar.addWidget(self.slider)
        self.time_label = QLabel()
        self.time_label.setMinimumWidth(150)
        self.time_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        bar.addWidget(self.time_label)
        bar.addAction(self.actions["slower"])
        self.speed_label = QLabel()
        self.speed_label.setMinimumWidth(40)
        self.speed_label.setAlignment(Qt.AlignCenter)
        bar.addWidget(self.speed_label)
        bar.addAction(self.actions["faster"])

        #plan timing, only shown while the planner is open
        self._plan_bar_actions = [bar.addSeparator(), bar.addWidget(QLabel(" Plan starts at "))]
        self.plan_offset_spin = QDoubleSpinBox()
        self.plan_offset_spin.setRange(-600, 600)
        self.plan_offset_spin.setDecimals(2)
        self.plan_offset_spin.setSingleStep(0.05)
        self.plan_offset_spin.setSuffix(" s")
        self.plan_offset_spin.setKeyboardTracking(False)
        self.plan_offset_spin.setToolTip("Where the plan's t = 0 sits on the run's clock")
        self.plan_offset_spin.valueChanged.connect(self._on_plan_offset_edited)
        self._plan_bar_actions.append(bar.addWidget(self.plan_offset_spin))
        self.plan_auto_button = QToolButton()
        self.plan_auto_button.setText("Auto")
        self.plan_auto_button.setCheckable(True)
        self.plan_auto_button.setChecked(True)
        self.plan_auto_button.setFocusPolicy(Qt.NoFocus)
        self.plan_auto_button.setToolTip("Line the plan's first motion up with the run's first motion "
                                         "(plan t = 0 with no run loaded)")
        self.plan_auto_button.toggled.connect(self._on_plan_auto_toggled)
        self._plan_bar_actions.append(bar.addWidget(self.plan_auto_button))
        self._plan_bar_actions.append(self.actions["align_plan"])
        bar.addAction(self.actions["align_plan"])
        for action in self._plan_bar_actions:
            action.setVisible(False)

    # ---- playback ui

    def _on_slider(self, value_ms):
        if not self._syncing_slider:
            self.overlay.pause()
            self.overlay.seek(value_ms / 1000.0)

    def _update_playback_ui(self):
        ov = self.overlay
        rng = ov.time_range()
        self.slider.setEnabled(rng is not None)
        self._syncing_slider = True  #moving the slider to match the playhead shouldn't seek again
        if rng is not None:
            self.slider.setRange(int(rng[0] * 1000), int(rng[1] * 1000))
            self.slider.setValue(int((ov.time if ov.time is not None else rng[0]) * 1000))
        self._syncing_slider = False
        if rng is None:
            self.time_label.setText("")
        else:
            now = "—" if ov.time is None else f"{ov.time:.2f}"
            self.time_label.setText(f"{now} / {rng[1]:.2f} s")
        self.actions["play"].setText("⏸ Pause" if ov.playing else "▶ Play")
        self.speed_label.setText(f"{ov.speed:g}x")
        self.plan_overlay.set_playhead_active(ov.time is not None)

    # ---- plan playback

    def _refresh_plan_trace(self):
        """Rebuild the plan's trace (it plays on the same playhead as odom / camera)."""
        if not self.actions["planner"].isChecked() or self.planner.plan is None:
            self.overlay.set_extra_trace(PLAN_KEY, None)
            return
        tr = plan_trace(self.planner.sim, self.planner.plan, 0.0)
        if self._plan_offset_auto:
            self._plan_offset = auto_offset(self.overlay.run, tr)
        tr.t = tr.t + self._plan_offset  #velocities don't change with a time shift
        self.plan_offset_spin.blockSignals(True)
        self.plan_offset_spin.setValue(self._plan_offset)
        self.plan_offset_spin.blockSignals(False)
        if self.planner.dragging:  #the dragged step's end outline takes over
            self.overlay.hidden_ghosts.add(PLAN_KEY)
        else:
            self.overlay.hidden_ghosts.discard(PLAN_KEY)
        self.overlay.set_extra_trace(PLAN_KEY, tr)

    def _on_plan_offset_edited(self, value):
        self._plan_offset = value
        self.plan_auto_button.setChecked(False)  #typing a start time = manual
        self._plan_offset_auto = False
        self._refresh_plan_trace()

    def _on_plan_auto_toggled(self, on):
        self._plan_offset_auto = on
        self._refresh_plan_trace()

    def _align_plan_to_playhead(self):
        if self.overlay.time is None:
            return
        self._plan_offset = self.overlay.time
        self._plan_offset_auto = False
        self.plan_auto_button.blockSignals(True)
        self.plan_auto_button.setChecked(False)
        self.plan_auto_button.blockSignals(False)
        self._refresh_plan_trace()

    # ---- runs

    def _refresh_run_list(self):
        current = self.run_combo.currentData()
        self.run_combo.clear()
        self.run_combo.addItem("Select a run…", None)
        for folder in list_runs():
            self.run_combo.addItem(folder.name, str(folder.resolve()))
        i = self.run_combo.findData(current)
        self.run_combo.setCurrentIndex(max(i, 0))

    def _on_run_picked(self, index):
        folder = self.run_combo.itemData(index)
        if folder:
            self.load_run(folder)
        else:
            self.overlay.set_run(None)
            self.run_label.setText("")
            self._notes = []
            self._update_hint()

    def _browse_run(self):
        self.overlay.pause()
        start = str(cfg.RUNS_DIR) if cfg.RUNS_DIR.exists() else str(Path.home())
        folder = QFileDialog.getExistingDirectory(self, "Open run folder", start)
        if not folder:
            return
        if not is_run_folder(folder):
            QMessageBox.warning(self, "Not a run folder", "That folder has no track.csv or .ndjson robot log.")
            return
        self.load_run(folder)

    def load_run(self, folder):
        try:
            run = load_run(folder)
        except Exception as e:  #a bad file shouldn't kill the whole tool
            QMessageBox.critical(self, "Couldn't load run", f"{folder}\n\n{type(e).__name__}: {e}")
            return
        self.overlay.set_run(run)
        self._refresh_plan_trace()  #auto start time depends on the run
        self.actions["robot_trace"].setEnabled(run.robot is not None)
        self.actions["camera_trace"].setEnabled(run.camera is not None)
        i = self.run_combo.findData(str(Path(folder).resolve()))  #folders outside the runs dir just show "Select a run…"
        self.run_combo.setCurrentIndex(max(i, 0))

        parts = []
        if run.robot is not None:
            parts.append(f"odom {len(run.robot)} pts")
        if run.camera is not None:
            parts.append(f"camera {len(run.camera)} pts")
        parts.append("t = robot clock" if run.clock == "robot" else "t = video time")
        self.run_label.setText("   " + "  ·  ".join(parts))
        self._notes = run.notes
        self._update_hint()
        self.view.setFocus()

    # ---- robot profiles

    def _refresh_profile_combo(self):
        self.profile_combo.clear()
        for p in self.profiles.profiles:
            self.profile_combo.addItem(f"{p.name}  ({p.length_in:g} x {p.width_in:g})", p.name)
        self.profile_combo.setCurrentIndex(self.profile_combo.findData(self.profiles.active_name))

    def _on_profile_picked(self, index):
        self.profiles.set_active(self.profile_combo.itemData(index))
        self._apply_profile()

    def _edit_profiles(self):
        self.overlay.pause()
        dialog = ProfileDialog(self.profiles, self)
        if dialog.exec() == QDialog.Accepted:
            self.profiles.replace(dialog.profiles, dialog.active_name)
            self._refresh_profile_combo()
            self._apply_profile()

    def _apply_profile(self):
        profile = self.profiles.active()
        self.overlay.set_profile(profile)
        self.plan_overlay.set_profile(profile)
        self.planner.set_profile(profile)  #swing arcs + time estimates
