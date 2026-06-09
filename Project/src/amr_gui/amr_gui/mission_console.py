#!/usr/bin/env python3
"""
Mission Console GUI for the AMR service robot.

Phase 9 introduced this module. Master Plan §3.1 (amr_gui responsibilities),
§6.9 (canonical landmark name set).

Threading model (CRITICAL — see Phase 9 brief §1.3): the tkinter main loop
owns the main thread. rclpy.spin_once is called on a tkinter timer
(root.after) every 50ms. All ROS callbacks therefore run on the tkinter
thread; widget updates are safe from those callbacks. NEVER use threading
or queues here — the design is single-threaded by intent.

Usage:
    ros2 run amr_gui mission_console
    # or
    ros2 launch amr_gui gui.launch.py
"""

from __future__ import annotations

import datetime as dt
import tkinter as tk
from tkinter import ttk
from typing import Optional

import rclpy
from rclpy.node import Node

from amr_gui.action_client import MissionActionClient
from amr_mission_manager.action import ExecuteMission


# Mission types per Master Plan §6.9 + Phase 9 brief §1.5.
MISSION_TYPES = [
    ("grocery", "Grocery Delivery", "supermarket"),
    ("food", "Food Delivery", "restaurant"),
    ("fire", "Fire Emergency", "fire_station"),
    ("medical", "Medical Emergency", "pharmacy"),
]

# Destination houses per Master Plan §6.9.
DESTINATION_HOUSES = ["house_1", "house_2", "house_3", "house_4", "house_5"]


# Visual design palette (Phase 9 cosmetic redesign — no logic impact).
COLORS = {
    "bg":           "#f5f6fa",
    "card":         "#ffffff",
    "border":       "#d1d5db",
    "text":         "#1f2937",
    "muted":        "#6b7280",
    "primary":      "#2563eb",
    "primary_dark": "#1d4ed8",
    "danger":       "#dc2626",
    "danger_dark":  "#b91c1c",
    "log_bg":       "#1e293b",
    "log_text":     "#e2e8f0",
    "log_dim":      "#94a3b8",
    "log_send":     "#60a5fa",
    "log_error":    "#f87171",
    "log_success":  "#4ade80",
    "log_warning":  "#fbbf24",
}

# Status-pill colors keyed by status kind: (bg, fg).
STATUS_COLORS = {
    "idle":     ("#e5e7eb", "#374151"),
    "busy":     ("#dbeafe", "#1e40af"),
    "complete": ("#dcfce7", "#15803d"),
    "error":    ("#fee2e2", "#991b1b"),
    "warning":  ("#fef3c7", "#92400e"),
}


class MissionConsole:
    """The tkinter-based mission entry GUI."""

    SPIN_INTERVAL_MS = 50  # rclpy.spin_once polled every 50ms (~20Hz)

    def __init__(self, root: tk.Tk, node: Node) -> None:
        self._root = root
        self._node = node
        self._client = MissionActionClient(node)
        self._wire_callbacks()

        # GUI state
        self._mission_type_var = tk.StringVar(value="grocery")
        self._destination_var = tk.StringVar(value=DESTINATION_HOUSES[0])

        self._apply_theme()
        self._build_widgets()
        self._set_idle()
        self._set_status("idle", "idle")

        # Start the rclpy spin pump.
        self._root.after(self.SPIN_INTERVAL_MS, self._spin_pump)

        # Handle window close.
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ----- Theme -----

    def _apply_theme(self) -> None:
        """Configure ttk styles for a modern, calm appearance."""
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass  # fall back to default if clam isn't available

        bg = COLORS["bg"]
        card = COLORS["card"]
        border = COLORS["border"]
        text = COLORS["text"]
        muted = COLORS["muted"]
        primary = COLORS["primary"]
        primary_dark = COLORS["primary_dark"]
        danger = COLORS["danger"]
        danger_dark = COLORS["danger_dark"]

        # Frames
        style.configure("Main.TFrame", background=bg)
        style.configure("TFrame", background=bg)

        # Section LabelFrames
        style.configure(
            "Section.TLabelframe",
            background=bg,
            foreground=text,
            bordercolor=border,
            borderwidth=1,
            relief="solid",
        )
        style.configure(
            "Section.TLabelframe.Label",
            background=bg,
            foreground=muted,
            font=("TkDefaultFont", 9, "bold"),
        )

        # Header labels
        style.configure(
            "Title.TLabel",
            background=bg,
            foreground=primary,
            font=("TkDefaultFont", 17, "bold"),
        )
        style.configure(
            "Subtitle.TLabel",
            background=bg,
            foreground=muted,
            font=("TkDefaultFont", 10),
        )
        style.configure(
            "StatusLabel.TLabel",
            background=bg,
            foreground=muted,
            font=("TkDefaultFont", 9, "bold"),
        )

        # Radiobuttons — bigger hit targets, calm color
        style.configure(
            "Mission.TRadiobutton",
            background=bg,
            foreground=text,
            font=("TkDefaultFont", 10),
            indicatorcolor=card,
            indicatorbackground=card,
            indicatorforeground=primary,
        )
        style.map(
            "Mission.TRadiobutton",
            background=[("active", bg)],
            indicatorcolor=[("selected", primary), ("!selected", card)],
            foreground=[("disabled", muted)],
        )

        # Combobox
        style.configure(
            "TCombobox",
            fieldbackground=card,
            foreground=text,
            background=card,
            bordercolor=border,
            arrowcolor=primary,
            padding=4,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", card)],
            selectbackground=[("readonly", card)],
            selectforeground=[("readonly", text)],
        )

        # Primary button (Send)
        style.configure(
            "Primary.TButton",
            background=primary,
            foreground="#ffffff",
            font=("TkDefaultFont", 11, "bold"),
            padding=(14, 10),
            bordercolor=primary,
            borderwidth=0,
            focuscolor=primary,
        )
        style.map(
            "Primary.TButton",
            background=[
                ("active", primary_dark),
                ("pressed", primary_dark),
                ("disabled", "#cbd5e1"),
            ],
            foreground=[("disabled", "#94a3b8")],
        )

        # Danger button (Cancel)
        style.configure(
            "Danger.TButton",
            background=danger,
            foreground="#ffffff",
            font=("TkDefaultFont", 11, "bold"),
            padding=(14, 10),
            bordercolor=danger,
            borderwidth=0,
            focuscolor=danger,
        )
        style.map(
            "Danger.TButton",
            background=[
                ("active", danger_dark),
                ("pressed", danger_dark),
                ("disabled", "#e5e7eb"),
            ],
            foreground=[("disabled", "#9ca3af")],
        )

        # Dark scrollbar to match the log
        style.configure(
            "Log.Vertical.TScrollbar",
            background=COLORS["log_bg"],
            troughcolor="#334155",
            bordercolor=COLORS["log_bg"],
            arrowcolor=COLORS["log_dim"],
            gripcount=0,
            relief="flat",
        )

    # ----- Widget construction -----

    def _build_widgets(self) -> None:
        self._root.title("AMR Mission Console")
        self._root.geometry("540x580")
        self._root.resizable(False, False)
        self._root.configure(bg=COLORS["bg"])

        main = ttk.Frame(self._root, padding=14, style="Main.TFrame")
        main.pack(fill=tk.BOTH, expand=True)

        # --- Header ---
        header = ttk.Frame(main, style="Main.TFrame")
        header.pack(fill=tk.X, pady=(0, 14))
        ttk.Label(
            header, text="AMR Mission Console", style="Title.TLabel"
        ).pack(anchor=tk.W)
        ttk.Label(
            header,
            text="Autonomous service-robot dispatcher",
            style="Subtitle.TLabel",
        ).pack(anchor=tk.W, pady=(2, 0))

        # --- Mission type selector ---
        type_frame = ttk.LabelFrame(
            main, text="MISSION TYPE",
            padding=(12, 6, 12, 10),
            style="Section.TLabelframe",
        )
        type_frame.pack(fill=tk.X, pady=(0, 10))
        for mission_id, label, source in MISSION_TYPES:
            text = f"  {label}    →   {source}"
            ttk.Radiobutton(
                type_frame,
                text=text,
                value=mission_id,
                variable=self._mission_type_var,
                style="Mission.TRadiobutton",
            ).pack(anchor=tk.W, pady=2, padx=2)

        # --- Destination dropdown ---
        dest_frame = ttk.LabelFrame(
            main, text="DESTINATION HOUSE",
            padding=(12, 6, 12, 10),
            style="Section.TLabelframe",
        )
        dest_frame.pack(fill=tk.X, pady=(0, 12))
        dest_combo = ttk.Combobox(
            dest_frame,
            textvariable=self._destination_var,
            values=DESTINATION_HOUSES,
            state="readonly",
            font=("TkDefaultFont", 10),
        )
        dest_combo.pack(fill=tk.X, ipady=2)

        # --- Action buttons ---
        button_frame = ttk.Frame(main, style="Main.TFrame")
        button_frame.pack(fill=tk.X, pady=(0, 14))
        self._send_button = ttk.Button(
            button_frame,
            text="Send Mission",
            command=self._on_send,
            style="Primary.TButton",
        )
        self._send_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 6))
        self._cancel_button = ttk.Button(
            button_frame,
            text="Cancel",
            command=self._on_cancel,
            style="Danger.TButton",
            state=tk.DISABLED,
        )
        self._cancel_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(6, 0))

        # --- Status pill ---
        status_outer = ttk.Frame(main, style="Main.TFrame")
        status_outer.pack(fill=tk.X, pady=(0, 12))
        ttk.Label(
            status_outer, text="STATUS", style="StatusLabel.TLabel"
        ).pack(side=tk.LEFT, padx=(0, 10))
        # tk.Label (not ttk) for direct per-instance bg/fg control of the pill.
        self._status_pill = tk.Label(
            status_outer,
            text="  idle  ",
            font=("TkDefaultFont", 10, "bold"),
            bg=STATUS_COLORS["idle"][0],
            fg=STATUS_COLORS["idle"][1],
            padx=14,
            pady=4,
            bd=0,
        )
        self._status_pill.pack(side=tk.LEFT)

        # --- Mission log (dark, monospace, color-coded) ---
        log_frame = ttk.LabelFrame(
            main, text="MISSION LOG",
            padding=(2, 4, 2, 2),
            style="Section.TLabelframe",
        )
        log_frame.pack(fill=tk.BOTH, expand=True)

        log_inner = tk.Frame(log_frame, bg=COLORS["log_bg"], bd=0)
        log_inner.pack(fill=tk.BOTH, expand=True)

        self._log = tk.Text(
            log_inner,
            height=10,
            wrap=tk.WORD,
            state=tk.DISABLED,
            bg=COLORS["log_bg"],
            fg=COLORS["log_text"],
            font=("TkFixedFont", 9),
            relief="flat",
            bd=0,
            padx=10,
            pady=8,
            insertbackground=COLORS["log_text"],
            selectbackground="#475569",
            highlightthickness=0,
        )

        # Color-coded log tags
        self._log.tag_configure("timestamp", foreground=COLORS["log_dim"])
        self._log.tag_configure("info",      foreground=COLORS["log_text"])
        self._log.tag_configure("send",      foreground=COLORS["log_send"])
        self._log.tag_configure("error",     foreground=COLORS["log_error"])
        self._log.tag_configure("success",   foreground=COLORS["log_success"])
        self._log.tag_configure("warning",   foreground=COLORS["log_warning"])

        scrollbar = ttk.Scrollbar(
            log_inner, command=self._log.yview, style="Log.Vertical.TScrollbar"
        )
        self._log.configure(yscrollcommand=scrollbar.set)
        self._log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # ----- ROS / action callback wiring -----

    def _wire_callbacks(self) -> None:
        self._client.on_goal_accepted = self._on_goal_accepted
        self._client.on_goal_rejected = self._on_goal_rejected
        self._client.on_feedback = self._on_feedback
        self._client.on_result = self._on_result
        self._client.on_server_unavailable = self._on_server_unavailable

    # ----- Button handlers -----

    def _on_send(self) -> None:
        mission_type = self._mission_type_var.get()
        destination = self._destination_var.get()
        self._log_line(
            f"Sending mission: type={mission_type}, dest={destination}", "send"
        )
        self._set_busy("dispatching...")
        ok = self._client.send_mission(mission_type, destination)
        if not ok:
            # on_server_unavailable already fired from inside send_mission.
            return

    def _on_cancel(self) -> None:
        self._log_line("Cancel requested by user", "warning")
        self._client.cancel_mission()

    def _on_close(self) -> None:
        if self._client.has_active_goal():
            self._log_line("Window closing — cancelling active mission", "warning")
            self._client.cancel_mission()
        # Brief delay to let cancel acknowledgement flush, then exit
        self._root.after(200, self._root.destroy)

    # ----- Action callbacks (run on tkinter thread via spin_once) -----

    def _on_goal_accepted(self) -> None:
        self._log_line("Goal accepted by mission server", "success")

    def _on_goal_rejected(self, reason: str) -> None:
        self._log_line(f"Goal REJECTED: {reason}", "error")
        self._set_status(f"rejected: {reason}", "error")
        self._set_idle()

    def _on_server_unavailable(self) -> None:
        self._log_line("ERROR: mission server is not available", "error")
        self._set_status("ERROR: server unavailable", "error")
        self._set_idle()

    def _on_feedback(self, feedback: ExecuteMission.Feedback) -> None:
        phase = feedback.current_phase
        dist = feedback.distance_to_current_goal
        if phase in ("navigating_to_source", "navigating_to_destination", "returning_to_dock"):
            self._set_status(f"{phase} ({dist:.1f}m)", "busy")
        else:
            self._set_status(phase, "busy")
        self._log_line(f"Feedback: phase={phase}, dist={dist:.2f}m", "info")

    def _on_result(self, result: ExecuteMission.Result) -> None:
        if result.success:
            d = result.mission_duration
            secs = d.sec + d.nanosec * 1e-9
            self._set_status(f"complete (took {secs:.1f}s)", "complete")
            self._log_line(
                f"Mission COMPLETE: {result.message} (duration: {secs:.1f}s)",
                "success",
            )
        else:
            self._set_status(f"FAILED: {result.message}", "error")
            self._log_line(f"Mission FAILED: {result.message}", "error")
        self._set_idle()

    # ----- State helpers -----

    def _set_idle(self) -> None:
        self._send_button.configure(state=tk.NORMAL)
        self._cancel_button.configure(state=tk.DISABLED)

    def _set_busy(self, status_text: str) -> None:
        self._send_button.configure(state=tk.DISABLED)
        self._cancel_button.configure(state=tk.NORMAL)
        self._set_status(status_text, "busy")

    def _set_status(self, text: str, kind: str = "idle") -> None:
        bg, fg = STATUS_COLORS.get(kind, STATUS_COLORS["idle"])
        self._status_pill.configure(text=f"  {text}  ", bg=bg, fg=fg)

    def _log_line(self, message: str, tag: str = "info") -> None:
        timestamp = dt.datetime.now().strftime("%H:%M:%S")
        self._log.configure(state=tk.NORMAL)
        self._log.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self._log.insert(tk.END, f"{message}\n", tag)
        self._log.see(tk.END)
        self._log.configure(state=tk.DISABLED)

    # ----- rclpy spin pump -----

    def _spin_pump(self) -> None:
        try:
            rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception as exc:
            self._log_line(f"rclpy spin error: {exc}", "error")
        if rclpy.ok():
            self._root.after(self.SPIN_INTERVAL_MS, self._spin_pump)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("mission_console_gui")
    root = tk.Tk()
    console = MissionConsole(root, node)
    try:
        root.mainloop()
    finally:
        console._client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
