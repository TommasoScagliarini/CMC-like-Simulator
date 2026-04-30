"""
create_setup.py
===============
Small Tkinter wizard that creates a simulator setup XML file.
"""

from __future__ import annotations

import os
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from config import SimulatorConfig
from path_resolver import REPO_ROOT, resolve_simulator_paths
from setup_io import (
    build_simulation_setup,
    read_kinematics_time_range,
    write_setup_xml,
)


def _default_time_values() -> dict[str, str]:
    cfg = SimulatorConfig()
    defaults = {
        "t_start": f"{cfg.t_start:.15g}",
        "t_end": f"{cfg.t_end:.15g}",
    }
    try:
        resolved = resolve_simulator_paths(cfg)
    except Exception:
        return defaults

    try:
        inferred_start, inferred_end = read_kinematics_time_range(
            resolved.kinematics_path
        )
    except Exception:
        return defaults

    defaults["t_start"] = f"{inferred_start:.15g}"
    defaults["t_end"] = f"{inferred_end:.15g}"
    return defaults


class SetupWizard:
    def __init__(self) -> None:
        defaults = _default_time_values()

        self.root = tk.Tk()
        self.root.title("Create Simulator Setup XML")
        self.root.minsize(920, 290)
        self.models_dir = (REPO_ROOT / "models").resolve()

        self.model_var = tk.StringVar(value="")
        self.kinematics_var = tk.StringVar(value="")
        self.external_var = tk.StringVar(value="")
        self.reserve_var = tk.StringVar(value="")
        self.t_start_var = tk.StringVar(value=defaults["t_start"])
        self.t_end_var = tk.StringVar(value=defaults["t_end"])

        self._build_ui()

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        self._add_row(
            frame,
            key="model",
            row=0,
            label="Model (.osim)",
            variable=self.model_var,
            browse_title="Select OpenSim model",
            filetypes=[("OpenSim model", "*.osim"), ("All files", "*.*")],
        )
        self._add_row(
            frame,
            key="kinematics",
            row=1,
            label="Kinematics (.sto/.mot)",
            variable=self.kinematics_var,
            browse_title="Select kinematics file",
            filetypes=[
                ("Kinematics files", "*.sto *.mot"),
                ("All files", "*.*"),
            ],
        )
        self._add_row(
            frame,
            key="external",
            row=2,
            label="External loads (.xml)",
            variable=self.external_var,
            browse_title="Select ExternalLoads XML",
            filetypes=[("XML files", "*.xml"), ("All files", "*.*")],
        )
        self._add_row(
            frame,
            key="reserve",
            row=3,
            label="Reserve actuators (.xml)",
            variable=self.reserve_var,
            browse_title="Select reserve actuators XML",
            filetypes=[("XML files", "*.xml"), ("All files", "*.*")],
        )

        ttk.Label(frame, text="t_start [s]").grid(
            row=4,
            column=0,
            sticky="w",
            padx=(0, 10),
            pady=6,
        )
        ttk.Entry(frame, textvariable=self.t_start_var).grid(
            row=4,
            column=1,
            sticky="ew",
            pady=6,
        )
        ttk.Label(frame, text="t_end [s]").grid(
            row=5,
            column=0,
            sticky="w",
            padx=(0, 10),
            pady=6,
        )
        ttk.Entry(frame, textvariable=self.t_end_var).grid(
            row=5,
            column=1,
            sticky="ew",
            pady=6,
        )

        save_btn = ttk.Button(frame, text="Save setup XML", command=self._save_setup)
        save_btn.grid(row=6, column=1, sticky="e", pady=(14, 0))

    def _add_row(
        self,
        parent: ttk.Frame,
        *,
        key: str,
        row: int,
        label: str,
        variable: tk.StringVar,
        browse_title: str,
        filetypes: list[tuple[str, str]],
    ) -> None:
        ttk.Label(parent, text=label).grid(
            row=row,
            column=0,
            sticky="w",
            padx=(0, 10),
            pady=6,
        )
        ttk.Entry(parent, textvariable=variable).grid(
            row=row,
            column=1,
            sticky="ew",
            pady=6,
        )
        ttk.Button(
            parent,
            text="Sfoglia...",
            command=lambda: self._browse_file(key, variable, browse_title, filetypes),
        ).grid(row=row, column=2, sticky="w", padx=(10, 0), pady=6)

    def _browse_file(
        self,
        key: str,
        variable: tk.StringVar,
        title: str,
        filetypes: list[tuple[str, str]],
    ) -> None:
        initial_dir = str(self.models_dir)

        previous_cwd = Path.cwd()
        try:
            os.chdir(initial_dir)
            self.root.tk.call("cd", initial_dir)
            filename = filedialog.askopenfilename(
                parent=self.root,
                title=title,
                initialdir=initial_dir,
                initialfile="",
                filetypes=filetypes,
            )
        finally:
            os.chdir(previous_cwd)
        if filename:
            variable.set(filename)
            if key == "kinematics":
                self._refresh_time_range(filename)

    def _refresh_time_range(self, kinematics_path: str | Path) -> None:
        try:
            inferred_start, inferred_end = read_kinematics_time_range(kinematics_path)
        except Exception as exc:
            messagebox.showwarning(
                "Kinematics range unavailable",
                f"Unable to infer t_start/t_end from the selected kinematics file.\n\n{exc}",
                parent=self.root,
            )
            return

        self.t_start_var.set(f"{inferred_start:.15g}")
        self.t_end_var.set(f"{inferred_end:.15g}")

    def _save_setup(self) -> None:
        try:
            setup = build_simulation_setup(
                self.model_var.get(),
                self.kinematics_var.get(),
                self.external_var.get(),
                self.reserve_var.get(),
                self.t_start_var.get(),
                self.t_end_var.get(),
            )
        except Exception as exc:
            messagebox.showerror(
                "Invalid setup",
                str(exc),
                parent=self.root,
            )
            return

        initial_dir = str(self.models_dir)
        initial_name = f"{setup.model_file.stem}_setup.xml"
        previous_cwd = Path.cwd()
        try:
            os.chdir(initial_dir)
            self.root.tk.call("cd", initial_dir)
            destination = filedialog.asksaveasfilename(
                parent=self.root,
                title="Save simulator setup XML",
                initialdir=initial_dir,
                initialfile=initial_name,
                defaultextension=".xml",
                filetypes=[("XML files", "*.xml"), ("All files", "*.*")],
            )
        finally:
            os.chdir(previous_cwd)
        if not destination:
            return

        try:
            saved_path = write_setup_xml(setup, destination)
        except Exception as exc:
            messagebox.showerror(
                "Save failed",
                str(exc),
                parent=self.root,
            )
            return

        messagebox.showinfo(
            "Setup saved",
            f"Setup XML saved to:\n{saved_path}",
            parent=self.root,
        )
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    SetupWizard().run()


if __name__ == "__main__":
    main()
