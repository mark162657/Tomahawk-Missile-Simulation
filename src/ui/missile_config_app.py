import tkinter as tk
from tkinter import messagebox, ttk

from src.missile.config_store import (
    DEFAULT_CONFIGURATION,
    FIELD_ORDER,
    FIELD_UNITS,
    load_configurations,
    save_configurations,
    validate_configuration,
)


class MissileConfigApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Missile Configuration Manager")
        self.root.geometry("980x640")
        self.root.minsize(900, 600)

        self.configurations = load_configurations()
        self.selected_index = 0

        self.name_var = tk.StringVar()
        self.field_vars = {field: tk.StringVar() for field in FIELD_ORDER}
        self.status_var = tk.StringVar(value="Missile configuration manager ready.")

        self._build_layout()
        self._refresh_listbox()
        self._load_selected_into_form(0)

    def _build_layout(self) -> None:
        self.root.configure(bg="#d8d3c4")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Card.TLabelframe", background="#f7f1e3", borderwidth=1)
        style.configure("Card.TLabelframe.Label", background="#f7f1e3", foreground="#30261c")

        shell = tk.Frame(self.root, bg="#d8d3c4")
        shell.pack(fill="both", expand=True, padx=16, pady=16)
        shell.grid_columnconfigure(0, weight=1)
        shell.grid_columnconfigure(1, weight=2)
        shell.grid_rowconfigure(1, weight=1)

        header = tk.Frame(shell, bg="#5b4b3a", padx=18, pady=16)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        tk.Label(
            header,
            text="Missile Configuration Manager",
            font=("Helvetica", 22, "bold"),
            bg="#5b4b3a",
            fg="#f8f4ec",
        ).pack(anchor="w")
        tk.Label(
            header,
            text="Create, name, and edit missile configurations shared with the planner.",
            font=("Helvetica", 11),
            bg="#5b4b3a",
            fg="#eadfce",
        ).pack(anchor="w", pady=(4, 0))

        list_card = ttk.LabelFrame(shell, text="Configurations", style="Card.TLabelframe")
        list_card.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        self._build_list_section(list_card)

        editor_card = ttk.LabelFrame(shell, text="Editor", style="Card.TLabelframe")
        editor_card.grid(row=1, column=1, sticky="nsew")
        self._build_editor_section(editor_card)

    def _build_list_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="both", expand=True)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)

        self.listbox = tk.Listbox(frame, activestyle="none", exportselection=False, font=("Menlo", 11))
        self.listbox.grid(row=0, column=0, sticky="nsew")
        self.listbox.bind("<<ListboxSelect>>", self._on_select)

        button_row = tk.Frame(frame, bg="#f7f1e3")
        button_row.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        button_row.grid_columnconfigure((0, 1), weight=1)

        ttk.Button(button_row, text="New Config", command=self.new_configuration).grid(
            row=0, column=0, sticky="ew", padx=(0, 8)
        )
        ttk.Button(button_row, text="Delete Config", command=self.delete_configuration).grid(
            row=0, column=1, sticky="ew"
        )

        ttk.Button(button_row, text="Reload", command=self.reload_configurations).grid(
            row=1, column=0, sticky="ew", padx=(0, 8), pady=(8, 0)
        )
        ttk.Button(button_row, text="Save All", command=self.save_current_configuration).grid(
            row=1, column=1, sticky="ew", pady=(8, 0)
        )

    def _build_editor_section(self, parent: ttk.LabelFrame) -> None:
        frame = tk.Frame(parent, bg="#f7f1e3", padx=14, pady=14)
        frame.pack(fill="both", expand=True)
        frame.grid_columnconfigure(1, weight=1)

        tk.Label(frame, text="Configuration Name", bg="#f7f1e3", fg="#30261c").grid(
            row=0, column=0, sticky="w", pady=5
        )
        tk.Entry(frame, textvariable=self.name_var, bg="#fffdf7", relief="flat").grid(
            row=0, column=1, sticky="ew", pady=5, padx=(12, 0)
        )

        for row_index, field in enumerate(FIELD_ORDER, start=1):
            label = f"{field.replace('_', ' ').title()} ({FIELD_UNITS[field]})"
            tk.Label(frame, text=label, bg="#f7f1e3", fg="#30261c").grid(
                row=row_index, column=0, sticky="w", pady=4
            )
            tk.Entry(frame, textvariable=self.field_vars[field], bg="#fffdf7", relief="flat").grid(
                row=row_index, column=1, sticky="ew", pady=4, padx=(12, 0)
            )

        tk.Label(
            frame,
            textvariable=self.status_var,
            bg="#f7f1e3",
            fg="#6d5f52",
            justify="left",
            anchor="w",
            font=("Helvetica", 10),
        ).grid(row=len(FIELD_ORDER) + 1, column=0, columnspan=2, sticky="ew", pady=(14, 0))

        ttk.Button(frame, text="Save Current Configuration", command=self.save_current_configuration).grid(
            row=len(FIELD_ORDER) + 2, column=0, columnspan=2, sticky="ew", pady=(12, 0)
        )

    def _refresh_listbox(self) -> None:
        self.listbox.delete(0, tk.END)
        for config in self.configurations:
            self.listbox.insert(tk.END, config["name"])

    def _load_selected_into_form(self, index: int) -> None:
        index = max(0, min(index, len(self.configurations) - 1))
        self.selected_index = index
        config = self.configurations[index]
        self.name_var.set(config["name"])
        for field in FIELD_ORDER:
            self.field_vars[field].set(str(config[field]))
        self.listbox.selection_clear(0, tk.END)
        self.listbox.selection_set(index)
        self.listbox.activate(index)

    def _collect_form(self) -> dict:
        payload = {"name": self.name_var.get().strip()}
        for field in FIELD_ORDER:
            payload[field] = self.field_vars[field].get().strip()
        return validate_configuration(payload)

    def _on_select(self, _event=None) -> None:
        selection = self.listbox.curselection()
        if not selection:
            return
        self._load_selected_into_form(selection[0])

    def new_configuration(self) -> None:
        base_name = "New Missile"
        existing = {config["name"].lower() for config in self.configurations}
        candidate = base_name
        suffix = 2
        while candidate.lower() in existing:
            candidate = f"{base_name} {suffix}"
            suffix += 1

        new_config = DEFAULT_CONFIGURATION.copy()
        new_config["name"] = candidate
        self.configurations.append(new_config)
        self._refresh_listbox()
        self._load_selected_into_form(len(self.configurations) - 1)
        self.status_var.set("Created a new configuration draft. Edit fields, then save.")

    def delete_configuration(self) -> None:
        if len(self.configurations) == 1:
            messagebox.showerror("Delete Error", "At least one missile configuration must remain.")
            return

        selection = self.listbox.curselection()
        if not selection:
            return

        index = selection[0]
        name = self.configurations[index]["name"]
        self.configurations.pop(index)
        save_configurations(self.configurations)
        self._refresh_listbox()
        self._load_selected_into_form(max(0, index - 1))
        self.status_var.set(f"Deleted configuration '{name}'.")

    def reload_configurations(self) -> None:
        self.configurations = load_configurations()
        self._refresh_listbox()
        self._load_selected_into_form(0)
        self.status_var.set("Reloaded configurations from disk.")

    def save_current_configuration(self) -> None:
        try:
            config = self._collect_form()
            self.configurations[self.selected_index] = config
            save_configurations(self.configurations)
            self.configurations = load_configurations()
            self._refresh_listbox()

            saved_index = next(
                (idx for idx, item in enumerate(self.configurations) if item["name"] == config["name"]),
                0,
            )
            self._load_selected_into_form(saved_index)
            self.status_var.set(f"Saved configuration '{config['name']}'.")
        except Exception as exc:
            messagebox.showerror("Save Error", str(exc))


def main() -> None:
    root = tk.Tk()
    MissileConfigApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
