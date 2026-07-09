import tkinter as tk
from tkinter import ttk

class OverrideScorer:
    def __init__(self, root):
        self.root = root
        self.root.title("V5RC Override - 2x2 Grid Scorer")
        
        # Scoring Rules Point Values [1]
        self.VAL_ALLIANCE_PIN = 5
        self.VAL_YELLOW_PIN = 10
        self.VAL_ROBOT_MIDFIELD = 8

        # Midfield and Auto State
        self.red_midfield_robots = tk.IntVar(value=0)
        self.blue_midfield_robots = tk.IntVar(value=0)
        self.midfield_yellow_pins = tk.IntVar(value=0)

        # Quadrant Data [2]
        self.quadrants = []
        for i in range(4):
            self.quadrants.append({
                "toggle": tk.StringVar(value="Yellow"),
                "red_pins": tk.IntVar(value=0),
                "blue_pins": tk.IntVar(value=0),
                "yellow_pins": tk.IntVar(value=0)
            })

        self.setup_ui()

    def setup_ui(self):
        # Left Side Container for All Inputs
        input_frame = ttk.Frame(self.root, padding="10")
        input_frame.pack(side=tk.LEFT, fill=tk.BOTH)

        ttk.Label(input_frame, text="Field Scoring Inputs", font=('Helvetica', 16, 'bold')).grid(row=0, column=0, columnspan=2, pady=10)

        # --- 2x2 Card Layout for Quadrants ---
        for i in range(4):
            frame = ttk.LabelFrame(input_frame, text=f"Quadrant {i+1}")
            # Calculate 2x2 grid position
            row_idx = (i // 2) + 1
            col_idx = i % 2
            frame.grid(row=row_idx, column=col_idx, padx=10, pady=10, sticky="nsew")
            
            # Toggle Selection [3]
            ttk.Label(frame, text="Toggle:").grid(row=0, column=0)
            ttk.OptionMenu(frame, self.quadrants[i]["toggle"], "Yellow", "Red", "Blue", "Yellow", command=self.update_score).grid(row=0, column=1, columnspan=2)
            
            # Custom Pin Widgets [4]
            self.create_pin_widget(frame, "red", self.quadrants[i]["red_pins"], 1)
            self.create_pin_widget(frame, "blue", self.quadrants[i]["blue_pins"], 2)
            self.create_pin_widget(frame, "yellow", self.quadrants[i]["yellow_pins"], 3)

        # --- Midfield Controls (Placed below the 2x2 grid) [5] ---
        mid_frame = ttk.LabelFrame(input_frame, text="Midfield & Endgame")
        mid_frame.grid(row=3, column=0, columnspan=2, pady=15, padx=10, sticky="ew")
        
        # Grid inside Midfield frame for clean horizontal alignment
        self.create_midfield_input(mid_frame, "Red Robots:", self.red_midfield_robots, 0)
        self.create_midfield_input(mid_frame, "Blue Robots:", self.blue_midfield_robots, 1)
        self.create_midfield_input(mid_frame, "Mid Yellow Pins:", self.midfield_yellow_pins, 2)

        # --- Right Side: Results & Summary ---
        viz_frame = ttk.Frame(self.root, padding="20")
        viz_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.score_label = ttk.Label(viz_frame, text="RED: 0 | BLUE: 0", font=('Helvetica', 28, 'bold'))
        self.score_label.pack(pady=20)

        self.map_text = tk.Text(viz_frame, height=12, width=45, font=('Courier', 10), bg="#f4f4f4", relief="flat")
        self.map_text.pack(padx=10, pady=10)
        
        self.update_score()

    def create_pin_widget(self, parent, color, var, row):
        # Buttons use closures carefully to ensure the correct variable is updated
        tk.Button(parent, text="-", width=2, command=lambda v=var: self.adjust_pin(v, -1)).grid(row=row, column=0)
        
        canvas = tk.Canvas(parent, width=42, height=42, highlightthickness=0)
        canvas.grid(row=row, column=1, padx=8, pady=4)
        
        fill_color = color if color != "yellow" else "#FFD700" 
        canvas.create_oval(4, 4, 38, 38, fill=fill_color, outline="black", width=2)
        
        text_id = canvas.create_text(21, 21, text=str(var.get()), 
                                     fill="white" if color != "yellow" else "black", 
                                     font=('Helvetica', 11, 'bold'))
        
        tk.Button(parent, text="+", width=2, command=lambda v=var: self.adjust_pin(v, 1)).grid(row=row, column=2)
        
        if not hasattr(self, 'canvas_texts'): self.canvas_texts = {}
        self.canvas_texts[str(var)] = (canvas, text_id)

    def create_midfield_input(self, parent, label, var, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, padx=10, pady=2)
        tk.Spinbox(parent, from_=0, to=20, width=5, textvariable=var, command=self.update_score).grid(row=row, column=1, padx=10, pady=2)

    def adjust_pin(self, var, delta):
        new_val = max(0, var.get() + delta)
        var.set(new_val)
        self.update_score()

    def update_score(self, *args):
        red_total = 0
        blue_total = 0
        summary = " FIELD SUMMARY\n " + "="*30 + "\n"

        for i, q in enumerate(self.quadrants):
            # Refresh circle text display
            for key in ["red_pins", "blue_pins", "yellow_pins"]:
                var = q[key]
                if str(var) in self.canvas_texts:
                    canvas, text_id = self.canvas_texts[str(var)]
                    canvas.itemconfig(text_id, text=str(var.get()))

            # Points for Alliance Pins (5 pts each) [1]
            red_total += q["red_pins"].get() * self.VAL_ALLIANCE_PIN
            blue_total += q["blue_pins"].get() * self.VAL_ALLIANCE_PIN
            
            # Points for Yellow Pins (10 pts for Toggle Owner) [6, 7]
            owner = q["toggle"].get()
            if owner == "Red":
                red_total += q["yellow_pins"].get() * self.VAL_YELLOW_PIN
            elif owner == "Blue":
                blue_total += q["yellow_pins"].get() * self.VAL_YELLOW_PIN
            
            summary += f" Q{i+1} [{owner:6}]: R:{q['red_pins'].get():<2} B:{q['blue_pins'].get():<2} Y:{q['yellow_pins'].get():<2}\n"

        # Points for Robots in Midfield (8 pts each) [1]
        r_m, b_m = self.red_midfield_robots.get(), self.blue_midfield_robots.get()
        red_total += r_m * self.VAL_ROBOT_MIDFIELD
        blue_total += b_m * self.VAL_ROBOT_MIDFIELD
        
        # Ownership of Midfield Yellow Pins [7]
        if r_m > b_m:
            red_total += self.midfield_yellow_pins.get() * self.VAL_YELLOW_PIN
        elif b_m > r_m:
            blue_total += self.midfield_yellow_pins.get() * self.VAL_YELLOW_PIN

        self.score_label.config(text=f"RED: {red_total} | BLUE: {blue_total}")
        self.map_text.delete('1.0', tk.END)
        self.map_text.insert(tk.END, summary)

if __name__ == "__main__":
    root = tk.Tk()
    app = OverrideScorer(root)
    root.mainloop()