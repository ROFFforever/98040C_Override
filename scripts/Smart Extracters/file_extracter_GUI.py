import os
import shutil
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

class ExtractorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Project Hierarchy Extractor")
        self.root.geometry("700x500")
        self.root.minsize(500, 400)
        
        # State variables
        self.source_root = ""
        self.destination_dir = ""
        self.selected_folder_path = ""
        
        self.create_widgets()
        
    def create_widgets(self):
        # Top Frame: Path Selections
        top_frame = ttk.LabelFrame(self.root, text=" 1. Select Directories ", padding=10)
        top_frame.pack(fill="x", padx=10, pady=5)
        
        # Source Row
        ttk.Label(top_frame, text="Project Root:").grid(row=0, column=0, sticky="w", pady=2)
        self.src_entry = ttk.Entry(top_frame, width=55)
        self.src_entry.grid(row=0, column=1, padx=5, pady=2)
        ttk.Button(top_frame, text="Browse...", command=self.browse_source).grid(row=0, column=2, pady=2)
        
        # Destination Row
        ttk.Label(top_frame, text="Extract To:").grid(row=1, column=0, sticky="w", pady=2)
        self.dest_entry = ttk.Entry(top_frame, width=55)
        self.dest_entry.grid(row=1, column=1, padx=5, pady=2)
        ttk.Button(top_frame, text="Browse...", command=self.browse_destination).grid(row=1, column=2, pady=2)
        
        # Middle Frame: Explorer Tree
        mid_frame = ttk.LabelFrame(self.root, text=" 2. Select a Folder to Extract ", padding=10)
        mid_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Scrollbars for Tree
        tree_scroll_y = ttk.Scrollbar(mid_frame, orient="vertical")
        tree_scroll_y.pack(side="right", fill="y")
        tree_scroll_x = ttk.Scrollbar(mid_frame, orient="horizontal")
        tree_scroll_x.pack(side="bottom", fill="x")
        
        # Treeview hierarchy
        self.tree = ttk.Treeview(mid_frame, columns=("path"), show="tree", 
                                 yscrollcommand=tree_scroll_y.set, xscrollcommand=tree_scroll_x.set)
        self.tree.pack(fill="both", expand=True)
        tree_scroll_y.config(command=self.tree.yview)
        tree_scroll_x.config(command=self.tree.xview)
        
        # Bind tree item selection
        self.tree.bind("<<TreeviewSelect>>", self.on_folder_select)
        
        # Bottom Frame: Contextual Action Button
        self.action_frame = ttk.Frame(self.root, height=50, padding=10)
        self.action_frame.pack(fill="x", side="bottom")
        self.action_frame.pack_propagate(False) # Keep fixed height
        
        # Extract button (Hidden by default)
        self.extract_btn = ttk.Button(self.action_frame, text="Extract Folder", 
                                      command=self.run_extraction, style="Accent.TButton")
        
        # Setup modern layout styling options if available
        style = ttk.Style()
        style.configure("Accent.TButton", font=("Helvetica", 10, "bold"))

    def browse_source(self):
        directory = filedialog.askdirectory(title="Select Project Root Path")
        if directory:
            self.source_root = os.path.normpath(directory)
            self.src_entry.delete(0, tk.END)
            self.src_entry.insert(0, self.source_root)
            self.populate_tree()
            self.hide_extract_button()
            
    def browse_destination(self):
        directory = filedialog.askdirectory(title="Select Extraction Destination")
        if directory:
            self.destination_dir = os.path.normpath(directory)
            self.dest_entry.delete(0, tk.END)
            self.dest_entry.insert(0, self.destination_dir)

    def populate_tree(self):
        # Clear existing items
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        if not os.path.exists(self.source_root):
            return
            
        # Add the root node
        root_name = os.path.basename(self.source_root) or self.source_root
        root_node = self.tree.insert("", "end", text=root_name, open=True, values=(self.source_root,))
        
        # Recursively map directories only
        self._append_subdirs_to_tree(self.source_root, root_node)

    def _append_subdirs_to_tree(self, current_dir, parent_node):
        try:
            # Only list items to avoid scanning massive project sub-trees instantly
            for entry in os.scandir(current_dir):
                if entry.is_dir() and not entry.name.startswith('.'): # Skips hidden folders like .git
                    node = self.tree.insert(parent_node, "end", text=entry.name, values=(entry.path,))
                    # Pre-populate next level to allow expansion arrows to show up
                    self._append_subdirs_to_tree(entry.path, node)
        except PermissionError:
            pass

    def on_folder_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items:
            self.hide_extract_button()
            return
            
        # Get absolute path attached to tree item
        item_values = self.tree.item(selected_items[0], "values")
        if item_values:
            self.selected_folder_path = item_values[0]
            # Update extraction button dynamic text and reveal it
            folder_name = os.path.basename(self.selected_folder_path) or self.selected_folder_path
            self.extract_btn.config(text=f"Extract '{folder_name}' Folder")
            self.extract_btn.pack(side="right")
        else:
            self.hide_extract_button()

    def hide_extract_button(self):
        self.extract_btn.pack_forget()
        self.selected_folder_path = ""

    def run_extraction(self):
        # Update destination from manual entry box alterations
        self.destination_dir = os.path.normpath(self.dest_entry.get().strip())
        
        if not self.selected_folder_path:
            messagebox.showwarning("Selection Missing", "Please select a folder from the tree layout first.")
            return
            
        if not self.destination_dir:
            messagebox.showwarning("Destination Missing", "Please select or type an extraction destination folder path.")
            return

        # Double check existence
        os.makedirs(self.destination_dir, exist_ok=True)
        
        success_count = 0
        error_count = 0

        # Run extraction exclusively inside the selected subfolder
        for root_dir, dirs, files in os.walk(self.selected_folder_path):
            for file in files:
                source_file_path = os.path.join(root_dir, file)
                
                # Calculate relative path relative to the SELECTED folder
                rel_path = os.path.relpath(source_file_path, self.selected_folder_path)
                
                # Create flat naming scheme
                safe_file_name = rel_path.replace(os.sep, '_')
                file_base, _ = os.path.splitext(safe_file_name)
                safe_file_name_txt = file_base + ".txt"
                
                destination_file_path = os.path.join(self.destination_dir, safe_file_name_txt)
                
                try:
                    shutil.copy2(source_file_path, destination_file_path)
                    
                    # Intercept text formatting to write relative paths
                    with open(destination_file_path, 'r+', encoding='utf-8', errors='ignore') as f:
                        content = f.read()
                        f.seek(0, 0)
                        f.write(f"// Original Path: {rel_path}\n\n" + content)
                    success_count += 1
                except Exception:
                    error_count += 1

        # Summary Toast Message
        folder_title = os.path.basename(self.selected_folder_path) or self.selected_folder_path
        msg = f"Extraction complete for '{folder_title}'!\n\nSuccessfully converted: {success_count} files."
        if error_count > 0:
            msg += f"\nFailed/Skipped: {error_count} items."
        
        messagebox.showinfo("Success", msg)

if __name__ == "__main__":
    app_root = tk.Tk()
    gui = ExtractorGUI(app_root)
    app_root.mainloop()
