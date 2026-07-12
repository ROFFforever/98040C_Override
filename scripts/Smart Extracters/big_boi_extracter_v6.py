#THE PURPOSE OF THIS SCRIPT IS TO EXTRACT RELEVANT FILES FROM A CODE PROJECT INTO SIMPLE TXT FILES, SO THAT THESE CAN BE INPUTTED INTO LLM'S AS CONTEXT WITH EASE

import json
import os
import shutil
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
API_KEY = "AQ.Ab8RN6K8cwNTGB98xw844JwcosPJsIVFEEXVW7l5HXC9vkEApQ"
try:
    from google import genai
    GENAI_AVAILABLE = True
except ImportError:
    GENAI_AVAILABLE = False

# Max file size limit in bytes (200MB) and word count (500,000 words)
MAX_FILE_SIZE = 200 * 1024 * 1024 
MAX_WORD_COUNT = 350_000

# Directories to ignore during traversal (build artifacts, dependencies, hidden dirs)
IGNORED_DIRS = {
    '.git', '.svn', '.hg', '__pycache__', 'node_modules', '.venv', 'venv', 'env', 
    '.env', 'dist', 'build', 'target', 'out', '.idea', '.vscode', '.next', '.nuxt', 
    'coverage', '.mypy_cache', '.pytest_cache', '.tox', 'eggs', '.eggs', 'vendor'
}

# File extensions to ignore (Images, binaries, archives, audio/video, fonts, documents)
IGNORED_EXTENSIONS = {
    # Images
    '.png', '.jpg', '.jpeg', '.gif', '.bmp', '.webp', '.svg', '.ico', '.tiff', '.tif', 
    '.psd', '.ai', '.eps', '.raw', '.heic', '.heif', '.avif', '.icns', '.cur',
    # Audio / Video
    '.mp3', '.mp4', '.wav', '.ogg', '.flac', '.aac', '.mov', '.avi', '.mkv', '.webm', 
    '.m4a', '.wma', '.flv', '.wmv',
    # Archives / Compressed / Binaries
    '.zip', '.tar', '.gz', '.tgz', '.rar', '.7z', '.bz2', '.xz', '.exe', '.dll', '.so', 
    '.dylib', '.bin', '.dat', '.obj', '.o', '.a', '.lib', '.pyc', '.pyo', '.pyd', 
    '.iso', '.dmg', '.pkg', '.deb', '.rpm', '.apk',
    # Documents / Proprietary formats
    '.pdf', '.doc', '.docx', '.xls', '.xlsx', '.ppt', '.pptx', '.odt', '.ods', '.odp', '.epub',
    # Databases / Fonts / Miscellaneous binaries
    '.sqlite', '.sqlite3', '.db', '.mdb', '.accdb', '.ttf', '.otf', '.woff', '.woff2', 
    '.eot', '.keystore', '.pem', '.crt', '.der', '.pyi', '.idx', '.pack'
}

# Specific filenames to always ignore (system files, large lockfiles that waste LLM tokens)
IGNORED_FILENAMES = {
    '.DS_Store', 'Thumbs.db', 'desktop.ini', 
    'package-lock.json', 'yarn.lock', 'pnpm-lock.yaml', 'poetry.lock', 'Cargo.lock', 'Gemfile.lock'
}

def is_binary_file(file_path, blocksize=1024):
    """Check if a file is binary by searching for null bytes in the first block."""
    try:
        with open(file_path, 'rb') as f:
            chunk = f.read(blocksize)
            if b'\x00' in chunk:
                return True
    except Exception:
        return True  # Treat unreadable files as binary/ignored
    return False

class ExtractorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Project Hierarchy Extractor (Smart Packer with AI Pruning)")
        self.root.geometry("720x600")
        self.root.minsize(550, 480)
        
        self.source_root = ""
        self.destination_dir = ""
        self.selected_folder_path = ""
        self.ai_excluded_paths = set()
        
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
        
        # AI Smart Pruning Frame
        ai_frame = ttk.LabelFrame(self.root, text=" 2. AI Smart Pruning (Granular Context Optimization) ", padding=10)
        ai_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(ai_frame, text="Gemini API Key:").grid(row=0, column=0, sticky="w", pady=2)
        self.api_key_entry = ttk.Entry(ai_frame, width=38)
        self.api_key_entry.grid(row=0, column=1, padx=5, pady=2)
        
        # Pre-populate with environment variable if present
        default_key = os.environ.get("GEMINI_API_KEY", os.environ.get("GOOGLE_API_KEY", ""))
        if default_key:
            self.api_key_entry.insert(0, default_key)
            
        self.ai_btn = ttk.Button(ai_frame, text="🤖 AI Suggest Redundant Items", 
                                 command=self.run_ai_prune_analysis)
        self.ai_btn.grid(row=0, column=2, padx=5, pady=2)
        
        self.ai_status_label = ttk.Label(ai_frame, text="AI Exclusions: Inactive (0 specific files/folders excluded)", 
                                         foreground="#555555")
        self.ai_status_label.grid(row=1, column=0, columnspan=3, sticky="w", pady=(4, 0))
        
        # Middle Frame: Explorer Tree
        mid_frame = ttk.LabelFrame(self.root, text=" 3. Select a Folder to Extract ", padding=10)
        mid_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        tree_scroll_y = ttk.Scrollbar(mid_frame, orient="vertical")
        tree_scroll_y.pack(side="right", fill="y")
        tree_scroll_x = ttk.Scrollbar(mid_frame, orient="horizontal")
        tree_scroll_x.pack(side="bottom", fill="x")
        
        self.tree = ttk.Treeview(mid_frame, columns=("path"), show="tree", 
                                 yscrollcommand=tree_scroll_y.set, xscrollcommand=tree_scroll_x.set)
        self.tree.pack(fill="both", expand=True)
        tree_scroll_y.config(command=self.tree.yview)
        tree_scroll_x.config(command=self.tree.xview)
        
        self.tree.bind("<<TreeviewSelect>>", self.on_folder_select)
        
        # Bottom Frame: Action Buttons (packed side='bottom' first so it sits at the very bottom)
        self.action_frame = ttk.Frame(self.root, height=50, padding=10)
        self.action_frame.pack(fill="x", side="bottom")
        self.action_frame.pack_propagate(False)
        
        # Options Bar (packed side='bottom' second so it sits just above action_frame)
        options_frame = ttk.Frame(self.root, padding=(10, 0, 10, 5))
        options_frame.pack(fill="x", side="bottom")
        
        # Filter Checkbox (default True to skip images, binaries & non-code)
        self.filter_var = tk.BooleanVar(value=True)
        self.filter_cb = ttk.Checkbutton(options_frame, 
                                         text="Filter out images, binaries & non-code files (recommended for LLMs)", 
                                         variable=self.filter_var)
        self.filter_cb.pack(side="left")
        
        # Finished Extracting Button (Bottom Left)
        self.finished_btn = ttk.Button(self.action_frame, text="Finished extracting", 
                                       command=self.finish_extraction_job, style="Finished.TButton")
        self.finished_btn.pack(side="left")
        
        # Extract Folder Button (Bottom Right)
        self.extract_btn = ttk.Button(self.action_frame, text="Extract Folder", 
                                      command=self.run_extraction, style="Accent.TButton")
        
        style = ttk.Style()
        style.configure("Accent.TButton", font=("Helvetica", 10, "bold"))
        style.configure("Finished.TButton", font=("Helvetica", 10, "bold"))

    def is_ai_excluded(self, full_path, is_dir=False):
        """Intelligently checks if a file or directory matches any of the AI-recommended exclusions."""
        if not self.ai_excluded_paths:
            return False
            
        norm_full = os.path.normpath(full_path)
        basename = os.path.basename(norm_full)
        
        # If it's a directory and its basename (like 'tests', 'docs', 'examples') was explicitly excluded
        if is_dir and basename in self.ai_excluded_paths:
            return True
            
        # Check relative path against project root and selected folder
        for base_dir in [self.source_root, self.selected_folder_path]:
            if not base_dir or not os.path.exists(base_dir):
                continue
            try:
                rel_path = os.path.normpath(os.path.relpath(norm_full, base_dir))
                if rel_path in self.ai_excluded_paths:
                    return True
                # Check if any parent folder in the relative path is excluded
                parts = rel_path.split(os.sep)
                for i in range(1, len(parts)):
                    sub_path = os.sep.join(parts[:i])
                    if sub_path in self.ai_excluded_paths or parts[i-1] in self.ai_excluded_paths:
                        return True
            except ValueError:
                pass
                
        # Fallback: check if the normalized path ends with any of the excluded relative paths
        for excl in self.ai_excluded_paths:
            excl_norm = os.path.normpath(excl)
            if norm_full.endswith(os.sep + excl_norm) or norm_full == excl_norm:
                return True
            if is_dir and basename == excl_norm:
                return True
            if not is_dir and basename == excl_norm and os.sep not in excl_norm:
                return True
                
        return False

    def generate_project_tree_str(self, root_dir, max_depth=6):
        """Generates a clean hierarchy tree of folders and files for LLM context analysis."""
        tree_lines = [f"{os.path.basename(root_dir)}/"]
        
        for root, dirs, files in os.walk(root_dir):
            # Skip ignored or already-excluded directories
            dirs[:] = [
                d for d in dirs 
                if not d.startswith('.') 
                and d not in IGNORED_DIRS 
                and not self.is_ai_excluded(os.path.join(root, d), is_dir=True)
            ]
            
            rel_path = os.path.relpath(root, root_dir)
            if rel_path == ".":
                depth = 0
            else:
                depth = rel_path.count(os.sep) + 1
                
            if depth > max_depth:
                dirs[:] = []
                continue
                
            if rel_path != ".":
                indent = "  " * depth
                tree_lines.append(f"{indent}{os.path.basename(root)}/")
                
            file_indent = "  " * (depth + 1)
            valid_files = []
            for file in sorted(files):
                file_full = os.path.join(root, file)
                if file.startswith('.') or file in IGNORED_FILENAMES or self.is_ai_excluded(file_full, is_dir=False):
                    continue
                _, ext = os.path.splitext(file)
                if ext.lower() in IGNORED_EXTENSIONS:
                    continue
                valid_files.append(file)
                
            for file in valid_files:
                tree_lines.append(f"{file_indent}/{file}")
                
        return "\n".join(tree_lines)

    def run_ai_prune_analysis(self):
        """Uses Google GenAI Gemini 2.5 Flash to identify redundant files/folders for LLM context loading."""
        if not GENAI_AVAILABLE:
            messagebox.showerror("SDK Missing", "The 'google-genai' library is required for AI Smart Pruning.\n\nPlease install it via terminal:\npip install google-genai")
            return

        api_key = self.api_key_entry.get().strip()
        if not api_key:
            messagebox.showwarning("API Key Required", "Please enter a valid Google Gemini API Key to run AI pruning.")
            return
            
        target_dir = self.selected_folder_path or self.source_root
        if not target_dir or not os.path.exists(target_dir):
            messagebox.showwarning("Directory Required", "Please browse and select a Project Root path first.")
            return
            
        self.ai_btn.config(state="disabled", text="🤖 Analyzing with Gemini...")
        
        def _analyze_thread():
            try:
                tree_str = self.generate_project_tree_str(target_dir)
                
                prompt = (
                    "# THE PURPOSE OF THIS MESSAGE IS TO EXTRACT RELEVANT FILES AND FOLDERS FROM A PROJECT DIRECTORY, "
                    "SO THAT THESE CAN BE INPUTTED INTO LLM'S AS CONTEXT WITH EASE.\n\n"
                    "You are an expert software architect and AI context optimization specialist. "
                    "Provided below is the root directory hierarchy of a software project. "
                    "Analyze this architecture and identify redundant, boilerplate, generated, test, documentation, "
                    "or peripheral items that can be removed without losing core structural or algorithmic context for an LLM.\n\n"
                    "CRITICAL PRUNING STRATEGY:\n"
                    "1. PREFER FILE-LEVEL GRANULARITY: Most times, choose specific irrelevant files within folders rather than dropping an entire folder. For example, in a 'utils/' or 'services/' or 'components/' folder, keep the core logic files but exclude mock helpers, temporary scripts, deprecated files, or boilerplate setup files.\n"
                    "2. ONLY DROP ENTIRE FOLDERS IF TRULY POINTLESS: Only exclude an entire folder (such as 'tests/', 'docs/', 'examples/', 'fixtures/', 'assets/', or 'coverage/') if every single file inside that folder is completely irrelevant for understanding the core application logic.\n"
                    "3. TARGET IRRELEVANT ITEMS: Look for test suites, linting/formatting configs, migration scripts, mock data, static assets, boilerplate setups, or standalone helper scripts that add noise without algorithmic value.\n\n"
                    "INSTRUCTIONS:\n"
                    "1. ONLY respond with a valid JSON list of strings representing the relative file or folder paths to be excluded (e.g., [\"tests\", \"docs\", \"src/utils/mock_helper.py\", \"api/legacy_handler.py\", \"config/local.env.example\"]).\n"
                    "2. Do NOT include any explanations, introductory text, or markdown formatting outside the JSON list.\n"
                    "3. Ensure the dropped file or folder paths genuinely exist in the provided directory tree.\n\n"
                    f"Here is the project directory, f'PROJECT_DIRECTORY':\n{tree_str}"
                )
                
                prompt_2 = (
                    "# THE PURPOSE OF THIS MESSAGE IS TO EXTRACT RELEVANT FILES AND FOLDERS FROM A PROJECT DIRECTORY, "
                    "SO THAT THESE CAN BE INPUTTED INTO LLM'S AS CONTEXT WITH EASE.\n\n"
                    "You are an expert software architect and AI context optimization specialist. "
                    "Provided below is the root directory hierarchy of a software project. "
                    "Analyze this architecture and identify redundant, boilerplate, generated, test, documentation, "
                    "or peripheral items that can be removed without losing core structural or algorithmic context for an LLM.\n\n"
                    "PRIORITIZE ACUTAL LOGIC OR ALGORITHMIC FILES: Focus on keeping files that contain core logic, algorithms, or unique implementations. Exclude files that are purely for testing, documentation, configuration, or auxiliary purposes, or even just library files to a certain extent.\n\n"
                    "CRITICAL PRUNING STRATEGY:\n"
                    "1. PREFER FILE-LEVEL GRANULARITY: Most times, choose specific irrelevant files within folders rather than dropping an entire folder. For example, in a 'utils/' or 'services/' or 'components/' folder, keep the core logic files but exclude mock helpers, temporary scripts, deprecated files, or boilerplate setup files.\n"
                    "2. ONLY DROP ENTIRE FOLDERS IF TRULY POINTLESS: Only exclude an entire folder (such as 'tests/', 'docs/', 'examples/', 'fixtures/', 'assets/', or 'coverage/') if every single file inside that folder is completely irrelevant for understanding the core application logic.\n"
                    "3. TARGET IRRELEVANT ITEMS: Look for test suites, linting/formatting configs, migration scripts, mock data, static assets, boilerplate setups, or standalone helper scripts that add noise without algorithmic value.\n\n"
                    "INSTRUCTIONS:\n"
                    "1. ONLY respond with a valid JSON list of strings representing the relative file or folder paths to be excluded (e.g., [\"tests\", \"docs\", \"src/utils/mock_helper.py\", \"api/legacy_handler.py\", \"config/local.env.example\"]).\n"
                    "2. Do NOT include any explanations, introductory text, or markdown formatting outside the JSON list.\n"
                    "3. Ensure the dropped file or folder paths genuinely exist in the provided directory tree.\n\n"
                    f"Here is the project directory, f'PROJECT_DIRECTORY':\n{tree_str}"
                )
                
                client = genai.Client(api_key=api_key)
                response = client.models.generate_content(
                    model='gemini-2.5-flash',
                    contents=prompt_2,
                )
                
                output_text = response.text.strip()
                # Clean markdown code block formatting if present
                if output_text.startswith("```"):
                    lines = output_text.splitlines()
                    output_text = "\n".join([l for l in lines if not l.startswith("```")]).strip()
                    
                suggested_exclusions = json.loads(output_text)
                if not isinstance(suggested_exclusions, list):
                    suggested_exclusions = []
                
                self.root.after(0, self._on_ai_analysis_success, suggested_exclusions)
                
            except Exception as e:
                err_msg = str(e)
                self.root.after(0, self._on_ai_analysis_error, err_msg)
                
        threading.Thread(target=_analyze_thread, daemon=True).start()

    def _on_ai_analysis_success(self, suggested_exclusions):
        self.ai_btn.config(state="normal", text="🤖 AI Suggest Redundant Items")
        
        if not suggested_exclusions:
            messagebox.showinfo("AI Pruning Analysis", "Gemini analyzed the project hierarchy and did not find any redundant items to exclude. Your structure looks concise!")
            return
            
        # Format recommendations for display
        items_str = "\n".join([f"• {item}" for item in suggested_exclusions[:20]])
        if len(suggested_exclusions) > 20:
            items_str += f"\n...and {len(suggested_exclusions) - 20} more."
            
        msg = (
            f"Gemini AI identified {len(suggested_exclusions)} redundant item(s) (preferring granular file-level pruning over blanket folder removal):\n\n"
            f"{items_str}\n\n"
            "Would you like to automatically exclude these items from extraction?"
        )
        
        if messagebox.askyesno("AI Pruning Recommendations", msg):
            for item in suggested_exclusions:
                clean_item = os.path.normpath(item.strip()).strip(os.sep)
                self.ai_excluded_paths.add(clean_item)
                
            self.ai_status_label.config(
                text=f"AI Exclusions Active: {len(self.ai_excluded_paths)} specific files/folders excluded", 
                foreground="#2e7d32"
            )
            self.populate_tree()
            messagebox.showinfo("Exclusions Applied", f"Added {len(suggested_exclusions)} AI-recommended exclusions! The explorer tree has been pruned and extraction will skip these paths.")

    def _on_ai_analysis_error(self, error_msg):
        self.ai_btn.config(state="normal", text="🤖 AI Suggest Redundant Items")
        messagebox.showerror("AI Pruning Error", f"Failed to analyze project with Gemini:\n\n{error_msg}")

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
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        if not os.path.exists(self.source_root):
            return
            
        root_name = os.path.basename(self.source_root) or self.source_root
        root_node = self.tree.insert("", "end", text=root_name, open=True, values=(self.source_root,))
        self._append_subdirs_to_tree(self.source_root, root_node)

    def _append_subdirs_to_tree(self, current_dir, parent_node):
        try:
            for entry in os.scandir(current_dir):
                if (entry.is_dir() and not entry.name.startswith('.') and 
                    entry.name not in IGNORED_DIRS and 
                    not self.is_ai_excluded(entry.path, is_dir=True)):
                    node = self.tree.insert(parent_node, "end", text=entry.name, values=(entry.path,))
                    self._append_subdirs_to_tree(entry.path, node)
        except PermissionError:
            pass

    def on_folder_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items:
            self.hide_extract_button()
            return
            
        item_values = self.tree.item(selected_items, "values")
        if item_values:
            self.selected_folder_path = item_values[0] # Extract string from tuple safely
            folder_name = os.path.basename(self.selected_folder_path) or self.selected_folder_path
            self.extract_btn.config(text=f"Extract '{folder_name}' Folder")
            self.extract_btn.pack(side="right")
        else:
            self.hide_extract_button()

    def hide_extract_button(self):
        self.extract_btn.pack_forget()
        self.selected_folder_path = ""

    def run_extraction(self):
        self.destination_dir = os.path.normpath(self.dest_entry.get().strip())
        
        if not self.selected_folder_path:
            messagebox.showwarning("Selection Missing", "Please select a folder from the tree layout first.")
            return
            
        if not self.destination_dir:
            messagebox.showwarning("Destination Missing", "Please select or type an extraction destination folder path.")
            return

        os.makedirs(self.destination_dir, exist_ok=True)
        
        folder_base_name = os.path.basename(self.selected_folder_path) or "extracted"
        
        combined_file_index = 1
        current_combined_path = os.path.join(self.destination_dir, f"{folder_base_name}_combined_{combined_file_index}.txt")
        current_combined_size = 0
        current_combined_words = 0
        
        processed_files = 0
        skipped_files = 0
        split_files_count = 0
        error_count = 0
        
        enable_filtering = self.filter_var.get()

        # Crawl selected folder
        for root_dir, dirs, files in os.walk(self.selected_folder_path):
            # Modify dirs in-place to skip hidden directories and ignored build/dependency directories
            if enable_filtering:
                dirs[:] = [
                    d for d in dirs 
                    if not d.startswith('.') 
                    and d not in IGNORED_DIRS 
                    and not self.is_ai_excluded(os.path.join(root_dir, d), is_dir=True)
                ]
                
            for file in files:
                source_file_path = os.path.join(root_dir, file)
                rel_path = os.path.relpath(source_file_path, self.selected_folder_path)
                
                if enable_filtering:
                    # 1. Skip system files, hidden files, and massive token-wasting lockfiles / AI exclusions
                    if (file in IGNORED_FILENAMES or file.startswith('.') or 
                        self.is_ai_excluded(source_file_path, is_dir=False)):
                        skipped_files += 1
                        continue
                        
                    # 2. Skip images, binaries, archives, audio, video, etc. by extension
                    _, ext = os.path.splitext(file)
                    if ext.lower() in IGNORED_EXTENSIONS:
                        skipped_files += 1
                        continue
                        
                    # 3. Check for binary content (catches extensionless images or compiled binaries)
                    if is_binary_file(source_file_path):
                        skipped_files += 1
                        continue
                
                try:
                    # Check original file size
                    file_size = os.path.getsize(source_file_path)
                    
                    with open(source_file_path, 'r', encoding='utf-8', errors='ignore') as f:
                        file_content = f.read()
                        
                    file_words = len(file_content.split())
                    
                    # 1. HANDLE OVERSIZED FILES (> 200MB OR > 500,000 words) -> SPLIT THEM
                    if file_size > MAX_FILE_SIZE or file_words > MAX_WORD_COUNT:
                        split_files_count += self.split_large_file(source_file_path, rel_path)
                        processed_files += 1
                        continue
                        
                    # FIXED: Added explicitly labeled "Original Path" lines inside the chunk headers
                    segment_header = f"\n\n{'='*80}\n// START OF FILE\n// Original Path: {rel_path}\n{'='*80}\n\n"
                    segment_footer = f"\n\n{'='*80}\n// END OF FILE\n// Original Path: {rel_path}\n{'='*80}\n"
                    full_segment = segment_header + file_content + segment_footer
                    segment_bytes_len = len(full_segment.encode('utf-8'))
                    segment_word_count = len(full_segment.split())
                    
                    # 2. HANDLE COMBINING FILES (< 200MB AND < 500,000 words)
                    # Stop combining if EITHER 500 thousand words OR 200MB is exceeded
                    if (current_combined_size + segment_bytes_len > MAX_FILE_SIZE or 
                        current_combined_words + segment_word_count > MAX_WORD_COUNT) and current_combined_size > 0:
                        combined_file_index += 1
                        current_combined_path = os.path.join(self.destination_dir, f"{folder_base_name}_combined_{combined_file_index}.txt")
                        current_combined_size = 0
                        current_combined_words = 0
                        
                    # Append data chunk to the active file segment
                    with open(current_combined_path, 'a', encoding='utf-8') as cf:
                        cf.write(full_segment)
                        
                    current_combined_size += segment_bytes_len
                    current_combined_words += segment_word_count
                    processed_files += 1
                    
                except Exception as e:
                    print(f"Error packing {rel_path}: {e}")
                    error_count += 1

        # Summary Feedback
        folder_title = os.path.basename(self.selected_folder_path) or self.selected_folder_path
        msg = f"Extraction complete for '{folder_title}'!\n\n"
        msg += f"Processed files: {processed_files}\n"
        if skipped_files > 0:
            msg += f"Skipped non-text/ignored/AI-excluded files: {skipped_files}\n"
        if split_files_count > 0:
            msg += f"Oversized files split up: {split_files_count}\n"
        if error_count > 0:
            msg += f"Errors encountered: {error_count}\n"
            
        messagebox.showinfo("Smart Extraction Success", msg)

    def split_large_file(self, source_path, rel_path):
        """Splits an oversized file into chunks under 200MB and under 500,000 words with structured headers."""
        safe_file_name = rel_path.replace(os.sep, '_')
        file_base, _ = os.path.splitext(safe_file_name)
        
        chunk_num = 1
        chunks_created = 0
        current_chunk_lines = []
        current_chunk_bytes = 0
        current_chunk_words = 0
        
        # Use safety limits slightly below max to allow room for headers
        max_bytes = MAX_FILE_SIZE - 50_000
        max_words = MAX_WORD_COUNT - 1_000
        
        with open(source_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line_bytes = len(line.encode('utf-8'))
                line_words = len(line.split())
                
                # If a single line itself is larger than the limit, break it by words
                if line_bytes > max_bytes or line_words > max_words:
                    if current_chunk_lines:
                        self._write_split_chunk(file_base, chunk_num, rel_path, current_chunk_lines)
                        chunk_num += 1
                        chunks_created += 1
                        current_chunk_lines = []
                        current_chunk_bytes = 0
                        current_chunk_words = 0
                    
                    words = line.split(' ')
                    temp_words = []
                    temp_bytes = 0
                    temp_count = 0
                    for w in words:
                        w_str = w + ' '
                        w_bytes = len(w_str.encode('utf-8'))
                        if (temp_bytes + w_bytes > max_bytes or temp_count + 1 > max_words) and temp_words:
                            self._write_split_chunk(file_base, chunk_num, rel_path, [ "".join(temp_words) ])
                            chunk_num += 1
                            chunks_created += 1
                            temp_words = []
                            temp_bytes = 0
                            temp_count = 0
                        temp_words.append(w_str)
                        temp_bytes += w_bytes
                        temp_count += 1
                    if temp_words:
                        current_chunk_lines = [ "".join(temp_words) ]
                        current_chunk_bytes = temp_bytes
                        current_chunk_words = temp_count
                    continue

                if (current_chunk_bytes + line_bytes > max_bytes or 
                    current_chunk_words + line_words > max_words) and current_chunk_lines:
                    
                    self._write_split_chunk(file_base, chunk_num, rel_path, current_chunk_lines)
                    chunk_num += 1
                    chunks_created += 1
                    current_chunk_lines = []
                    current_chunk_bytes = 0
                    current_chunk_words = 0
                    
                current_chunk_lines.append(line)
                current_chunk_bytes += line_bytes
                current_chunk_words += line_words
                
            if current_chunk_lines or chunks_created == 0:
                self._write_split_chunk(file_base, chunk_num, rel_path, current_chunk_lines)
                chunks_created += 1
                
        return chunks_created

    def _write_split_chunk(self, file_base, chunk_num, rel_path, lines):
        chunk_filename = f"{file_base}_part_{chunk_num}.txt"
        chunk_dest_path = os.path.join(self.destination_dir, chunk_filename)
        segment_header = (
            f"{'=' * 80}\n"
            f"// SPLIT FILE PART {chunk_num}\n"
            f"// Original Path: {rel_path}\n"
            f"{'=' * 80}\n\n"
        )
        with open(chunk_dest_path, "w", encoding="utf-8") as out_f:
            out_f.write(segment_header + "".join(lines))

    def finish_extraction_job(self):
        """Combines all extracted files in the destination directory into consolidated files capped at 500k words / 200MB."""
        self.destination_dir = os.path.normpath(self.dest_entry.get().strip())
        if not self.destination_dir or not os.path.exists(self.destination_dir):
            messagebox.showwarning("Destination Missing", "Please select or extract to a valid destination directory first.")
            return

        # Gather all regular files in destination directory
        files_to_combine = []
        for item in sorted(os.listdir(self.destination_dir)):
            full_path = os.path.join(self.destination_dir, item)
            if os.path.isfile(full_path) and not item.startswith('.'):
                files_to_combine.append(full_path)

        if not files_to_combine:
            messagebox.showinfo("No Files Found", "No files found in the destination directory to combine.")
            return

        # Create a temporary directory for generating consolidated files safely
        temp_dir = os.path.join(self.destination_dir, "_temp_consolidating")
        os.makedirs(temp_dir, exist_ok=True)

        package_index = 1
        current_package_path = os.path.join(temp_dir, f"final_combined_output_{package_index}.txt")
        current_package_bytes = 0
        current_package_words = 0

        for file_path in files_to_combine:
            try:
                with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
            except Exception as e:
                print(f"Error reading {file_path}: {e}")
                continue

            file_name = os.path.basename(file_path)
            
            # Clearly segment each combined file with an ASCII banner
            segment_header = (
                f"\n\n{'#' * 80}\n"
                f"# START OF EXTRACTED PACKAGE SECTION: {file_name}\n"
                f"{'#' * 80}\n\n"
            )
            segment_footer = (
                f"\n\n{'#' * 80}\n"
                f"# END OF EXTRACTED PACKAGE SECTION: {file_name}\n"
                f"{'#' * 80}\n"
            )
            full_segment = segment_header + content + segment_footer
            segment_bytes = len(full_segment.encode('utf-8'))
            segment_words = len(full_segment.split())

            # Check if adding this file exceeds EITHER 200MB OR 500,000 words
            if (current_package_bytes + segment_bytes > MAX_FILE_SIZE or 
                current_package_words + segment_words > MAX_WORD_COUNT) and current_package_bytes > 0:
                package_index += 1
                current_package_path = os.path.join(temp_dir, f"final_combined_output_{package_index}.txt")
                current_package_bytes = 0
                current_package_words = 0

            # Handle edge case where a single segment by itself exceeds the limits
            if segment_bytes > MAX_FILE_SIZE or segment_words > MAX_WORD_COUNT:
                if current_package_bytes > 0:
                    package_index += 1
                    current_package_path = os.path.join(temp_dir, f"final_combined_output_{package_index}.txt")
                    current_package_bytes = 0
                    current_package_words = 0
                
                lines = full_segment.splitlines(keepends=True)
                for line in lines:
                    l_bytes = len(line.encode('utf-8'))
                    l_words = len(line.split())
                    if (current_package_bytes + l_bytes > MAX_FILE_SIZE - 10000 or 
                        current_package_words + l_words > MAX_WORD_COUNT - 1000) and current_package_bytes > 0:
                        package_index += 1
                        current_package_path = os.path.join(temp_dir, f"final_combined_output_{package_index}.txt")
                        current_package_bytes = 0
                        current_package_words = 0
                    with open(current_package_path, 'a', encoding='utf-8') as pf:
                        pf.write(line)
                    current_package_bytes += l_bytes
                    current_package_words += l_words
            else:
                with open(current_package_path, 'a', encoding='utf-8') as pf:
                    pf.write(full_segment)
                current_package_bytes += segment_bytes
                current_package_words += segment_words

        # Replace original files in destination_dir with the newly consolidated output files
        for old_file in files_to_combine:
            try:
                os.remove(old_file)
            except Exception as e:
                print(f"Could not remove {old_file}: {e}")

        for new_file in sorted(os.listdir(temp_dir)):
            src_p = os.path.join(temp_dir, new_file)
            dst_p = os.path.join(self.destination_dir, new_file)
            try:
                shutil.move(src_p, dst_p)
            except Exception as e:
                print(f"Could not move {src_p}: {e}")

        try:
            os.rmdir(temp_dir)
        except Exception:
            pass

        messagebox.showinfo(
            "Extraction Job Completed",
            f"Successfully finished extraction job!\n\n"
            f"Consolidated {len(files_to_combine)} source file(s) into {package_index} final output file(s):\n"
            f"• Capped at max 500,000 words or 200MB per file.\n"
            f"• Each source file is cleanly segmented with ASCII banners."
        )


if __name__ == "__main__":
    app_root = tk.Tk()
    gui = ExtractorGUI(app_root)
    app_root.mainloop()
