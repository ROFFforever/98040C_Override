import os
import shutil

def extract_files_and_preserve_hierarchy(source_dir, destination_dir):
    # Ensure the destination directory exists
    os.makedirs(destination_dir, exist_ok=True)
    
    # Recursively traverse through all files in the source directory
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            source_file_path = os.path.join(root, file)
            
            # Calculate the relative path of the file from the source directory
            rel_path = os.path.relpath(source_file_path, source_dir)
            
            # Create a unique, flat filename for the destination by replacing path separators
            safe_file_name = rel_path.replace(os.sep, '_')
            
            # Split the filename from its original extension and force it to be .txt
            file_base, _ = os.path.splitext(safe_file_name)
            safe_file_name_txt = file_base + ".txt"
            
            destination_file_path = os.path.join(destination_dir, safe_file_name_txt)
            
            try:
                # Copy file and its metadata to the flat destination
                shutil.copy2(source_file_path, destination_file_path)
                
                # Prepend the original relative path to the top of the newly copied file
                with open(destination_file_path, 'r+', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                    f.seek(0, 0)
                    f.write(f"// Original Path: {rel_path}\n\n" + content)
                    
                print(f"Processed: {rel_path} -> {safe_file_name_txt}")
            except Exception as e:
                print(f"Error processing {rel_path}: {e}")

# --- Configuration with your exact paths ---
SOURCE_PROJECT_FOLDER = r'C:/dev/robotics/MOA/echo/include'
FLATTENED_DESTINATION_FOLDER = r'C:/Users/User/Downloads/Context'

if __name__ == "__main__":
    print(f"Source: {SOURCE_PROJECT_FOLDER}")
    print(f"Destination: {FLATTENED_DESTINATION_FOLDER}\n")
    
    extract_files_and_preserve_hierarchy(SOURCE_PROJECT_FOLDER, FLATTENED_DESTINATION_FOLDER)
    print("\nExtraction complete!")
