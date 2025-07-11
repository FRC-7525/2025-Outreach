import sys
from pathlib import Path

def list_json_filenames(folder):
    p = Path(folder)
    if not p.exists():
        print(f"Warning: Folder {folder} does not exist.")
        return set()
    return set(f.name for f in p.glob("*.json"))

def main(project_folder, library_folder):
    project_files = list_json_filenames(project_folder)
    library_files = list_json_filenames(library_folder)

    missing_from_project = library_files - project_files

    if missing_from_project:
        print(f"ERROR: The following vendordep files exist in the library but are missing from the project:")
        for f in sorted(missing_from_project):
            print(f"  - {f}")
        sys.exit(1)
    else:
        print("Success: All library vendordep files exist in the project.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: compare_vendordeps.py <project_vendordeps_folder> <library_vendordeps_folder>")
        sys.exit(2)

    main(sys.argv[1], sys.argv[2])
