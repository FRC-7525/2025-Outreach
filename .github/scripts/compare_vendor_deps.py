import sys
from pathlib import Path

def list_json_filenames(folder):
    return set(p.name for p in Path(folder).glob("*.json"))

def main(project_folder, library_folder):
    project_files = list_json_filenames(project_folder)
    library_files = list_json_filenames(library_folder)

    missing_in_lib = project_files - library_files
    missing_in_project = library_files - project_files

    if missing_in_lib or missing_in_project:
        if missing_in_lib:
            print(f"Files in project vendordeps but missing in library: {sorted(missing_in_lib)}")
        if missing_in_project:
            print(f"Files in library vendordeps but missing in project: {sorted(missing_in_project)}")
        sys.exit(1)  # Fail CI
    else:
        print("Vendor dependency files match exactly!")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: compare_vendordeps.py <project_vendordeps_folder> <library_vendordeps_folder>")
        sys.exit(2)

    main(sys.argv[1], sys.argv[2])
