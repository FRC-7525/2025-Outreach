import json
import os
import re
import sys
import urllib.request

# CONFIG
PIONEERS_LIB_REPO = "https://raw.githubusercontent.com/FRC-7525/PioneersLib/refs/heads/main/vendordeps/"
LOCAL_VENDORDEP_DIR = "vendordeps"



def extract_pioneerslib_version(build_gradle_path):
    with open(build_gradle_path, "r") as file:
        content = file.read()
        match = re.search(r"com\.github\.FRC-7525:pioneerslib:([\d\.]+)", content)
        if match:
            return match.group(1)
        else:
            print("❌ Could not find PioneersLib version in build.gradle")
            sys.exit(1)

def load_vendordep_versions(directory):
    versions = {}
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            path = os.path.join(directory, filename)
            with open(path, "r") as f:
                data = json.load(f)
                name = data.get("name", filename).lower()  # ✅ CHANGED: Case-insensitive name matching
                version = data.get("version", "unknown")
                versions[name] = version
    return versions

def fetch_pioneerslib_vendordeps():
    # THis function needs to be fixed to use the
    pass

def compare_versions(local, pioneers):
    errors = []
    for name, local_version in local.items():
        if name in pioneers:
            pioneers_version = pioneers[name]
            if local_version != pioneers_version:
                errors.append((name, local_version, pioneers_version))
    return errors

def getLatestPioneersLibVersion(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    # Match 'version = ' inside a publication block
    pub_block = re.search(r'publications\s*{.*?mavenJava\(.*?\{(.*?)\}', content, re.DOTALL)
    if pub_block:
        # Now search for version inside the block
        version_match = re.search(r'version\s*=\s*[\'"]([^\'"]+)[\'"]', pub_block.group(1))
        if version_match:
            return version_match.group(1)
    return None

def main():
    print("🔍 Checking teamLib Versions...")

    gradle_file = "../../build.gradle"
    pioneerslib_version = extract_pioneerslib_version(gradle_file)
    print(f"📦 PioneersLib version in build.gradle: {pioneerslib_version}")
    up_to_date_pioneerslib = getLatestPioneersLibVersion(gradle_file)
    if up_to_date_pioneerslib == pioneerslib_version:
        print(f"📦 Latest PioneersLib version is installed: {up_to_date_pioneerslib}")
    elif up_to_date_pioneerslib == None:
        print("⚠️ Could not determine the latest PioneersLib version.")
    else:
        print(f"⚠️ PioneersLib version mismatch: expected {up_to_date_pioneerslib}, found {pioneerslib_version}")
    print("🔍 Comparing vendordep versions...")

    local_versions = load_vendordep_versions(LOCAL_VENDORDEP_DIR)
    pioneers_versions = fetch_pioneerslib_vendordeps()  # ✅ CHANGED: No version passed in

    differences = compare_versions(local_versions, pioneers_versions)

    if differences:
        print("\n❌ Vendordep version mismatches found:")
        for name, local, pion in differences:
            print(f"  - {name}: local = {local}, pioneers = {pion}")
        sys.exit(1)
    else:
        print("✅ All vendordep versions match.")
        sys.exit(0)

if __name__ == "__main__":
    main()