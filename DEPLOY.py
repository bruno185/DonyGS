#
# Python deployment script for Apple IIGS
# Uses Golden Gate for compilation and Cadius for disk image updates
#

import os
import sys
import subprocess
from pathlib import Path

# ===============================================
#           Configuration Variables
# ===============================================
# Change these variables to your own configuration
PRG = "OBJExplorer"  # Program name without extension
lang = ".cc"
AppleDiskPath = "F:\\Bruno\\Dev\\AppleWin\\GS\\activeGS\\Live.Install.po"
ProdosDir = "/LIVE.INSTALL/OBJ/"
# Local workspace disk image
LocalDiskPath = "OBJ.po"
LocalProdosDir = "/ORCA/"
# ===============================================
#
def run_command(command, description=""):
    """Execute a shell command and print its stdout/stderr.

    This helper is used for Golden Gate commands and reports
    any failure with the provided description.
    """
    print(f"Executing: {command}")
    try:
        result = subprocess.run(command, shell=True, check=True, 
                              capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: {description}")
        print(f"Command failed: {command}")
        print(f"Error output: {e.stderr}")
        return False

def check_executable(exe_name):
    """Checks if an executable is available in the PATH"""
    try:
        result = subprocess.run(f"where {exe_name}", shell=True, 
                              capture_output=True, text=True, check=True)
        print(f"✓ {exe_name} found at: {result.stdout.strip()}")
        return True
    except subprocess.CalledProcessError:
        print(f"✗ ERROR: {exe_name} not found in PATH!")
        return False

def check_required_tools():
    """Checks that all required tools are available"""
    print("=" * 50)
    print("           Checking Required Tools")
    print("=" * 50)
    
    tools = ["iix.exe", "Cadius.exe"]
    all_found = True
    
    for tool in tools:
        if not check_executable(tool):
            all_found = False
    
    if not all_found:
        print("\nERROR: Missing required tools!")
        print("Please ensure the following are in your Windows PATH:")
        print("- iix.exe (Golden Gate compiler)")
        print("- Cadius.exe (Apple disk utility)")
        sys.exit(1)
    
    print("✓ All required tools found!")
    print()


def cleanup_intermediate_files():
    print("\nCleaning intermediate files...")
    intermediate_files = [f"{PRG}.a", f"{PRG}.root", f"{PRG}.sym", f"{PRG}.B"]
    for file in intermediate_files:
        if os.path.exists(file):
            os.remove(file)
            print(f"Deleted: {file}")


def cadius_command(command, args):
    """Execute a Cadius command with properly quoted arguments.

    Cadius requires arguments to be quoted separately when paths or folder
    names contain special characters or spaces.
    """
    full_cmd = 'Cadius.exe ' + command + ' ' + ' '.join(f'"{arg}"' for arg in args)
    print(f"Executing: {full_cmd}")
    result = subprocess.run(full_cmd, shell=True, capture_output=True, text=True)
    output = (result.stdout or "") + (result.stderr or "")
    if output:
        print(output)
    return result.returncode, output


def cadius_store_file(image_path, prodos_dir, filename):
    """Copy a file into a Cadius disk image.

    The helper deletes an existing target file if present, then adds the
    current executable using the ProDOS folder and absolute source path.
    """
    abs_filename = os.path.abspath(filename)
    target_file = f"{prodos_dir}{filename}"

    # Try to delete any stale copy of the target file first.
    # A missing target file is not fatal, because we only want to clear old copies.
    delete_code, delete_output = cadius_command("DELETEFILE", [image_path, target_file])
    if delete_code != 0 and "not found" not in delete_output.lower():
        print(f"WARNING: DELETEFILE reported an issue for {target_file} but deployment will continue.")

    # Add the compiled executable into the disk image under the desired ProDOS folder.
    add_code, add_output = cadius_command("ADDFILE", [image_path, prodos_dir, abs_filename])
    if add_code != 0:
        print(f"ERROR: ADDFILE failed for {filename} on {image_path}")
        return False

    return True


def main():
    # --------------- Check Required Tools ---------------
    check_required_tools()
    
    # --------------- Variables ---------------
    print("=" * 50)
    print("           Variables Configuration")
    print("=" * 50)
    
    print(f"Program name: {PRG}")
    print(f"Language extension: {lang}")
    print(f"Apple image disk path: {AppleDiskPath}")
    print(f"ProDOS directory: {ProdosDir}")
    print()
    
    # Verify required source and resource files before compiling
    source_file = f"{PRG}{lang}"
    rez_file = f"{PRG}.rez"
    
    if not os.path.exists(source_file):
        print(f"ERROR: Source file {source_file} not found!")
        sys.exit(1)
    
    if not os.path.exists(rez_file):
        print(f"WARNING: Resource file {rez_file} not found!")

    if not os.path.exists(AppleDiskPath):
        print(f"ERROR: Apple disk path {AppleDiskPath} does not exist!")
        sys.exit(1)
    
    # --------------- Golden Gate Compilation ---------------
    print("=" * 50)
    print("           Golden Gate Compilation")
    print("=" * 50)
    
    # Compile only the program source file (PRG + language extension)
    source_file = f"{PRG}{lang}"
    if not os.path.exists(source_file):
        print(f"ERROR: Source file {source_file} not found!")
        sys.exit(1)
    print(f"Compiling source: {source_file}")
    if not run_command(f"iix compile {source_file}", "Compiling source"):
        sys.exit(1)
    
    # Link the program
    if not run_command(f"iix -DKeepType=S16 link {PRG} keep={PRG}", "Linking program"):
        sys.exit(1)
    
    # Compile the resource file and create archive (only if .rez exists)
    has_resources = os.path.exists(rez_file)
    if has_resources:
        if not run_command(f"iix compile {PRG}.rez keep={PRG}", "Compiling resource file"):
            print("ERROR: Resource compilation failed!")
            has_resources = False
            sys.exit(1)
        else:
            # Create an archive in Cadius format to preserve the resource fork.
            # If rexport fails, continue with the plain executable instead.
            if not run_command(f"iix rexport -i cadius {PRG}", "Exporting resource with Cadius format"):
                print("WARNING: Export failed, will copy executable directly")
                has_resources = False


    # --------------- Cadius Disk Operations on disk images ---------------
    print("=" * 50)
    print("           Cadius Disk Operations")
    print("=" * 50)
    
    # Determine which executable file will be copied to the disk images
    file_to_copy = PRG
    print(f"Copying executable to disk: {file_to_copy}")
    # Check that the compiled executable exists before trying to copy it.
    if not os.path.exists(file_to_copy):
        print(f"ERROR: Executable file {file_to_copy} not found!")
        sys.exit(1)
    if not cadius_store_file(AppleDiskPath, ProdosDir, file_to_copy):
        cleanup_intermediate_files()
        sys.exit(1)

    # Copy the executable to the local disk image for local testing
    print(f"Copying executable to local disk: {file_to_copy}")
    if not cadius_store_file(LocalDiskPath, LocalProdosDir, file_to_copy):
        cleanup_intermediate_files()
        sys.exit(1)

    # Remove compilation artifacts generated by Golden Gate.
    cleanup_intermediate_files()

    print()


    # --------------- Done ---------------
    print("=" * 50)
    print("               ✅   SUCCESS   ✅")
    print("            Compilation Complete!")
    print("=" * 50)


    if has_resources:
        print(f"Program {PRG} with resources successfully compiled and deployed!")
    else:
        print(f"Program {PRG} successfully compiled and deployed!")

    # In addition to the Apple disk image, the program is also stored in the local OBJ.po Apple disk image.
    print(f"Deployed to: {AppleDiskPath}{ProdosDir}")
    print("=" * 50 + "\n")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)