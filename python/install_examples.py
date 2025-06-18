#!/usr/bin/env python3
"""
Installation script for miniROS Python examples dependencies
"""

import subprocess
import sys
import importlib.util

def check_package(package_name):
    """Check if a package is installed"""
    spec = importlib.util.find_spec(package_name)
    return spec is not None

def install_package(package_name, description=""):
    """Install a package using pip"""
    print(f"Installing {package_name}... {description}")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
        print(f"✅ {package_name} installed successfully")
        return True
    except subprocess.CalledProcessError:
        print(f"❌ Failed to install {package_name}")
        return False

def main():
    print("🚀 Installing dependencies for miniROS Python examples\n")
    
    # Required packages for advanced examples
    packages = [
        ("opencv-python", "Computer vision and image processing"),
        ("numpy", "Numerical computing"),
        ("rerun-sdk", "3D visualization and logging"),
    ]
    
    # Check which packages are missing
    missing_packages = []
    for package_name, description in packages:
        # Handle special cases for import names
        import_name = package_name
        if package_name == "opencv-python":
            import_name = "cv2"
        elif package_name == "rerun-sdk":
            import_name = "rerun"
        
        if not check_package(import_name):
            missing_packages.append((package_name, description))
        else:
            print(f"✅ {package_name} already installed")
    
    if not missing_packages:
        print("\n🎉 All dependencies are already installed!")
        print("\nYou can now run the advanced examples:")
        print("  python python/examples/image_publisher.py")
        print("  python python/examples/image_subscriber.py")
        print("  python python/examples/robot_visualization.py")
        return
    
    # Install missing packages
    print(f"\n📦 Installing {len(missing_packages)} missing packages...\n")
    
    failed_packages = []
    for package_name, description in missing_packages:
        if not install_package(package_name, description):
            failed_packages.append(package_name)
    
    # Summary
    print("\n" + "="*50)
    if not failed_packages:
        print("🎉 All dependencies installed successfully!")
        print("\nYou can now run the advanced examples:")
        print("  python python/examples/image_publisher.py")
        print("  python python/examples/image_subscriber.py") 
        print("  python python/examples/robot_visualization.py")
    else:
        print(f"⚠️  {len(failed_packages)} packages failed to install:")
        for package in failed_packages:
            print(f"  - {package}")
        print("\nPlease install them manually:")
        for package in failed_packages:
            print(f"  pip install {package}")

if __name__ == "__main__":
    main() 