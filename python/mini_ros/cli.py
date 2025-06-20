#!/usr/bin/env python3
"""
miniROS Python CLI Tool

Python command-line interface for miniROS.
Uses 'mini_ros_py' command to avoid conflicts with Rust version.

Philosophy: Keep it minimal - essential robotics functionality only.
"""

import argparse
import sys
import os
import json
import subprocess
from pathlib import Path
from typing import List, Dict, Optional


class MiniRosPyCLI:
    """Python CLI for miniROS operations"""
    
    def __init__(self):
        self.version = "0.1.2"
        
    def create_parser(self) -> argparse.ArgumentParser:
        """Create the argument parser with all subcommands"""
        parser = argparse.ArgumentParser(
            prog="mini_ros_py",
            description="🐍 miniROS Python CLI - Minimal robotics middleware",
            epilog="Philosophy: Maximum robotics performance, minimum complexity.",
            formatter_class=argparse.RawDescriptionHelpFormatter
        )
        
        parser.add_argument(
            '--version', '-V', 
            action='version', 
            version=f'miniROS Python CLI v{self.version}'
        )
        
        # Create subparsers
        subparsers = parser.add_subparsers(dest='command', help='Available commands')
        subparsers.required = True
        
        # Python-specific commands
        self._add_run_command(subparsers)
        self._add_examples_command(subparsers)
        self._add_test_command(subparsers)
        self._add_install_command(subparsers)
        self._add_version_command(subparsers)
        
        return parser
    
    def _add_run_command(self, subparsers):
        """Add 'run' command for running Python examples"""
        run_parser = subparsers.add_parser(
            'run',
            help='🏃 Run Python examples and scripts',
            description='Execute miniROS Python examples and utilities'
        )
        run_parser.add_argument(
            'example',
            help='Example name to run (e.g., minimal_publisher, turtlebot_controller)',
            nargs='?'
        )
        run_parser.add_argument(
            '--list', '-l',
            action='store_true',
            help='List available Python examples'
        )
        run_parser.add_argument(
            '--args',
            nargs='*',
            help='Additional arguments passed to the script'
        )
        
    def _add_examples_command(self, subparsers):
        """Add 'examples' command for managing examples"""
        examples_parser = subparsers.add_parser(
            'examples',
            help='📚 Manage Python examples',
            description='List, install, and manage miniROS Python examples'
        )
        examples_subparsers = examples_parser.add_subparsers(dest='examples_cmd', help='Examples commands')
        
        # List examples
        examples_subparsers.add_parser(
            'list',
            help='List all available examples'
        )
        
        # Install examples
        install_parser = examples_subparsers.add_parser(
            'install',
            help='Install examples to custom directory'
        )
        install_parser.add_argument(
            'directory',
            help='Target directory for examples'
        )
        
    def _add_test_command(self, subparsers):
        """Add 'test' command for running tests"""
        test_parser = subparsers.add_parser(
            'test',
            help='🧪 Run Python tests',
            description='Execute miniROS Python test suite'
        )
        test_parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            help='Verbose test output'
        )
        test_parser.add_argument(
            '--coverage', '-c',
            action='store_true',
            help='Run tests with coverage'
        )
        test_parser.add_argument(
            'pattern',
            nargs='?',
            help='Test pattern to match (optional)'
        )
        
    def _add_install_command(self, subparsers):
        """Add 'install' command for package installation"""
        install_parser = subparsers.add_parser(
            'install',
            help='📦 Install miniROS Python package',
            description='Install or reinstall miniROS Python bindings'
        )
        install_parser.add_argument(
            '--dev', '-d',
            action='store_true',
            help='Install in development mode'
        )
        install_parser.add_argument(
            '--user', '-u',
            action='store_true',
            help='Install for current user only'
        )
        
    def _add_version_command(self, subparsers):
        """Add 'version' command for version info"""
        version_parser = subparsers.add_parser(
            'version',
            help='📋 Show version information',
            description='Display miniROS Python version and environment info'
        )
        version_parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            help='Show detailed version information'
        )
        
    def get_examples_dir(self) -> Path:
        """Get the examples directory path"""
        current_dir = Path(__file__).parent.parent
        return current_dir / "examples"
    
    def list_examples(self) -> List[str]:
        """List all available Python examples"""
        examples_dir = self.get_examples_dir()
        if not examples_dir.exists():
            return []
        
        examples = []
        for file_path in examples_dir.glob("*.py"):
            if file_path.name.startswith("__"):
                continue
            examples.append(file_path.stem)
        
        return sorted(examples)
    
    def run_example(self, example_name: str, args: Optional[List[str]] = None) -> int:
        """Run a specific Python example"""
        examples_dir = self.get_examples_dir()
        example_file = examples_dir / f"{example_name}.py"
        
        if not example_file.exists():
            print(f"❌ Example '{example_name}' not found")
            print(f"Available examples: {', '.join(self.list_examples())}")
            return 1
        
        print(f"🏃 Running Python example: {example_name}")
        
        # Build command
        cmd = [sys.executable, str(example_file)]
        if args:
            cmd.extend(args)
        
        try:
            return subprocess.run(cmd).returncode
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted by user")
            return 130
        except Exception as e:
            print(f"❌ Failed to run example: {e}")
            return 1
    
    def run_tests(self, verbose: bool = False, coverage: bool = False, pattern: Optional[str] = None) -> int:
        """Run the Python test suite"""
        # Find project root
        current_dir = Path(__file__).parent.parent
        
        cmd = [sys.executable, "-m", "pytest"]
        
        if verbose:
            cmd.append("-v")
        
        if coverage:
            cmd.extend(["--cov=mini_ros", "--cov-report=term-missing"])
        
        if pattern:
            cmd.extend(["-k", pattern])
        
        # Add tests directory
        tests_dir = current_dir / "tests"
        if tests_dir.exists():
            cmd.append(str(tests_dir))
        
        print(f"🧪 Running Python tests: {' '.join(cmd[2:])}")
        
        try:
            return subprocess.run(cmd, cwd=current_dir).returncode
        except FileNotFoundError:
            print("❌ pytest not found. Install with: pip install pytest")
            return 1
        except Exception as e:
            print(f"❌ Failed to run tests: {e}")
            return 1
    
    def install_package(self, dev: bool = False, user: bool = False) -> int:
        """Install the miniROS Python package"""
        current_dir = Path(__file__).parent.parent
        
        cmd = [sys.executable, "-m", "pip", "install"]
        
        if dev:
            cmd.append("-e")
        
        if user:
            cmd.append("--user")
        
        cmd.append(str(current_dir))
        
        print(f"📦 Installing miniROS Python package: {' '.join(cmd)}")
        
        try:
            return subprocess.run(cmd).returncode
        except Exception as e:
            print(f"❌ Failed to install package: {e}")
            return 1
    
    def show_version(self, verbose: bool = False) -> None:
        """Show version information"""
        if verbose:
            import platform
            print(f"🐍 miniROS Python CLI v{self.version}")
            print(f"🐍 Python version: {platform.python_version()}")
            print(f"🏗️  Platform: {platform.platform()}")
            print(f"📁 Package location: {Path(__file__).parent}")
            
            try:
                import mini_ros
                print(f"📦 miniROS module: Available")
            except ImportError:
                print(f"📦 miniROS module: Not installed")
            
            print(f"\n💡 Philosophy: Maximum robotics performance, minimum complexity")
        else:
            print(f"miniROS Python CLI v{self.version}")
    
    def run(self, args: Optional[List[str]] = None) -> int:
        """Main entry point for the CLI"""
        parser = self.create_parser()
        
        try:
            parsed_args = parser.parse_args(args)
        except SystemExit as e:
            return e.code
        
        # Dispatch to appropriate handler
        try:
            if parsed_args.command == "run":
                if parsed_args.list:
                    examples = self.list_examples()
                    print("📚 Available Python examples:")
                    for example in examples:
                        print(f"  • {example}")
                    return 0
                elif parsed_args.example:
                    return self.run_example(parsed_args.example, parsed_args.args)
                else:
                    print("❌ No example specified. Use --list to see available examples.")
                    return 1
                    
            elif parsed_args.command == "examples":
                if parsed_args.examples_cmd == "list":
                    examples = self.list_examples()
                    print("📚 Available Python examples:")
                    for example in examples:
                        print(f"  • {example}")
                    return 0
                elif parsed_args.examples_cmd == "install":
                    # Copy examples to target directory
                    import shutil
                    target_dir = Path(parsed_args.directory)
                    examples_dir = self.get_examples_dir()
                    
                    if not examples_dir.exists():
                        print("❌ Examples directory not found")
                        return 1
                    
                    target_dir.mkdir(parents=True, exist_ok=True)
                    shutil.copytree(examples_dir, target_dir / "examples", dirs_exist_ok=True)
                    print(f"✅ Examples installed to: {target_dir / 'examples'}")
                    return 0
                    
            elif parsed_args.command == "test":
                return self.run_tests(
                    verbose=parsed_args.verbose,
                    coverage=parsed_args.coverage,
                    pattern=parsed_args.pattern
                )
                
            elif parsed_args.command == "install":
                return self.install_package(
                    dev=parsed_args.dev,
                    user=parsed_args.user
                )
                
            elif parsed_args.command == "version":
                self.show_version(verbose=parsed_args.verbose)
                return 0
                
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted by user")
            return 130
        except Exception as e:
            print(f"❌ Command failed: {e}")
            return 1
        
        return 0


def main():
    """Main entry point for mini_ros_py command"""
    cli = MiniRosPyCLI()
    return cli.run()


if __name__ == "__main__":
    sys.exit(main()) 