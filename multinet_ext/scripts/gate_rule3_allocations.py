import os
import re
import sys
import time

def main():
    start_time = time.perf_counter()
    
    # Define directories to scan (must be absolute or relative to script execution dir)
    # We will assume script is run from project root: d:\Engines\Custom\godot-multinet
    src_dir = os.path.join("multinet_ext", "src", "multinet")
    target_dirs = [
        os.path.join(src_dir, "world"),
        os.path.join(src_dir, "network")
    ]
    
    # Patterns that violate Rule 3 (Zero hot-path heap allocations)
    banned_patterns = [
        (re.compile(r'\bstd::vector\b'), "std::vector"),
        (re.compile(r'\bstd::map\b'), "std::map"),
        (re.compile(r'\bstd::unordered_map\b'), "std::unordered_map"),
        (re.compile(r'\bstd::string\b'), "std::string"),
        (re.compile(r'\bnew\s+[A-Za-z_]'), "new keyword"),
        (re.compile(r'\bmalloc\b'), "malloc")
    ]
    
    violations_found = 0
    scanned_files = 0
    
    for t_dir in target_dirs:
        if not os.path.exists(t_dir):
            continue
            
        for root, _, files in os.walk(t_dir):
            for file in files:
                if file.endswith(".h") or file.endswith(".cpp"):
                    file_path = os.path.join(root, file)
                    scanned_files += 1
                    
                    with open(file_path, "r", encoding="utf-8") as f:
                        lines = f.readlines()
                        
                    for line_idx, line in enumerate(lines):
                        # Skip single-line comments for fast scanning
                        stripped = line.strip()
                        if stripped.startswith("//"):
                            continue
                            
                        for pattern, name in banned_patterns:
                            if pattern.search(line):
                                print(f"[RULE 3 VIOLATION] {file_path}:{line_idx+1} -> Found '{name}'")
                                print(f"    {stripped}")
                                violations_found += 1
                                
    elapsed = (time.perf_counter() - start_time) * 1000.0
    
    if violations_found > 0:
        print(f"\n[GATE FAILED] Found {violations_found} heap allocation violations in {scanned_files} files.")
        print(f"Rule 3 demands ZERO hot-path heap allocations. Use BoundedPool, Arenas, or std::array.")
        sys.exit(1)
        
    print(f"[GATE PASSED] Rule 3 static analysis complete: {scanned_files} files scanned in {elapsed:.2f}ms.")
    sys.exit(0)

if __name__ == "__main__":
    main()
