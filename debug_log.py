import re

with open('headless.log', 'r', encoding='utf-8', errors='ignore') as f:
    lines = list(f)
    print(f"Total lines: {len(lines)}")
    
    # Find lines with "Cleared to land"
    cleared_lines = [i for i, l in enumerate(lines, 1) if 'Cleared to land' in l]
    print(f"Lines with 'Cleared to land': {cleared_lines[:10]}")
    
    if cleared_lines:
        idx = cleared_lines[0] - 1
        line = lines[idx]
        print(f"\nLine {cleared_lines[0]}: {repr(line[:80])}")
        
        rx = re.compile(r'\[t=\s*([\d\.]+)s\]\s*ATC:\s*Cleared to land', re.IGNORECASE)
        match = rx.search(line)
        print(f"Regex match: {match}")
        
        if not match:
            print(f"\nChecking for hidden characters...")
            print(f"Starts with '[t=': {line.strip().startswith('[t=')}")
            print(f"Contains 'ATC:': {'ATC:' in line}")
            print(f"First 20 chars (repr): {repr(line[:20])}")
