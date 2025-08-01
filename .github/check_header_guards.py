#!/usr/bin/env python3

# Copyright 2025 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re
import sys
from pathlib import Path


def to_macro_name(path: Path, repo_root: Path) -> str:
    path = path.resolve()
    repo_root = repo_root.resolve()

    try:
        rel_path = path.relative_to(repo_root)
    except ValueError:
        rel_path = path

    parts = list(rel_path.with_suffix("").parts)
    package = parts[0]

    if "include" in parts:
        include_subpath = parts[2:]
    elif "test" in parts:
        include_subpath = parts[1:]

    macro_parts = [package] + include_subpath
    macro = "_".join(part.upper() for part in macro_parts) + "_HPP_"
    return macro


def find_guard_lines(lines, expected_macro):
    ifndef_line = -1
    define_line = -1

    for i, line in enumerate(lines[:30]):
        if ifndef_line == -1 and re.match(rf"#ifndef\s+{re.escape(expected_macro)}", line.strip()):
            ifndef_line = i
        elif (
            define_line == -1
            and ifndef_line != -1
            and re.match(rf"#define\s+{re.escape(expected_macro)}", line.strip())
        ):
            define_line = i
            break

    return ifndef_line, define_line


def has_endif(lines, expected_macro):

    return any(
        re.match(rf"#endif\s+//\s+{re.escape(expected_macro)}", line.strip())
        for line in lines[-10:]
    )


def check_guard(filepath, repo_root):
    filepath = Path(filepath)

    if not filepath.exists():
        return False, f"File not found: {filepath}"

    expected_macro = to_macro_name(filepath, repo_root)

    try:
        with filepath.open("r", encoding="utf-8") as f:
            lines = f.readlines()
    except (IOError, UnicodeDecodeError) as e:
        return False, f"Error reading file {filepath}: {e}"

    ifndef_line, define_line = find_guard_lines(lines, expected_macro)
    endif_ok = has_endif(lines, expected_macro)

    return (ifndef_line != -1 and define_line != -1 and endif_ok), expected_macro


def main():
    repo_root = Path(os.getcwd()).resolve()
    failed = []

    for file in sys.argv[1:]:
        if file.endswith(".hpp"):
            ok, expected = check_guard(file, repo_root)
            if not ok:
                failed.append((file, expected))

    if failed:
        print("❌ Invalid or missing header guards:")
        for f, macro in failed:
            print(f"  - {f} (expected: {macro})")
        sys.exit(1)


if __name__ == "__main__":
    main()
