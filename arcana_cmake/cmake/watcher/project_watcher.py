#!/usr/bin/python3
import argparse as ag
import os
from sys import stderr
from typing import List
import datetime
import sys


def read_file_list(log_path: str) -> List[str]:
    """
    Read the log file of the watcher (file that contains all known files from last Make Update)
    """
    if not os.path.exists(log_path):
        return []
    with open(log_path, "r") as log_file:
        return [f.strip() for f in log_file.readlines()]


def is_folder_accepted(folder: List[str], whitelist: List[str]) -> bool:
    """
    Return True if folder is in the whitelist, or under a folder in the whitelist
    """
    for wl_folder_path in whitelist:
        wl_folder = wl_folder_path.split("/")

        # If depth of whitelisted folder is bigger than the folder, then it can't be a child folder
        if len(wl_folder) > len(folder):
            continue

        print(wl_folder)

        # Check for folder chain
        chain_ok = True
        for idx in range(len(wl_folder)):
            if folder[idx] != wl_folder[idx]:
                chain_ok = False
                break
        if chain_ok:
            return True
    return False


def walk_in_project(source_path: str, folders: List[str]) -> List[str]:
    """
    Walks in the project folder and get the list of all the files present there.
    If folders is empty, then it scans all the folders present.
    If a directory starts with a dot (.), then it is skipped.
    """
    files = []
    for dirpath, _, filenames in os.walk(source_path):
        # Get relative dir path
        if len(folders) != 0:
            folder = os.path.relpath(dirpath, source_path).split(os.sep)
            if not is_folder_accepted(folder, folders):
                continue

        # Add filenames
        files.extend([os.path.join(dirpath, fn) for fn in filenames])
    return files


def export_file_list(log_path: str, files: List[str]):
    """
    Write down in the log file all files that are contained
    """
    with open(log_path, "w") as log_file:
        log_file.write("\n".join(files))


if __name__ == "__main__":
    # Parsing arguments
    parser = ag.ArgumentParser()
    parser.add_argument("source_dir", type=str)
    parser.add_argument("build_dir", type=str)
    parser.add_argument("-fbl", nargs="+", type=str, dest="folders_blacklist")
    parser.add_argument(
        "-fwl", nargs="+", type=str, dest="folders_whitelist", default=[]
    )
    parser.add_argument("--cmake", nargs="+", type=str, dest="cmake_args", default="")
    parser.add_argument("-l", "--log", action="store_true")
    args = parser.parse_args()

    # If file does not exist, stop here CMake will be run anyway
    list_file = os.path.join(args.build_dir, "arcana_watcher.txt")
    now = datetime.datetime.now()
    prefix = f"[{now.time()} | Arcana Watcher] "

    old_file_list = read_file_list(list_file)
    new_file_list = walk_in_project(args.source_dir, args.folders_whitelist)

    if old_file_list != new_file_list:
        if os.path.exists(list_file):
            if args.log:
                print(f"{prefix}Found different files, rerunning CMake !", file=stderr)
            os.system(
                f"cmake -S {args.source_dir} -B {args.build_dir} {args.cmake_args}"
            )
        export_file_list(list_file, new_file_list)

    elif args.log:
        print(f"{prefix}No differences in project structure found !", file=stderr)
