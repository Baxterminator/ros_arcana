#!/usr/bin/python3

from cv2.aruco import (
    DICT_4X4_1000,
    DICT_5X5_1000,
    DICT_6X6_1000,
    DICT_7X7_1000,
    DICT_ARUCO_ORIGINAL,
    getPredefinedDictionary,
)
import numpy as np
import argparse as ag
import os

AVAILABLE_DICTS = {
    "4": DICT_4X4_1000,
    "5": DICT_5X5_1000,
    "6": DICT_6X6_1000,
    "7": DICT_7X7_1000,
    "original": DICT_ARUCO_ORIGINAL,
}


def byte2bintstr(b: int) -> str:
    if b > 255 or b < 0:
        raise RuntimeError("Argument of byte2intstr should be an int in [0, 255]")
    bcopy = int(b)
    out = ""
    power = 7
    while power >= 0:
        if bcopy // (2**power):
            bcopy -= 2**power
            out += "1"
        else:
            out += "0"
        power -= 1
    return out


if __name__ == "__main__":
    parser = ag.ArgumentParser()
    parser.add_argument(
        "size", help="The size of the marker", choices=AVAILABLE_DICTS.keys()
    )
    parser.add_argument("-o", dest="outfile", default="---")
    args = parser.parse_args()

    outfile = args.outfile
    if outfile == "---":
        outfile = f"aruco_{args.size}x{args.size}_1000.yaml"
        outfile = os.path.join("assets", "data", outfile)

    dest_dir = os.path.dirname(outfile)
    if not os.path.exists(dest_dir):
        os.mkdir(dest_dir)

    d = getPredefinedDictionary(AVAILABLE_DICTS[args.size])
    n_codes, n_bytes, n = np.shape(d.bytesList)
    print(f"Processing {n_codes} (of {n_bytes}) for ArUco codes of size {args.size}")
    with open(outfile, "w") as file:
        for i in range(n_codes):
            if i % 20 == 19:
                print(f"{i:4d}")
            else:
                print(f"{i:4d}", end="")
            code_bytes = d.bytesList[i].ravel()[:n_bytes]
            file.write(f"{i}: \"{''.join([byte2bintstr(b) for b in code_bytes])}\"\n")
