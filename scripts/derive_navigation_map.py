#!/usr/bin/python3
"""Correct saved unknown encoding in a new map artifact, preserving geometry."""
import argparse
from pathlib import Path
from stage4_route.map_derivation import derive_unknown_encoding

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-map', required=True, type=Path)
    p.add_argument('--candidate', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    print(derive_unknown_encoding(a.source_map, a.candidate, a.output))
