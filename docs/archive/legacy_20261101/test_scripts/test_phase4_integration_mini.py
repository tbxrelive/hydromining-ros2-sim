#!/usr/bin/env python3
"""Compatibility entry point for the bounded quick integration regression."""
import sys
from test_phase4_integration import main
if __name__ == '__main__':
    sys.argv = [sys.argv[0], '--quick']
    sys.exit(main())
