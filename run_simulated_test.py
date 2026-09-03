# -*- coding: utf-8 -*-
"""无相机、无机器人的完整逻辑模拟测试入口。"""
from __future__ import annotations

import unittest
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parent
    suite = unittest.defaultTestLoader.discover(
        str(root / "tests"), pattern="test_*.py", top_level_dir=str(root))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
