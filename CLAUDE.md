# CLAUDE.md — Agent Quick Reference

The binding single source of truth for all rules is [AGENTS.md](AGENTS.md).

## Non-Negotiables

1. **English only:** Identifiers, file names, comments, docstrings, commit messages, and documentation must be in English.
2. **No magic numbers or strings:** Constants with unit suffixes (`_m`, `_s`, `_rad`, `_px`, etc.) and `StrEnum` for states/categories.
3. **Multiline programming:** Max 100 columns, one statement per line, trailing commas on calls/collections, and maximum 3 positional parameters.
4. **Google-style docstrings:** Required on all public modules, classes, and functions; comments explain *why*, never *what*.
5. **Strict modularity:** Pure logic in `_core` (no `rclpy`, synthetic tests only); thin ROS adapters in `_ros` with conversions in `conversions.py`.
6. **Git conventions:** Conventional Commits (`<type>(<scope>): [TSA-<n> ]<summary>`), no direct commits or pushes to `main` or `develop`.
