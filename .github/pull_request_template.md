## Summary

<!-- Brief description of what this PR does and why. -->

## Type of Change

- [ ] `feat`: New feature or capability
- [ ] `fix`: Bug fix
- [ ] `refactor`: Code refactoring without behavior change
- [ ] `docs`: Documentation updates
- [ ] `test`: Unit tests or testing improvements
- [ ] `chore/ci`: Build, tooling, dependencies, or CI workflow changes

## Contract Impact

- [ ] None (no changes to `interfaces/tsa_interfaces/`)
- [ ] `tsa_interfaces` changed — **requires review and approval from each group (`overhead`, `fleet`, `manipulator`)**

## Definition of Done

- [ ] All tests green with `python -m pytest overhead/overhead_core`
- [ ] Code formatted and clean with `ruff format --check .` and `ruff check .`
- [ ] English only (identifiers, file names, comments, docstrings, commits)
- [ ] No magic numbers or strings (constants with unit suffixes and `StrEnum` used)
- [ ] Modularity respected: pure logic in `_core` (no `rclpy`), thin adapters in `_ros`
- [ ] Google-style docstrings present on all public modules, classes, and functions
- [ ] Colcon build passes cleanly
