# Changelog

All notable changes to this project are documented here.

## [1.2.0] - 2026-07-18

### Added
- Star History chart in README (中文/English)
- GitHub Actions CI workflow: syntax check, structure validation, monthly schedule
- CI passing badge in README

### Changed
- Updated all simulation images and models from latest run

## [1.1.0] - 2026-07-17

### Added
- 6 advanced control tutorials: t16 (frequency domain), t17 (lead-lag), t18 (system ID), t19 (H∞ robust), t20 (MPC), t21 (sliding mode)
- GitHub community health files: issue templates (bug, feature, tutorial request), PR template
- CODE_OF_CONDUCT.md and SECURITY.md
- FUNDING.yml

### Changed
- Total tutorials: 15 → 21
- README overhaul: added table of contents, quick start, prerequisites, collapsible sections, improved badges
- README_EN.md synced with latest structure
- Updated CONTRIBUTING.md with detailed guidelines

### Fixed
- t19 LQR state feedback closed-loop formula (line 138): replaced incorrect `feedback()` with proper `A-BK` formulation
- t19 fig3 layout: changed from 1×2 to clean 2×1, removed empty axes

## [1.0.0] - 2026-07-17

### Added
- 15 Simulink tutorials covering fundamentals, control theory, and mechatronic systems
- 14 `.slx` Simulink model files
- `run_all_tutorials.m` — one-click batch runner
- Simulation result figures in `docs/images/`
- English README (`README_EN.md`)
- Contributing guidelines (`CONTRIBUTING.md`)
- MIT License

### Tutorials
- **Phase 1 (t01-t08):** Signal basics, math blocks, first/second-order systems, PID control, sources/sinks, subsystems, masking
- **Phase 2 (t09-t12):** State-space models, LQR optimal control, Luenberger observer, Kalman filter
- **Phase 3 (t13-t15):** DC motor modeling, PMSM FOC vector control, embedded C code generation
