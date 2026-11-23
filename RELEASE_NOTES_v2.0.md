# Release v2.0 — ENCS4380 Homework 2 PID Control Toolkit

## Included
- Homework 1 Modules 1–4 retained in the same repository.
- Homework 2 P1 PID Playground with response metrics and validation checks.
- P2 Slow Sensor in the feedback loop.
- P3 Digital PID with sampling, anti-windup, Arduino code generation, and motor step-test workflow.
- P4 PID Tuning Challenge.
- My System reference application for a published DC motor model.

## Reference model used in My System
Source: Alejandro Von Chong and Dorindo Cárdenas, “Experimental data from an open-source DC motor control educational kit for calibration, system identification, and controller validation,” Data in Brief 69 (2026), Article 113274. DOI: 10.1016/j.dib.2026.113274.

The 20 ms model in Table 6 is G(s) = 25.59/(s + 22.37), with speed measured in RPM and PWM recorded as integer command counts. The static characterization sweeps 0–254 counts. Using 254 counts = 100% duty gives K = 2.90561 RPM per 1% PWM and tau = 0.04470 s. These are converted literature-reference parameters from a different test platform, not measurements of this team's Micromouse.

Dataset: https://data.mendeley.com/datasets/5xvg43r9r8/2

## Validation evidence recorded by the team
- P1: 3/3 validation checks passed (team-provided screenshot).
- P2: 4/4 validation checks passed (team-provided screenshot).
- P3: 4/4 validation checks passed (team-provided screenshot).
- P4: final hands-on verification remains with the team.
- My System: reference-model simulation screenshot recorded by the team; not hardware validation.

## Before publishing this release
1. Obtain instructor approval for using a published reference model instead of measurements from the team's own Micromouse.
2. If required, replace reference parameters with the team's encoder/PWM step-test values.
3. Complete P4 manual verification.
4. Confirm the final live GitHub Pages deployment and screenshots.
5. Confirm each team member's actual Homework 2 ownership and contribution record.
