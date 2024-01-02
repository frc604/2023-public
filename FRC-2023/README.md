# 2023 Robot Code

[![CI](https://github.com/frc604/FRC-2023/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/frc604/FRC-2023/actions/workflows/main.yml)

2023 onboard and offboard code for FRC604.

## Setup

### Install Python dependencies

- [Install](https://www.python.org/downloads/) Python 3.9 if it is not yet installed

- Run `pip3 install -r requirements.txt`

### To enable autoformatting when saving a file:

- [Install](https://marketplace.visualstudio.com/items?itemName=richardwillis.vscode-spotless-gradle) the `richardwillis.vscode-spotless-gradle` VS Code extension.

- In `File > Preferences > Settings`, search for `Format on Save` and enable it.

- If asked to select a formatter, choose `Spotless Gradle`.

## Code Structure

- `src/main/java/frc/quixlib`: General utilities to be used year-after-year
- `src/main/java/frc/robot`: Year-specific code
- `src/test/java/frc/...`: Unit tests for both `quixlib` and `robot`
- `offboard`: Code that doesn't run onboard the robot

## Commands

- `./gradlew spotlessApply` to format all code.
- `python3 offboard/quixpf/quixsam.py --local --view3d` to run the localizer in simulation
- `python3 offboard\scoring-selector\scoring_selector.py --local` to run the scoring selector in simulation
- `simulateWithQuixsam.bat` in command prompt (or `./simulateWithQuixsam.bat` in powershell) to run Quixsam and simulate code at the same time (only works on Windows)
