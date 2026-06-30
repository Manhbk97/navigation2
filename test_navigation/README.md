# Nav2 Goal-Arrival Evaluator

Sends predefined goals to ROS 2 Jazzy Nav2, captures arrival pose, computes
translation/heading errors, and logs every trial to:

- a CSV file (appended live — crash-safe)
- an XLSX workbook (written at end of run, formatted, with a summary sheet)

Pass criteria:

- translation error ≤ 5 cm
- heading error ≤ 5°

## 1. Install dependencies (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
# ROS 2 / Nav2 message + TF packages
sudo apt install ros-jazzy-nav2-msgs ros-jazzy-tf2-ros

# Python deps (either apt or pip)
sudo apt install python3-openpyxl python3-yaml
# or:
pip3 install --user openpyxl PyYAML
```

LibreOffice Calc opens the XLSX directly (`xdg-open nav_eval.xlsx`).

## 2. Define your goals

Copy `config.example.yaml` to `config.yaml` and edit:

- `robot_namespace`: namespace of the robot's Nav2 stack — sets the
  `navigate_to_pose` action name. `auto` (default) discovers it from the ROS
  graph, `""` uses the global namespace, or pin it explicitly (e.g. `apple`).
- `map_frame`, `base_frame`, `pose_topic` are optional — if omitted they
  default off `robot_namespace` (`map`, `<ns>/base_link`, `/<ns>/amcl_pose`).
  Set them only if your robot uses different names.
- `pose_source`: `tf` (Nav2's own estimate) or `topic` (e.g. `/amcl_pose`, or
  a mocap-bridged topic).
  - **Note:** with `tf`, you're measuring Nav2's controller against Nav2's
    own localization — localization error is invisible. For genuine ±5 cm
    verification, use an external ground truth (motion capture, AprilTag
    overhead camera, etc.) published as `PoseWithCovarianceStamped`.
- `goals:` with your test poses (`theta` in radians).
- Thresholds, repeats, settle time as needed.

## 3. Run

Bring up your robot + Nav2 as usual, then:

```bash
python3 nav_eval.py --config config.yaml
```

With `interactive: true`, the script pauses before each trial so you can
reset the robot to its start pose. Press **ENTER** to send the next goal.

On finish:

```
========== SUMMARY ==========
Trials run : 15
Passed     : 13
Pass rate  : 86.7%
CSV log    : /home/you/logs/nav_eval.csv
XLSX log   : /home/you/logs/nav_eval.xlsx
```

## 4. Output

### `trials` sheet
Every trial in the order it ran:

```
timestamp, trial_id, goal_id,
x_goal, y_goal, theta_goal_deg,
x_actual, y_actual, theta_actual_deg,
e_trans_m, e_heading_deg, notes
```

- Angles are stored in **degrees** in the log (goals are configured in radians).
- `notes` flags issues: the Nav2 status (`ABORTED` / `CANCELED` / `REJECTED`)
  when the goal didn't succeed, or `no_pose (<status>)` when TF/topic was
  unavailable; empty on a clean `SUCCEEDED` trial.
- Pass/fail (translation ≤ threshold, |heading| ≤ threshold, status
  `SUCCEEDED`) is computed at run time and reported in the console summary
  only — it's no longer a per-trial column.

### `summary` sheet
All values are **Excel formulas** referencing the `trials` sheet — edit or
add rows on `trials` and the summary recomputes:

- Overall block: total trials, mean/max e_trans, mean/max |e_head|.
- Per-goal block: same columns, one row per goal_id from your config.

Open in LibreOffice Calc:

```bash
libreoffice --calc nav_eval.xlsx
```

## 5. Tips

- The CSV is the durable log. If the script crashes mid-run, you still have
  every trial in CSV. You can re-generate the XLSX from the CSV later if needed.
- Want a separate file per test session? Put a date in `xlsx_path`, e.g.
  `xlsx_path: ./logs/nav_eval_$(date +%F).xlsx` (set it from a launch script).
- For statistically meaningful numbers, aim for **N ≥ 10 trials per goal**
  and a diverse goal set (mix of distances, headings, approach directions).
