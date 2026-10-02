#!/usr/bin/env bash
# Run a selected subset of RoboCasa atomic tasks with horizons under 20 seconds.
# Source: https://robocasa.ai/docs/build/html/tasks/atomic_tasks.html
# (26 selected tasks; horizon in seconds noted on each line)

set -u

# robot.py exits immediately without this; fail fast instead of once per task.
if [[ -z "${OPENAI_API_KEY:-}" ]]; then
  echo "OPENAI_API_KEY is not set. Run: export OPENAI_API_KEY=sk-..." >&2
  exit 1
fi

TASKS=(
  # PickPlaceFridgeShelfToDrawer    # 12s
  # PickPlaceToasterOvenToCounter   # 18s
  SlideToasterOvenRack            # 5s
  TurnOnBlender                   # 5s
  CloseElectricKettleLid          # 7s
  CloseFridgeDrawer               # 7s
  # NavigateKitchen                 # 7s
  OpenElectricKettleLid           # 8s
  OpenFridgeDrawer                # 8s
  OpenStandMixerHead              # 8s
  StartCoffeeMachine              # 8s
  CloseStandMixerHead             # 9s
  OpenBlenderLid                  # 9s
  SlideDishwasherRack             # 9s
  TurnSinkSpout                   # 10s
  TurnOnElectricKettle            # 11s
  CloseToasterOvenDoor            # 12s
  PickPlaceCounterToDrawer        # 12s
  PickPlaceFridgeDrawerToShelf    # 12s
  LowerHeat                       # 14s
  OpenToasterOvenDoor             # 14s
  CheesyBread                     # 15s
  PickPlaceDrawerToCounter        # 15s
  AdjustWaterTemperature          # 16s
  CloseBlenderLid                 # 17s
  TurnOnStove                     # 17s
  PickPlaceCounterToBlender       # 18s
  AdjustToasterOvenTemperature    # 19s
  CloseDrawer                     # 19s
)

failed=()
for task_name in "${TASKS[@]}"; do
  echo "=============================="
  echo "Running task: $task_name"
  echo "=============================="
  if ! mjpython robot.py --task "$task_name" --seed 6 --vision --max-calls 20; then
    echo "!! Task failed: $task_name" >&2
    failed+=("$task_name")
  fi
done

echo
echo "Finished ${#TASKS[@]} tasks."
if (( ${#failed[@]} )); then
  echo "Failed (${#failed[@]}): ${failed[*]}"
  exit 1
fi
