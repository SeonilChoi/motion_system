#!/usr/bin/env bash
set -euo pipefail

# Publish one MotorFrameMultiArray to /motor_state.
# silver_lain.yaml default expects controller indexes 0..35.
COUNT="${1:-36}"

ros2 topic pub --once /motor_state motion_system_msgs/msg/MotorFrameMultiArray "$(COUNT="$COUNT" python3 - <<'PY'
import os
import yaml

count = int(os.environ.get("COUNT", "36"))
frames = []
for i in range(count):
    frames.append({
        "number_of_target_interfaces": 0,
        "target_interface_id": [],
        "controller_index": i,
        "controlword": 0,
        "statusword": 0,
        "errorcode": 0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
    })

print(yaml.safe_dump({"data": frames}, default_flow_style=True).strip())
PY
)"
#!/usr/bin/env bash
set -euo pipefail
