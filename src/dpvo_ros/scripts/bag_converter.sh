#!/bin/bash
# convert_bag1_to_bag2.sh
# Usage: ./convert_bag1_to_bag2.sh <input.bag>

set -e

# ── Input validation ──────────────────────────────────────────────────────────
if [ $# -ne 1 ]; then
    echo "Usage: $0 <input.bag>"
    exit 1
fi

INPUT_BAG="$1"

if [ ! -f "$INPUT_BAG" ]; then
    echo "[ERROR] File not found: $INPUT_BAG"
    exit 1
fi

if [[ "$INPUT_BAG" != *.bag ]]; then
    echo "[ERROR] Input must be a .bag file"
    exit 1
fi

# ── Derive output folder name ─────────────────────────────────────────────────
BASENAME=$(basename "$INPUT_BAG" .bag)
OUTPUT_DIR="$(dirname "$INPUT_BAG")/${BASENAME}_bag"

echo "[INFO] Input:  $INPUT_BAG"
echo "[INFO] Output: $OUTPUT_DIR"

# ── Check dependencies ────────────────────────────────────────────────────────
if ! command -v rosbags-convert &> /dev/null; then
    echo "[INFO] rosbags not found, installing..."
    pip install rosbags
fi

if ! command -v python3 &> /dev/null; then
    echo "[ERROR] python3 is required"
    exit 1
fi

# ── Convert ───────────────────────────────────────────────────────────────────
echo "[INFO] Converting with rosbags-convert..."

if [ -d "$OUTPUT_DIR" ]; then
    echo "[WARN] Output directory already exists, removing: $OUTPUT_DIR"
    rm -rf "$OUTPUT_DIR"
fi

rosbags-convert "$INPUT_BAG" --dst "$OUTPUT_DIR"

echo "[INFO] Conversion done, fixing metadata..."

# ── Fix metadata ──────────────────────────────────────────────────────────────
METADATA_FILE="$OUTPUT_DIR/metadata.yaml"

if [ ! -f "$METADATA_FILE" ]; then
    echo "[ERROR] metadata.yaml not found in $OUTPUT_DIR"
    exit 1
fi

python3 - "$METADATA_FILE" <<'EOF'
import sys
import yaml

class LiteralInt(int):
    pass

def int_representer(dumper, value):
    return dumper.represent_scalar('tag:yaml.org,2002:int', str(value))

yaml.add_representer(LiteralInt, int_representer)

metadata_path = sys.argv[1]

with open(metadata_path, 'r') as f:
    meta = yaml.safe_load(f)

info = meta['rosbag2_bagfile_information']

# Downgrade version for compatibility
info['version'] = 8

# Fix large int serialization (prevent scientific notation)
info['duration']['nanoseconds'] = LiteralInt(info['duration']['nanoseconds'])
info['starting_time']['nanoseconds_since_epoch'] = LiteralInt(
    info['starting_time']['nanoseconds_since_epoch']
)

for f in info.get('files', []):
    f['duration']['nanoseconds'] = LiteralInt(f['duration']['nanoseconds'])
    f['starting_time']['nanoseconds_since_epoch'] = LiteralInt(
        f['starting_time']['nanoseconds_since_epoch']
    )

for topic in info.get('topics_with_message_count', []):
    tm = topic['topic_metadata']
    # Remove version-9 field unsupported by older rosbag2
    tm.pop('type_description_hash', None)
    # Replace empty QoS list with empty string (older yaml-cpp can't parse [])
    tm['offered_qos_profiles'] = ''

with open(metadata_path, 'w') as f:
    yaml.dump(meta, f, default_flow_style=False, allow_unicode=True)

print("[INFO] metadata.yaml patched successfully")
EOF

# ── Verify ────────────────────────────────────────────────────────────────────
echo "[INFO] Verifying with ros2 bag info..."
ros2 bag info "$OUTPUT_DIR"

echo ""
echo "[DONE] $OUTPUT_DIR is ready"