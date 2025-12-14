#!/bin/bash
# Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Generate TensorRT engine files for all ONNX models
# Usage: ./generate_engines.sh [models_directory]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../.." && pwd )"
MODELS_DIR="${1:-${SCRIPT_DIR}/depth_anything_v3/models}"

echo "Sourcing the built workspace..."
source "$WORKSPACE_DIR/install/setup.bash"

echo "Generating TensorRT engines..."
"$WORKSPACE_DIR/install/depth_anything_v3/lib/depth_anything_v3/generate_engines" "${MODELS_DIR}"

echo "Done! Engine files have been generated in ${MODELS_DIR}"
