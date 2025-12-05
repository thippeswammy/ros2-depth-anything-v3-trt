# ONNX Export for Depth Anything V3

This directory contains the export script to convert Depth Anything V3 models to ONNX format.

## Prerequisites

1. **Download the official model** from Hugging Face:
   - [DA3METRIC-LARGE](https://huggingface.co/depth-anything/DA3METRIC-LARGE)

2. **Set up the Depth Anything 3 environment** following the official repository:
   - https://github.com/ByteDance-Seed/depth-anything-3

## Export Steps

### 1. Copy the export script

Copy `export.py` from this directory to the Depth Anything 3 repository:

```bash
cp export.py /path/to/depth-anything-3/src/depth_anything_3/
```

### 2. Modify the API for ONNX compatibility

Edit `src/depth_anything_3/api.py` and change the autocast dtype from `bfloat16` to `float16` for ONNX compatibility:

**Before:**
```python
# Determine optimal autocast dtype
autocast_dtype = torch.bfloat16 if torch.cuda.is_bf16_supported() else torch.float16
with torch.no_grad():
    with torch.autocast(device_type=image.device.type, dtype=autocast_dtype):
        return self.model(
            image, extrinsics, intrinsics, export_feat_layers, infer_gs, use_ray_pose, ref_view_strategy
        )
```

**After:**
```python
# Determine optimal autocast dtype
autocast_dtype = torch.float16
with torch.no_grad():
    with torch.autocast(device_type=image.device.type, dtype=autocast_dtype):
        return self.model(
            image, extrinsics, intrinsics, export_feat_layers, infer_gs, use_ray_pose, ref_view_strategy
        )
```

> **Note:** This change is required because ONNX does not fully support `bfloat16` operations.

### 3. Run the export

```bash
cd /path/to/depth-anything-3/src/depth_anything_3/

python export.py \
    --model-dir DA3METRIC-LARGE \
    --height 280 \
    --width 504 \
    --output-dir /path/to/output/
```

This will generate `DA3METRIC-LARGE.onnx` in the specified output directory.

## Export Options

| Argument | Default | Description |
|----------|---------|-------------|
| `--model-dir` | `DA3METRIC-LARGE` | Path to the model checkpoint directory |
| `--height` | `280` | Input height (must be divisible by 14) |
| `--width` | `504` | Input width (must be divisible by 14) |
| `--batch-size` | `1` | Batch size for export |
| `--opset` | `20` | ONNX opset version |
| `--device` | `cuda` | Device for export (`cuda` or `cpu`) |
| `--output-dir` | `.` | Output directory for ONNX file |
| `--demo-image` | - | Optional image path to run inference demo |

## Model Outputs

The exported ONNX model has two outputs:
- `depth`: Metric depth map `[B, 1, H, W]`
- `sky`: Sky classification logits `[B, 1, H, W]`
