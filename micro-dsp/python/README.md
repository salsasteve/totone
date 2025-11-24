# micro-dsp Python Utilities

Helper scripts for DSP development and testing.

## Overview

These Python scripts assist in developing and verifying the DSP library implementation, particularly for fixed-point arithmetic, window functions, and test data generation.

## Scripts

### generate_q15_cos.py

Generates Q15 fixed-point cosine lookup tables for embedded systems.

**Usage:**
```bash
python generate_q15_cos.py
```

**Purpose:**
- Precompute cosine values in Q15 fixed-point format
- Generate Rust array constants for lookup tables
- Useful for FFT implementations that need cosine tables

### generate_sine.py

Generates sine wave test data for DSP testing.

**Usage:**
```bash
python generate_sine.py
```

**Purpose:**
- Create synthetic sine wave audio samples
- Generate test vectors for FFT verification
- Output in formats suitable for Rust test code

### check_apply_hann_window.py

Verifies the Hann window function implementation.

**Usage:**
```bash
python check_apply_hann_window.py
```

**Purpose:**
- Validate window function coefficients
- Compare against reference implementation
- Verify correct application to sample buffers

### check_q15cos.py

Tests Q15 fixed-point cosine implementation.

**Usage:**
```bash
python check_q15cos.py
```

**Purpose:**
- Verify Q15 cosine lookup table accuracy
- Compare fixed-point vs floating-point results
- Check quantization error bounds

### check_arc_cos.py

Tests arccosine function implementations.

**Usage:**
```bash
python check_arc_cos.py
```

**Purpose:**
- Verify arccosine calculations
- Test approximation methods for embedded use
- Validate trigonometric function implementations

### rand.py

Generates random test data for DSP testing.

**Usage:**
```bash
python rand.py
```

**Purpose:**
- Generate random audio samples
- Create noise test vectors
- Generate edge cases for testing

## Requirements

These scripts typically require:
- Python 3.8+
- NumPy (for numerical calculations)

Install dependencies:
```bash
pip install numpy
```

## Usage Notes

- Most scripts output to stdout (print Rust code or test data)
- Redirect output to files if needed: `python script.py > output.rs`
- Modify constants at the top of each script to customize output

## Integration

These scripts are used during development to:
1. Generate lookup tables for embedded code
2. Create test data for unit tests
3. Verify mathematical implementations
4. Generate reference outputs for comparison

The generated code/data can be copy-pasted into Rust test files or used as reference implementations.

