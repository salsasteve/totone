import math

def generate_bin_indices(num_bars, total_fft_bins_count):
    """
    Calculates (start_index, end_index) for FFT bins for each bar.
    - num_bars: The number of display bars.
    - total_fft_bins_count: The total number of FFT bins (e.g., 512 for indices 0-511).
                            The end_index generated can be equal to total_fft_bins_count,
                            making it suitable for exclusive upper bounds in slices.
    """

    # --- Even Spreading (Adjusted for Full Coverage) ---
    even_bins = []
    if num_bars > 0:
        # Calculate a nominal group size.
        # Integer division, so some bins might be slightly larger if there's a remainder,
        # especially handled by ensuring the last bar goes to the end.
        
        # Distribute bins as evenly as possible
        # basic_group_size = total_fft_bins_count // num_bars
        # remainder = total_fft_bins_count % num_bars
        
        current_start_idx = 0
        for i in range(num_bars):
            # Determine the end index for the current bar
            if i < num_bars - 1:
                # A more robust way to distribute for "evenness" including remainders:
                # Each of the first `remainder` bins gets `basic_group_size + 1` elements.
                # The rest get `basic_group_size`.
                # Simpler approach: make all but last bar roughly equal, last takes rest.
                group_size_for_this_bar = (total_fft_bins_count // num_bars)
                # Add 1 to group_size for the first 'remainder' bars to distribute remainder
                if i < (total_fft_bins_count % num_bars):
                    group_size_for_this_bar +=1
                
                current_end_idx = current_start_idx + group_size_for_this_bar

            else: # For the last bar, ensure it goes to the very end
                current_end_idx = total_fft_bins_count
            
            # Ensure indices are clamped and valid
            # start_index should not exceed total_fft_bins_count
            # end_index should not exceed total_fft_bins_count (as it's exclusive for slices)
            final_start = min(current_start_idx, total_fft_bins_count)
            final_end = min(current_end_idx, total_fft_bins_count)

            # Ensure end is not less than start (can happen if current_start_idx was already at the end)
            if final_end < final_start:
                final_end = final_start
                
            even_bins.append((final_start, final_end))
            current_start_idx = current_end_idx # Next bar starts where this one logically ended

            # Safety: if current_start_idx is already at or beyond the total count,
            # fill remaining bars with empty bins at the end.
            if current_start_idx >= total_fft_bins_count and i < num_bars - 1:
                for _ in range(i + 1, num_bars):
                    even_bins.append((total_fft_bins_count, total_fft_bins_count))
                break
    else: # num_bars is 0 or negative
        pass


    # --- Exponential Spreading ---
    exp_bins = []
    exp_factor = 1.1 
    if num_bars > 0:
        max_bar_f = float(num_bars)
        max_bin_f = float(total_fft_bins_count)

        if exp_factor == 1.0: # Linear spread if exp_factor is 1.0
            for i in range(num_bars):
                start_index_f = (i / max_bar_f) * max_bin_f
                end_index_f = ((i + 1) / max_bar_f) * max_bin_f
                
                start_idx = min(int(start_index_f), total_fft_bins_count)
                end_idx = min(int(end_index_f), total_fft_bins_count)
                if end_idx < start_idx: # Ensure valid range
                    end_idx = start_idx
                exp_bins.append((start_idx, end_idx))
        else: # True exponential spread
            for i in range(num_bars):
                start_ratio = i / max_bar_f
                end_ratio = (i + 1) / max_bar_f

                # Formula: (base^ratio - 1) / (base - 1) * total_range
                start_index_f = (math.pow(exp_factor, start_ratio) - 1.0) / (exp_factor - 1.0) * max_bin_f
                end_index_f = (math.pow(exp_factor, end_ratio) - 1.0) / (exp_factor - 1.0) * max_bin_f
                
                start_idx = min(int(round(start_index_f)), total_fft_bins_count) # Rounding might distribute better
                end_idx = min(int(round(end_index_f)), total_fft_bins_count)

                # Ensure end_index is not less than start_index.
                if end_idx < start_idx and start_idx < total_fft_bins_count:
                    end_idx = start_idx # Create an empty or minimal bin
                
                exp_bins.append((start_idx, end_idx))
    else: # num_bars is 0 or negative
        pass
        
    return even_bins, exp_bins

def print_rust_array(name_prefix, arr, num_bars_for_name, actual_len):
    print(f"// Generated for NUM_BARS = {num_bars_for_name}, TOTAL_FFT_BINS_COUNT = {TOTAL_FFT_BINS_PARAM}") # Use global for comment
    print(f"pub const {name_prefix}_{num_bars_for_name}_{TOTAL_FFT_BINS_PARAM}: [(usize, usize); {actual_len}] = [")
    for item in arr:
        print(f"    ({item[0]}, {item[1]}),")
    print("];\n")

# --- Configuration ---
# 1. USER_NUM_BARS: The number of bars your display will have.
#    This often depends on screen_width (e.g., screen_width / 8).
#    Set this to the specific number you are targeting for this precomputation.
USER_NUM_BARS = 8 # Example: for a screen_width of 240px, if each bar cell is 8px

# 2. TOTAL_FFT_BINS_PARAM: The total count of bins from your FFT.
#    If your FFT gives 512 data points, these are indexed 0 to 511.
#    Set this parameter to 512. The script will ensure calculations correctly
#    handle indices up to 511 (inclusive for start, exclusive for end).
TOTAL_FFT_BINS_PARAM = 512
# --- End Configuration ---

if USER_NUM_BARS <= 0:
    print(f"// USER_NUM_BARS set to {USER_NUM_BARS}. Generating empty arrays if 0, or skipping if negative.")
    if USER_NUM_BARS == 0: # Define empty consts if num_bars is 0
        print_rust_array("EVEN_BIN_INDICES", [], 0, 0)
        print_rust_array("EXP_BIN_INDICES", [], 0, 0)
else:
    even_indices, exp_indices = generate_bin_indices(USER_NUM_BARS, TOTAL_FFT_BINS_PARAM)
    # The length of the generated array will be USER_NUM_BARS
    print_rust_array("EVEN_BIN_INDICES", even_indices, USER_NUM_BARS, len(even_indices))
    print_rust_array("EXP_BIN_INDICES", exp_indices, USER_NUM_BARS, len(exp_indices))