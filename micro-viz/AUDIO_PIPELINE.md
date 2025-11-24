# Audio Visualization Pipeline

This document visualizes the complete audio processing and visualization pipeline in `micro-viz`.

## Pipeline Flow Diagram

```mermaid
flowchart TD
    %% Input Stage
    Audio[Audio Input] --> FFT[FFT Processing<br/>512/256 bins]
    FFT --> Magnitude[Magnitude Spectrum<br/>f32 array]
    
    %% FFT Data Provider (Generic Interface)
    Magnitude --> FftProvider[FftDataProvider<br/>Generic Interface]
    FftProvider --> |get_magnitude_bins| ProviderAPI[Provider API]
    
    %% Main Visualizer
    ProviderAPI --> Visualizer[Visualizer<br/>Main Coordinator]
    Visualizer --> |update| ModeSelect{Visualization Mode}
    
    %% Visualization Modes
    ModeSelect -->|Bars| BarsMode[BarsMode]
    ModeSelect -->|Skinny| SkinnyBarsMode[SkinnyBarsMode]
    ModeSelect -->|Sine| SineWaveMode[SineWaveMode]
    ModeSelect -->|Sawtooth| SawtoothWaveMode[SawtoothWaveMode]
    ModeSelect -->|Square| SquareWaveMode[SquareWaveMode]
    ModeSelect -->|ZigZag| ZigZagMode[ZigZagMode]
    
    %% Processing Pipeline (within each mode)
    BarsMode --> Process[Processing Pipeline]
    SkinnyBarsMode --> Process
    SineWaveMode --> Process
    SawtoothWaveMode --> Process
    SquareWaveMode --> Process
    ZigZagMode --> Process
    
    %% Band Aggregation
    Process --> BandAgg[SpectralBandAggregator<br/>Map FFT bins → Display elements]
    BandAgg --> |band_ranges| BinStrategy[BinSummaryStrategy<br/>Max/Average/Sum]
    
    %% FFT Data Processing
    FftProvider --> |get_band_aggregate| BinStrategy
    BinStrategy --> |aggregated values| Scaling[Scaling Strategy]
    
    %% Scaling and Normalization
    Scaling --> RollingMax[RollingMaxTracker<br/>Dynamic max tracking]
    RollingMax --> |rolling_max| ScaleContext[ScalingContext]
    Scaling --> |scaled values| ScaleContext
    
    %% Animation
    ScaleContext --> Animator[SpectrumValueAnimator<br/>Smooth interpolation]
    Animator --> |current_heights| ColorCalc[Color Calculation]
    
    %% Color Processing
    ColorCalc --> ColorMode{ColorMode}
    ColorMode --> |Spectrum| SpectrumColor[SpectrumColorMode<br/>Rainbow wheel]
    ColorMode --> |Gradient| GradientColor[GradientColorMode<br/>Grayscale]
    ColorMode --> |Static| StaticColor[StaticPaletteColorMode<br/>Fixed palette]
    ColorMode --> |Dynamic| DynamicColor[DynamicFftColorMode<br/>FFT-based]
    ColorMode --> |Shifting| ShiftingColor[ShiftingSpectrumColorMode<br/>Animated]
    
    %% Rendering
    SpectrumColor --> Renderer[Renderer<br/>Drawing Primitives]
    GradientColor --> Renderer
    StaticColor --> Renderer
    DynamicColor --> Renderer
    ShiftingColor --> Renderer
    
    Renderer --> |render_bar| BarDraw[Draw Bars]
    Renderer --> |render_line| LineDraw[Draw Lines]
    Renderer --> |render_waveform| WaveDraw[Draw Waveforms]
    
    %% Output
    BarDraw --> Display[Display Output<br/>LED Matrix/Frame Buffer]
    LineDraw --> Display
    WaveDraw --> Display
    
    %% Styling
    classDef input fill:#e1f5ff,stroke:#01579b,stroke-width:2px
    classDef processing fill:#fff3e0,stroke:#e65100,stroke-width:2px
    classDef mode fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef color fill:#e8f5e9,stroke:#1b5e20,stroke-width:2px
    classDef output fill:#fce4ec,stroke:#880e4f,stroke-width:2px
    
    class Audio,FFT,Magnitude input
    class FftProvider,ProviderAPI,Process,BandAgg,BinStrategy,Scaling,RollingMax,ScaleContext,Animator processing
    class Visualizer,ModeSelect,BarsMode,SkinnyBarsMode,SineWaveMode,SawtoothWaveMode,SquareWaveMode,ZigZagMode mode
    class ColorCalc,ColorMode,SpectrumColor,GradientColor,StaticColor,DynamicColor,ShiftingColor color
    class Renderer,BarDraw,LineDraw,WaveDraw,Display output
```

## Component Details

### Input Layer
- **Audio Input**: Raw audio signal from microphone or audio source
- **FFT Processing**: Fast Fourier Transform converts time-domain to frequency-domain
- **Magnitude Spectrum**: Array of magnitude values (typically 512 bins)

### Data Provider Layer
- **FftDataProvider**: Generic trait interface that abstracts FFT data format
  - `FftArrayProvider`: Wraps `&[f32]` array
  - `FftVecProvider`: Wraps `Vec<f32>`
  - Direct array support: `[f32; N]`

### Visualization Layer
- **Visualizer**: Main coordinator that manages the visualization pipeline
- **VisualizationMode**: Trait for different visualization shapes
  - **BarsMode**: Standard vertical bars
  - **SkinnyBarsMode**: Thin bars with configurable spacing
  - **SineWaveMode**: Smooth sine wave interpolation
  - **SawtoothWaveMode**: Sawtooth waveform
  - **SquareWaveMode**: Square waveform
  - **ZigZagMode**: Zigzag pattern

### Processing Pipeline
1. **SpectralBandAggregator**: Maps FFT bins to display elements
   - Exponential or linear band distribution
   - Maps 512 bins → N display elements (e.g., 32 bars)

2. **BinSummaryStrategy**: Aggregates bin values within each band
   - `Max`: Take maximum value
   - `Average`: Take mean value
   - `Sum`: Sum all values

3. **Scaling Strategy**: Normalizes values to display coordinates
   - `LinearRollingMaxScaling`: Linear scaling with rolling max
   - `LogarithmicScaling`: Logarithmic scaling
   - `SquareRootScaling`: Square root scaling

4. **RollingMaxTracker**: Tracks dynamic maximum for adaptive scaling
   - Updates with decay factor
   - Prevents sudden jumps in visualization

5. **SpectrumValueAnimator**: Smoothly interpolates between value changes
   - Ease-out quadratic interpolation
   - Prevents choppy animations
   - Interpolates over N frames

### Color Processing
- **ColorMode**: Trait for color calculation strategies
  - **SpectrumColorMode**: Rainbow color wheel based on position
  - **GradientColorMode**: Grayscale intensity gradient
  - **StaticPaletteColorMode**: Fixed color palette cycling
  - **DynamicFftColorMode**: Colors based on FFT magnitude
  - **ShiftingSpectrumColorMode**: Animated shifting rainbow

### Rendering Layer
- **Renderer**: Low-level drawing primitives
  - `render_bar()`: Draws vertical bars
  - `render_line()`: Draws lines (for waves)
  - `render_waveform()`: Draws waveforms

### Output Layer
- **Display**: Final output to LED matrix or framebuffer
  - Uses `embedded-graphics` `DrawTarget` trait
  - RGB888 color format
  - Screen dimensions: typically 256x64

## Data Flow Example

```
1. Audio → FFT → [512 f32 magnitude values]

2. FftArrayProvider::new(&magnitude_array)
   → Provides generic FftDataProvider interface

3. Visualizer::update(&mut display, &fft_provider)
   → Selects visualization mode (e.g., BarsMode)

4. BarsMode::render()
   → Gets band ranges from SpectralBandAggregator
   → Aggregates bins using BinSummaryStrategy::Max
   → Scales using LinearRollingMaxScaling
   → Updates SpectrumValueAnimator with new targets
   → Gets current interpolated heights

5. For each bar:
   → Calculate color using ColorMode (e.g., SpectrumColorMode)
   → Render bar using Renderer::render_bar()

6. Display updated with new frame
```

## Frame Update Cycle

```mermaid
sequenceDiagram
    participant App as Application
    participant Viz as Visualizer
    participant Mode as VisualizationMode
    participant Anim as Animator
    participant Color as ColorMode
    participant Render as Renderer
    participant Disp as Display
    
    App->>Viz: update(display, fft_data)
    Viz->>Mode: update(frame_count)
    Viz->>Mode: render(display, fft_data, config)
    
    Mode->>Mode: Check if new cycle start
    alt New cycle
        Mode->>Mode: Calculate new targets from FFT
        Mode->>Anim: set_new_targets(targets)
    end
    
    Mode->>Anim: update_and_get_current_values()
    Anim-->>Mode: current_heights: Vec<u16>
    
    loop For each element
        Mode->>Color: calculate_color(context)
        Color-->>Mode: Rgb888 color
        Mode->>Render: render_bar/line(x, y, color)
        Render->>Disp: Draw to framebuffer
    end
    
    Mode-->>Viz: Ok(())
    Viz-->>App: Ok(())
```

## Configuration Flow

```mermaid
graph LR
    Config[VisualizationConfig] --> Screen[Screen Dimensions<br/>width × height]
    Config --> Elements[Number of Elements<br/>bars/wave points]
    Config --> BandAgg[SpectralBandAggregator<br/>bin → element mapping]
    Config --> BinStrat[BinSummaryStrategy<br/>Max/Average/Sum]
    Config --> Animator[SpectrumValueAnimator<br/>interpolation settings]
    Config --> Spacing[Bar Spacing<br/>pixel spacing]
    
    Screen --> Calc[Calculate num_elements]
    Calc --> BandAgg
    BandAgg --> Animator
```

## Key Design Principles

1. **Modularity**: Each component is independent and swappable
2. **Generic FFT Input**: Works with any FFT format via `FftDataProvider` trait
3. **Pluggable Modes**: Easy to add new visualization shapes
4. **Pluggable Colors**: Easy to add new color strategies
5. **Smooth Animation**: Value animator prevents choppy transitions
6. **Dynamic Scaling**: Rolling max adapts to signal level
7. **no_std Compatible**: All components work in embedded environments

