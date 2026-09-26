# OV5640 ISP — theory from first principles

Sep 23, 2026 · @niranjan

Companion document: the tuning playbook. This one is the why; that one is the how.

## Sensor timing: HTS, VTS and t\_row

HTS and VTS are the *total* line and frame lengths in the sensor's own clock domain, blanking included. They are the denominators of every timing number in the sensor.

- **HTS** (Total Horizontal Size, 0x380C/0x380D) = the length of one row period, counted in pixel-clock periods. Default 0x0B1C = 2844.
- **VTS** (Total Vertical Size, 0x380E/0x380F) = the number of row periods in one frame, counted in rows. Default 0x07B0 = 1968.
- **t\_row** (also called the line time or 1H) = the time to read out one row, including the horizontal blanking that follows it.

```latex
t_{row} = \frac{HTS}{f_{PCLK}}, \qquad t_{frame} = VTS \cdot t_{row} = \frac{HTS \cdot VTS}{f_{PCLK}}
```

HTS is always larger than the output width and VTS larger than the output height. The extra is blanking: time for the row to settle on the column ADCs, for the ISP line buffers to drain, and for the output interface to keep up. Blanking is not waste, it is the slack the pipeline needs.

### Working the defaults backwards

The datasheet's reset values are the full 5 MP mode at 15 fps:

```latex
f_{PCLK} = HTS \cdot VTS \cdot fps = 2844 \times 1968 \times 15 \approx 84\ \text{MHz}
```

```latex
t_{row} = \frac{2844}{84 \times 10^{6}} \approx 33.9\ \mu s
```

That number is worth remembering, because it explains the banding registers. One half-cycle of 50 Hz mains light lasts 10 ms, which is 10 ms / 33.9 us = 295 rows. The datasheet's default B50 step (0x3A08/0x3A09) is 0x127 = 295. The 60 Hz default is 0xF6 = 246, and 295/246 = 1.2 = 60/50. The whole flicker system is built on t\_row.

### What each knob actually does

| Change | Effect |
| --- | --- |
| Raise HTS | Longer t\_row, lower frame rate, coarser exposure quantisation |
| Raise VTS | Lower frame rate, longer maximum exposure, same t\_row |
| Raise PCLK | Higher frame rate, everything else fixed |

VTS is the one you move at runtime. Exposure is counted in rows and cannot exceed the frame, so a long exposure needs VTS raised first. HTS is normally fixed per mode, because moving it changes t\_row and therefore invalidates the banding steps and the AEC exposure table.

## Rolling shutter and exposure

The OV5640 has one set of column ADCs shared by the whole array, so rows are reset and read one after another. Row *r* integrates over a window of the same length as every other row, but shifted in time by r x t\_row.

```latex
\text{row } r: \quad \big[\, t_0 + r\,t_{row},\ \ t_0 + r\,t_{row} + E\,t_{row} \,\big]
```

E is the exposure in rows. This is why exposure is counted in rows at all: the exposure is created by delaying the read pointer E rows behind the reset pointer, so the only natural unit is the row period. The registers 0x3500-0x3502 hold E x 16, and the bottom 4 bits must be zero because the OV5640 cannot do fractional lines.

The ceiling is the frame:

```latex
E \le VTS + \{0x350C, 0x350D\} - 4
```

So the exposure ladder is quantised in units of t\_row and capped by VTS. Both come straight from the previous section.

### Three consequences

**Geometric skew.** The top and bottom of the frame are captured a full readout apart. A vertical pole crossing the frame at velocity v is imaged as a slanted line with slope v x t\_row per row. At 33.9 us per row and 1944 rows, the top and bottom of a full-resolution frame are 66 ms apart. That is the classic jello effect.

**Flicker banding.** Because each row starts at a different phase of the mains flicker, rows integrate different amounts of light unless the exposure is an exact multiple of the flicker half-period. This is developed in the exposure section of the tuning playbook.

**Timestamping.** For visual-inertial work the frame does not have a single timestamp. The correct model is:

```latex
t_{r} = t_{\text{frame start}} + r\,t_{row} + \tfrac{1}{2} E\,t_{row}
```

The exposure midpoint is the effective sample time of that row, and it moves when AEC changes E. A calibration that treats the frame as instantaneous absorbs t\_row x rows of error into the extrinsics.

### Global reset (FREX)

Section 4.10.2 of the datasheet describes FREX mode, where the whole array starts integrating at once and a shutter closes before readout. It removes skew but needs external control and is not usable together with the rolling strobe function.

## Mirror and flip

Mirror reverses the column readout order and flip reverses the row order. Both happen in the readout sequencer, so they cost nothing and do not resample the image.

The one thing that is not free is the **Bayer phase**. The colour filter array is a fixed 2x2 tile on the die. Reversing the readout by one row or one column shifts which colour lands first:

| Readout | First 2x2 seen by the ISP |
| --- | --- |
| Normal | BG / GR |
| Mirror | GB / RG |
| Flip | GR / BG |
| Mirror + flip | RG / GB |

If the demosaicer is not told, red and blue swap and every edge shows colour fringes. That is why each direction has two bits:

- 0x3820\[1\] / 0x3821\[1\] change the sensor readout order.
- 0x3820\[2\] / 0x3821\[2\] tell the ISP about it.

Section 4.1 says the ISP auto-detects the row colour for flip, so in practice you set the pair together and check the output. There is a second-order effect: because the readout start address is defined in sensor coordinates, mirroring can shift the effective window by one pixel, which flips the phase again. If colours look wrong after enabling mirror, adjusting the X start or X offset by 1 is the usual fix.

Everything downstream that is spatially indexed must follow the same orientation: the lens shading map, the AEC zone weights, and any camera calibration. Set orientation once at bring-up and treat it as fixed.

## Black level

A pixel in total darkness does not read zero. Two things sit under the signal:

- **ADC and amplifier offset**, a fixed pedestal, roughly constant.
- **Dark current**, thermally generated carriers that accumulate during integration. It grows linearly with exposure time and roughly doubles every 6-8 degC.

```latex
x = S + D(T)\,E\,t_{row} + P + n
```

The correction is a subtraction with a deliberate offset left behind:

```latex
x' = (x - \hat b) + b_{\text{target}}
```

The OV5640 does not need you to measure b-hat in a dark box. The array has optically shielded rows that see no light, and the BLC block averages them every frame, so the estimate tracks temperature and exposure by itself. A dark-box capture is still how you *verify* the result and how you would characterise dark current, but it is not how the sensor derives its black level.

### Why the target is not zero

Register 0x4009 sets the target at 16 in the 10-bit range rather than 0. The reason is clipping. Noise around the true black level is roughly symmetric, so after subtracting to exactly zero, half of it would be clipped away. For zero-mean Gaussian noise:

```latex
\mathbb{E}\big[\max(0, X)\big] = \frac{\sigma}{\sqrt{2\pi}} > 0
```

The clipped mean is positive and, worse, it is larger for the noisier channel. That lifts and tints the shadows. Keeping a pedestal preserves the symmetric distribution; a later stage removes it (LENC has an add-BLC-back bit, 0x5841\[3\]).

### Why it must come first

Every later block assumes the signal is proportional to light. That is only true after the pedestal is gone.

- Digital gain must compute g(x - b), not gx - b, or the pedestal scales too. This is why digital gain lives inside the BLC block on this sensor.
- Lens shading multiplies by a gain map. An uncorrected pedestal gets multiplied along with the signal, so corners end up brighter in the shadows.
- White balance multiplies each channel by a different gain, so an uncorrected pedestal becomes a colour cast in the black.

A black level error is therefore not a brightness error, it is a contrast, shading and colour error all at once.

### The controls

0x4000\[0\] enables BLC. 0x4003\[7\] triggers a recalibration over N frames. 0x4003\[6\] freezes the current value, and 0x4005\[1\] makes it update continuously. Continuous update tracks thermal drift but is vulnerable if stray light reaches the shielded rows.

## Lens shading and chief ray angle

LENC corrects **brightness and colour falloff**, not geometry. A grid or checkerboard chart measures barrel and pincushion distortion, which this sensor does not correct at all. The input LENC needs is a **flat field**: a uniformly lit featureless white surface filling the frame. What you are measuring is how much darker, and how much more pink or green, a corner is than the centre.

### Where the falloff comes from

**Natural vignetting, the cos^4 law.** For an off-axis field point at angle theta:

```latex
E(\theta) = E_0 \cos^{4}\theta
```

One cosine comes from the tilted exit pupil projecting a smaller area, one from the tilted sensor patch, and two from the inverse-square increase in distance. At 30 degrees off axis this alone costs 44% of the light.

**Mechanical vignetting.** Barrel walls and apertures clip part of the cone for off-axis points.

**Pixel-level angular response**, which is the CRA story below.

### Chief ray angle

The chief ray is the ray from a field point that passes through the **centre of the aperture stop**. It is the axis of the cone of light that lands on that pixel. The chief ray angle (CRA) is the angle between that ray and the sensor normal at the point where it lands.

```latex
\text{CRA}(h) = \arctan\!\left(\frac{h}{d_{\text{XP}}}\right)
```

Here h is the image height, the distance from the optical axis on the sensor, and d\_XP the distance from the exit pupil to the sensor. In the centre the CRA is 0 and the light arrives straight on. At the corner of a short, wide lens it can be 25-30 degrees.

This matters because a pixel is not a flat bucket. It is a stack: microlens, colour filter, then a photodiode at the bottom of a well. The microlens focuses the cone onto the diode. At a steep angle the focused spot walks sideways, hits the well wall or the neighbouring pixel, and that pixel loses sensitivity and gains crosstalk.

Sensor makers fix this by **shifting the microlenses** progressively outwards, so each pixel's microlens is aimed at the CRA it expects to receive. That means a sensor has a designed CRA profile, typically quoted as a maximum CRA at the corner. Pairing a lens whose CRA profile does not match the sensor's is the single most common cause of severe corner shading, and no amount of LENC gain fully recovers it, because the light genuinely went into the wrong pixel.

### Why shading is coloured

Two mechanisms make the falloff channel-dependent:

1. **Interference IR-cut filters** pass a band whose cut-off wavelength shifts towards blue with incidence angle, roughly as the square of the angle. At the corner, the filter clips more red than it does in the centre, so corners go cyan.
2. **Crosstalk** into neighbouring pixels is wavelength-dependent, because longer wavelengths penetrate deeper into silicon before converting. Red carriers diffuse further and leak into neighbours more.

This is why the OV5640 stores **separate maps per channel**: a 6x6 grid for green (0x5800-0x5823) and 5x5 grids for blue and red packed as nibbles (0x5824-0x583C) with offsets in 0x583D.

### The correction

```latex
x'(u,v) = G_c(u,v)\, x(u,v), \qquad G_c(u,v) = \frac{F_c(u_0,v_0)}{F_c(u,v)}
```

F\_c is the measured flat-field response of channel c and (u0,v0) the optical centre. Between grid nodes the gain is bilinearly interpolated. The HSCALE and VSCALE registers (0x5842-0x5849) hold the *reciprocal* of the block size, so the hardware finds a block with a multiply and a shift instead of a divide.

The cost of correction is noise. Multiplying a corner by 2 multiplies its noise by 2, so the corner SNR drops even as the brightness matches. That is why the block is gain-adaptive: 0x583E, 0x583F and 0x5840 reduce the correction strength as sensor gain rises, trading corner brightness for corner noise in the dark.

## Auto white balance

### The problem

What a pixel records is the product of three spectra integrated over wavelength:

```latex
I_c = \int L(\lambda)\, R(\lambda)\, S_c(\lambda)\, d\lambda, \qquad c \in \{R, G, B\}
```

L is the illuminant, R the surface reflectance, S\_c the camera's spectral sensitivity. You measure three numbers and want to recover R, but L is unknown. The problem is fundamentally under-determined: a white sheet under tungsten and an orange sheet under daylight can produce identical RGB. Human vision solves it by context and adaptation; a camera has to guess, and every AWB algorithm is a different prior about what scenes look like.

### The von Kries approximation

If the sensitivities were narrowband, changing the illuminant would scale each channel by a constant. Real sensitivities are broad, but the diagonal model is close enough to be universal:

```latex
\begin{pmatrix} R' \\ G' \\ B' \end{pmatrix} = \operatorname{diag}(g_R, g_G, g_B) \begin{pmatrix} R \\ G \\ B \end{pmatrix}
```

So AWB reduces to estimating two numbers, since one channel (usually G) is held at 1. The natural coordinates are the chromaticities r = R/G and b = B/G, which are independent of brightness.

### The priors

**Grey world.** Assume the scene reflectance averages to grey, so the average colour *is* the illuminant:

```latex
g_R = \frac{\bar G}{\bar R}, \qquad g_B = \frac{\bar G}{\bar B}
```

Cheap, one accumulator per channel, and it is what 0x5183\[7\] calls simple AWB. It fails whenever the assumption fails: a field of grass, a red brick wall, a person filling the frame.

**White patch / max-RGB.** Assume the brightest pixels are specular highlights, which reflect the illuminant directly. Fragile because clipped pixels lie.

**Illuminant gamut, which is advanced AWB.** Real illuminants are not arbitrary. Daylight and blackbody sources trace a one-dimensional curve, the Planckian locus, through (r, b) space, with fluorescents scattered nearby. Restricting the estimate to a narrow band around that curve removes most of the degrees of freedom that grey world gets wrong. A strongly coloured scene then pulls the estimate only along the locus, which is why a green field comes out green rather than neutralised to grey. The window for this is FAE-tuned on the OV5640 (0x5186-0x5190).

### How the hardware supports it

- **Statistics gathering.** The frame is divided into zones and only pixels that qualify as white candidates are accumulated. 0x5191 and 0x5192 set top and bottom limits: clipped pixels are excluded because their chroma is wrong, and near-black pixels because their chroma is noise. 0x5193-0x5195 cap the per-channel results.
- **Temporal filtering.** 0x5185 provides a stable/unstable hysteresis, exactly the same idea as the AEC stable band, and 0x5181/0x5182 set step size and speed. Without it, gains hunt whenever someone walks through the frame.
- **Readback and override.** 0x519F-0x51A4 report the current gains at 12-bit resolution. Setting 0x3406\[0\] switches to manual, with gains written to 0x3400-0x3405.

### Its place in the pipeline

AWB gains are applied in the raw domain, before gamma and before the colour matrix. This ordering matters for tuning: the colour matrix is derived assuming its input is already white balanced, so a CCM measured with the wrong gains is wrong everywhere, and gamma applied before white balance would make the gains non-linear and break the diagonal model.

## Colour matrix

### What it fixes

White balance makes neutrals neutral. It does nothing for saturated colours, because the camera's filters are not the human cone responses. The red filter passes some green light, the green filter passes some blue, and so on. That spectral overlap is the crosstalk the matrix removes:

```latex
\begin{pmatrix} R \\ G \\ B \end{pmatrix} = C \begin{pmatrix} R_0 \\ G_0 \\ B_0 \end{pmatrix}, \qquad \sum_j c_{ij} = 1
```

The row-sum constraint is what keeps white white. Without it the matrix would undo the white balance you just set. The off-diagonal terms are negative, because removing a contaminant means subtracting it.

### Solving for it

Stack the measured patch colours as columns of M and the reference values as columns of T, then solve a constrained least-squares problem:

```latex
\min_{C} \; \lVert CM - T \rVert_F^2 \quad \text{s.t.} \quad C\mathbf{1} = \mathbf{1}
```

The unconstrained solution is C = T M^T (M M^T)^-1, and the constraint is handled with a Lagrange multiplier or by reparameterising each row with two free variables. In practice the error is minimised in a perceptually uniform space (CIELAB delta-E after applying gamma) rather than in linear RGB, because equal RGB errors are not equally visible.

### The noise cost

For independent per-channel noise, output channel i has noise gain:

```latex
\sigma_i = \sigma \sqrt{\textstyle\sum_j c_{ij}^2}
```

A strongly saturating matrix with large negative off-diagonals can easily double chroma noise. This is the central trade-off: colour accuracy against chroma noise. It is also why many tunings blend the matrix towards identity at high gain.

### Does it depend on the illuminant?

Yes. If the camera's sensitivities were an exact linear transform of the human observer's (the Luther-Ives condition), one matrix would work under every light. They are not, so the optimal C differs per illuminant. Two matrices measured under A (2856 K) and D65 can differ by 10-20% in their off-diagonal terms.

The standard answer is to calibrate under two or three illuminants, typically A, a fluorescent such as TL84 or CWF, and D65, then interpolate between them at runtime using the estimated colour temperature.

The OV5640 has **one** CMX register bank, so it cannot interpolate by itself. You have two options:

1. Fit a single compromise matrix over patches captured under all your illuminants. Simplest, and usually good enough for machine-vision work.
2. Precompute two or three matrices and have firmware switch or blend them, using the AWB gain readback (0x519F-0x51A4) as a colour temperature proxy. The ratio of R gain to B gain is monotonic in colour temperature.

### OV5640 specifics

The hardware folds the RGB-to-YUV conversion into the same matrix, so the nine registers hold the product, not the CCM itself:

```latex
\text{CMX} = \text{RGB2YUV} \cdot C
```

CMX1-3 (0x5381-0x5383) form the Y row, CMX4-6 the U row, CMX7-9 the V row, with magnitudes in the registers and signs collected in 0x538A and 0x538B. Register 0x5380\[1\] chooses the fixed-point format, 1.7 or 2.6. So design C in linear RGB, multiply by your YUV matrix, then quantise. The matrix figure printed in section 5.7 of the datasheet is garbled by the PDF, so do not copy it literally.

## Raw gamma

### Why a curve exists at all

The sensor is linear: double the photons, double the code. Human brightness perception is not. Lightness follows roughly a cube root of luminance:

```latex
L^* = 116\,\left(\frac{Y}{Y_n}\right)^{1/3} - 16
```

If you send linear data to a display and view it, the shadows are crushed and the highlights waste codes. Encoding with roughly y = x^(1/2.2) distributes code values so that equal code steps are approximately equal perceptual steps. That is an information-theoretic argument, not an aesthetic one: with 8 bits out, a linear encoding needs about 11-12 bits to avoid visible banding in the shadows, while a gamma encoding does not.

The datasheet's phrasing, "compensate for the non-linear characteristics of the sensor", is the historical CRT framing. The modern reading is: it is the transfer function that maps the linear capture to the output encoding, and it doubles as the tone curve where you place contrast.

### The noise consequence

Noise is amplified by the local slope of the curve:

```latex
\sigma_y \approx \lvert f'(x) \rvert \, \sigma_x
```

A pure power law has f'(x) -> infinity as x -> 0, so a steep toe multiplies shadow noise dramatically. This is exactly why sRGB, Rec.709 and every practical curve use a **linear segment near black**. The first knot of your gamma table is the single most important one for perceived noise.

The inverse applies at the top: a flat highlight slope compresses highlight contrast, which is desirable (it mimics film shoulder roll-off) but loses discrimination between bright tones.

### The OV5640 implementation

The curve is piecewise linear with 15 stored knot outputs, YST00 to YST0E at 0x5481-0x548F, plus an end slope at 0x5490 (0x5480\[1\] enables manual control of it). The input breakpoints are fixed in hardware and not published in this datasheet; they are non-uniform, closely spaced in the shadows where the curve bends most and widely spaced in the highlights where it is nearly straight.

Between knots the hardware interpolates linearly, so the curve you get is a polyline through your points. Two properties must hold:

- **Monotonicity.** YST values must strictly increase, or tones invert.
- **Smooth slope changes.** A sudden slope change between two segments shows as a visible contour on smooth gradients such as a sky or a wall.

On this sensor it is RAW gamma: it is applied in the Bayer domain, before demosaic and before the colour matrix. That has a side effect worth knowing about. Because the curve is compressive, applying it before the matrix means the matrix operates on non-linear data, which makes the matrix a mild approximation. It is a hardware ordering constraint, not a choice.

## Defect pixel cancellation

### The physics

Silicon defects and process contamination create pixels that do not behave like their neighbours.

- **Hot or white pixels** have elevated dark current from a leakage path, often a crystal defect or a metal impurity in the depletion region. Their error grows with exposure time and with temperature, so a pixel that is invisible at 1 ms can be saturated at 100 ms. Night mode makes them appear.
- **Dead or black pixels** have a broken photodiode or a broken transfer path and read near the pedestal regardless of light.
- **Coupled pixels** sit between the two, responding but with the wrong slope.

Some are present at wafer test, and more accumulate over the sensor's life, notably from cosmic ray damage. A fixed defect table burned at manufacture is therefore never complete, which is why this block detects dynamically every frame.

### Detection as a hypothesis test

In the Bayer domain, the neighbours of the same colour are two pixels away. The eight of them form the reference set N. The classic rule is an order-statistic test:

```latex
\text{white: } p > \max(N) + T \qquad \text{black: } p < \min(N) - T
```

and the replacement is usually the median of N, chosen because it is robust when two defects are adjacent.

The interesting part is what T should be. The test compares a pixel against the local signal variation, and that variation has two sources: real image detail, and noise. Noise grows with the square root of the signal, so a fixed T is wrong at both ends of the range. A well-tuned threshold is signal-dependent:

```latex
T(p) \approx k \sqrt{\sigma_{\text{read}}^2 + p/g}
```

### The trade-off

This is a detector, so it has both error types:

- **T too small:** false positives. The filter erases genuine single-pixel detail, and it cannot tell a hot pixel from a distant streetlight, a specular glint on a wire, or a star. On resolution charts it destroys the finest line pairs, which shows up as an apparent loss of MTF.
- **T too large:** false negatives. Weak hot pixels survive and appear as fixed coloured speckles that are especially objectionable because, unlike noise, they do not move between frames. A static bright dot also becomes a phantom feature that a corner detector will track happily, which corrupts any structure-from-motion or VIO pipeline.

The OV5640 exposes only two enable bits, 0x5000\[2\] for black and 0x5000\[1\] for white cancellation, with the thresholds FAE-tuned. Practically, you decide whether each is on, at which gain levels, and you verify against the two failure modes above.

## Colour interpolation (CIP)

CIP is three operations sharing one block: raw denoise, demosaic, and edge enhancement. They are grouped because they all work on the same local neighbourhood and must be ordered carefully. Denoising after demosaic would smear interpolation errors; sharpening before demosaic would amplify them.

### Demosaic

Each pixel measures one colour. Green is sampled on a quincunx at half the pixels; red and blue at a quarter each, on a lattice with half the sampling frequency in each axis.

**Bilinear** interpolation is the naive answer. For a missing green at a red site:

```latex
\hat{G} = \tfrac{1}{4}\left(G_N + G_S + G_E + G_W\right)
```

It averages across edges as happily as along them, which produces the two classic artefacts: **zippering** (alternating light and dark pixels along a near-horizontal edge) and **false colour** (coloured fringes where luminance detail exceeds the chroma sampling rate).

**Edge-directed** interpolation, the Hamilton-Adams family, first estimates which direction is safe by combining a green gradient with a second difference from the same-colour channel:

```latex
\Delta_H = \lvert G_W - G_E \rvert + \lvert 2R_0 - R_{WW} - R_{EE} \rvert
```

and the analogous Delta\_V. Interpolate along the smaller gradient. The second-difference term is what lets a red pixel contribute information about green structure: the two channels are correlated because most surfaces are broadband.

**Colour difference interpolation.** Once green is complete, red and blue are interpolated not directly but through the differences R - G and B - G. These are nearly constant across an object, because a surface's hue changes slowly even where its brightness changes fast. This single idea removes most false colour.

### The Nyquist limit

Red and blue are sampled at half the spatial frequency of luminance. Above that limit, chroma aliases: fine repetitive texture such as fabric or a resolution chart produces moire in colour that no post-processing can remove, since the information was aliased at capture. The optical cure is a blur filter in front of the sensor; small sensors like this one usually rely on the lens being soft enough to do the job.

### Denoise and sharpen

Sharpening is unsharp masking with a coring threshold:

```latex
y = x + k \cdot \mathcal{C}_T\!\big(x - h*x\big), \qquad \mathcal{C}_T(d) = \operatorname{sign}(d)\max(\lvert d \rvert - T, 0)
```

Here h is a blur kernel, so the convolution of h with x is a blurred copy and the difference between x and that blurred copy is the local detail. Without coring, the operator amplifies noise exactly as much as detail, since both live in the high-frequency band. Coring is the soft threshold that says: below T, assume it is noise and pass nothing. In the registers, MT plays the role of strength k and TH the role of threshold T.

Denoise is the same decision run the other way: differences below a threshold are averaged away, differences above it are preserved as edges.

### Gain scheduling

The correct T is not constant, because noise grows with gain. That is why each parameter has a threshold1/threshold2 pair and an offset1/offset2 pair: the parameter ramps linearly with sensor gain between the two thresholds. The defaults show the intent clearly.

| Parameter | Low gain | High gain | Direction |
| --- | --- | --- | --- |
| Denoise (0x5306, 0x5307) | 0x09 | 0x16 | Stronger |
| Sharpen strength MT (0x5302, 0x5303) | 0x18 | 0x0E | Weaker |
| Sharpen threshold TH (0x530B, 0x530C) | 0x04 | 0x06 | Higher |

In bright light the image is clean, so sharpen hard and denoise lightly. In the dark, back off sharpening (it would amplify noise), raise the coring threshold, and denoise harder. Manual overrides are 0x5308\[6\] for sharpen and 0x5308\[4\] for denoise, and the live values can be read at 0x530D-0x530F.

### If the output feeds an algorithm rather than an eye

Sharpening creates overshoot at edges, which shifts the apparent position of a gradient peak and biases subpixel corner and edge localisation. Denoising removes exactly the fine texture that feature descriptors rely on. For a vision pipeline, prefer a mild setting or disable enhancement entirely, and tune for the detector rather than for the viewer.
