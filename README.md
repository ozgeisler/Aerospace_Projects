# Dominant Poles Approximation in Control Systems

This project investigates the **dominant pole approximation** concept by comparing a third-order system with its second-order approximation in the time domain.

The effect of the third pole parameter \( p_3 \) is analyzed through:

- Step response comparison
- Percent overshoot
- Settling time (4%)
- Rise time (10% → 80%)

---

## 1️⃣ System Definition

### Full Third-Order System

\[
G(s) =
\frac{s + 2}
{(s + 1 + 3i)(s + 1 - 3i)\left(\frac{1}{p_3}s + 1\right)}
\]

Poles:
- Complex conjugate poles: \( -1 \pm 3i \)
- Third pole: \( -p_3 \)

Zero:
- \( -2 \)

---

### Dominant-Pole Approximation

Neglecting the third pole:

\[
\tilde{G}(s) =
\frac{s + 2}
{(s + 1 + 3i)(s + 1 - 3i)}
\]

Expanded denominator:

\[
(s+1+3i)(s+1-3i) = (s+1)^2 + 9 = s^2 + 2s + 10
\]

\[
\tilde{G}(s) = \frac{s+2}{s^2 + 2s + 10}
\]

---

## 2️⃣ Objective

The goal is to study:

- When is the third pole negligible?
- How does varying \( p_3 \) affect system performance?
- When does the second-order approximation become accurate?

---

## 3️⃣ Methodology

The following steps are implemented in MATLAB:

1. Define both full and approximate transfer functions
2. Compute step responses
3. Calculate performance metrics numerically:
   - Percent Overshoot
   - Settling Time (4% band)
   - Rise Time (10% → 80%)
4. Sweep \( p_3 \) over a range
5. Compare results graphically

---

## 4️⃣ Step Response Comparison

For a fixed large value of \( p_3 \), the third pole moves far to the left in the s-plane.

When \( p_3 \) is large:
- The third pole becomes fast
- Its influence decays quickly
- The system behaves like a second-order system

---

## 5️⃣ Parameter Study: Effect of \( p_3 \)

Subplots are generated for:

\[
p_3 = 0.1, \; 1, \; 3, \; 10
\]

Observations:

- Small \( p_3 \) → third pole is near the imaginary axis  
  → Strong impact on dynamics  
  → Approximation is poor  

- Large \( p_3 \) → third pole far left  
  → Fast decay  
  → Approximation becomes accurate  

---

## 6️⃣ Performance Metrics vs \( p_3 \)

The following metrics are plotted as functions of \( p_3 \):

- Percent Overshoot
- Settling Time (4%)
- Rise Time (10%–80%)

The approximate system metrics are shown as horizontal reference lines.

Key Insight:

As \( p_3 \to \infty \),

\[
G(s) \to \tilde{G}(s)
\]

and performance metrics converge.

---

## 7️⃣ Important Note

Classical second-order analytical formulas (based only on \( \zeta \) and \( \omega_n \)) do not perfectly predict overshoot and settling time here because:

- The system contains a zero at \( s = -2 \)
- The numerator is not constant

Therefore, performance metrics are computed numerically from the time response.

---


