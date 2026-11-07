DC Motor Speed Control Using Model Predictive Control (MPC) and Kalman Filter
A Google Colab simulation project that models a DC motor, estimates its
speed and armature current using a discrete-time Kalman Filter, and
regulates motor speed using Model Predictive Control (MPC).

The notebook also investigates measurement and process noise, compares
MPC driven by noisy measurements against MPC driven by Kalman-filtered
states, and provides an interactive parameter-tuning GUI.
Project objectives
- Develop a state-space model of a separately excited DC motor.
- Estimate motor states (angular speed and armature current) from
  noisy sensor measurements using a Kalman Filter.
- Use estimated states as feedback to an MPC controller for speed
  regulation.
- Study how measurement noise and process noise affect estimation and
  control.
- Compare Kalman-filter-based MPC with MPC using raw, noisy
  measurements.
- Visualize motor states, estimates, reference speed, and applied
  armature voltage.
- Provide an interactive GUI to modify motor, noise, and controller
  parameters.
Repository contents
  File                                 Description
  DC_Motor_MPC_Kalman_Filter.ipynb   Main Google Colab/Jupyter notebook
containing the model, Kalman
Filter, MPC, simulations, plots,
noise studies, and interactive GUI.
README.md                          Project overview, mathematical
model, setup, usage, and
interpretation.
System architecture
Target speed (RPM)
       |
       v
     MPC controller <----------------------+
       |                                   |
       v                                   |
Armature voltage                          |
       |                                   |
       v                                   |
    DC motor                               |
       |                                   |
       +----> Actual speed and current     |
       |                                   |
       v                                   |
 Noisy sensor measurements                 |
       |                                   |
       v                                   |
  Kalman Filter                             |
       |                                   |
       +----> Estimated speed and current -+
The motor is simulated as a discrete-time linear system. The plant
states are affected by process disturbances, while the sensor
measurements are affected by measurement noise. The Kalman Filter
combines the model prediction with sensor measurements. MPC predicts
future motor behaviour and calculates the voltage input while respecting
voltage limits.
Mathematical model
The armature-controlled DC motor is represented by the electrical and
mechanical equations:

Electrical equation

\(L`\frac{di_a}{dt}`{=tex}=V_a-Ri_a-K_e`\omega`{=tex}\)

Mechanical equation

\(J`\frac{d\omega}{dt}`{=tex}=K_ti_a-b`\omega`{=tex}\)

where:

  Symbol             Meaning                        Unit
  (V_a)              Armature voltage               V
(i_a)              Armature current               A
(\omega{=tex})   Angular speed                  rad/s
\(R\)              Armature resistance            ohm
\(L\)              Armature inductance            H
\(J\)              Rotor/load inertia             kg m²
\(b\)              Viscous friction coefficient   N m s
(K_t)              Torque constant                N m/A
(K_e)              Back-EMF constant              V s/rad

Choose the state and input vectors as:
```{=tex}
\begin{bmatrix}\omega\\i_a\end{bmatrix}
```
, `\qquad `{=tex}u=V_a \]

The continuous-time state-space model is:

\[ `\dot{x}`{=tex}=Ax+Bu \]

with

\[ A=
```{=tex}
\begin{bmatrix}
-\frac{b}{J} & \frac{K_t}{J}\\
-\frac{K_e}{L} & -\frac{R}{L}
\end{bmatrix}
```
, `\qquad`{=tex} B=
```{=tex}
\begin{bmatrix}
0\\
\frac{1}{L}
\end{bmatrix}
```
The notebook discretizes the model using zero-order-hold discretization:

\(x\_{k+1}=A_dx_k+B_du_k+w_k\)

\(y_k=Cx_k+v_k\)

The default measurement matrix is (C=I_2), meaning both speed and
armature current are measured. (w_k) is process noise and (v_k) is
measurement noise.
Kalman Filter
The Kalman Filter estimates the motor state from the discrete model and
noisy measurements.

Prediction

\(`\hat{x}`{=tex}*{k\|k-1}=A_d`\hat{x}`{=tex}*{k-1\|k-1}+B_du\_{k-1}\)

\(P\_{k\|k-1}=A_dP\_{k-1\|k-1}A_d\^T+Q\)

Measurement update

\(K_k=P\_{k\|k-1}C^T(CP\_{k\|k-1}C^T+R)\^{-1}\)
+K_k(y_k-C`\hat{x}`{=tex}\_{k\|k-1}) \]

\[ P\_{k\|k}=(I-K_kC)P\_{k\|k-1} \]

Here, (Q) is the process-noise covariance, (R) is the measurement-noise
covariance, (P) is the estimation-error covariance, and (K_k) is the
Kalman gain.

In the notebook, the GUI exposes noise standard deviations. The
simulation squares these values to obtain the corresponding diagonal
covariance entries.

## Model Predictive Control (MPC)

At every sampling instant, MPC predicts future motor states over a
finite prediction horizon and minimizes a cost function. The notebook
uses the following form:

\[ J=`\sum`{=tex}\_{j=1}\^{N_p} `\left[
(x_{k+j}-x_{\mathrm{ref}})^TQ_x(x_{k+j}-x_{\mathrm{ref}})
+R_u u_{k+j-1}^2
\right]`{=tex}\]

where:

-   (N_p) is the prediction horizon.
-   (Q_x) weights speed and current tracking errors.
-   (R_u) penalizes large control voltages.
-   (x\_{`\mathrm{ref}`{=tex}}=\[`\omega`{=tex}\_{`\mathrm{ref}`{=tex}},0\]\^T)
    in this implementation.

The optimization is subject to the voltage constraint:

\[ -V\_{`\max`{=tex}}`\leq `{=tex}u_k`\leq `{=tex}V\_{`\max`{=tex}} \]

The optimizer calculates a sequence of future voltage inputs, but only
the first input is applied. The optimization is repeated at the next
sample using the latest state feedback. The implementation uses SciPy's
`minimize` function with the L-BFGS-B method and bounded voltage inputs.

> **Modeling note:** The current reference in the implemented cost
> function is zero. This is a simple weighting choice for the
> demonstration, not a claim that a loaded motor can operate at zero
> current. For a more physically meaningful current-aware design, use a
> suitable current reference or formulate current as a soft
> constraint/penalty based on the motor's operating point.

## Software and dependencies

-   Python 3
-   NumPy
-   SciPy
-   Matplotlib
-   ipywidgets
-   IPython display utilities

The notebook installs the required packages in its first code cell. No
physical motor, microcontroller, or external sensor is required because
this is a software simulation.

## How to run in Google Colab

1.  Download or clone this project.
2.  Open [Google Colab](https://colab.research.google.com/).
3.  Select **File → Upload notebook**.
4.  Upload `DC_Motor_MPC_Kalman_Filter.ipynb`.
5.  Run the notebook cells from top to bottom.
6.  Review the baseline plots and controller comparison.
7.  Run the noise experiments.
8.  Use the interactive GUI to change motor parameters, noise levels,
    target speed, voltage limit, and MPC tuning values.

The GUI uses `ipywidgets`. If the controls do not render, rerun the GUI
cell and check that notebook widget output is enabled in the Colab
environment.

## Default simulation parameters

The default values are illustrative and should be replaced with measured
motor specifications when available.

  Parameter                    Default value
  -------------------------- ---------------
  Armature resistance, (R)           2.0 ohm
  Armature inductance, (L)             0.5 H
  Rotor inertia, (J)              0.02 kg m²
  Viscous friction, (b)            0.2 N m s
  Torque constant, (K_t)                 0.1
  Back-EMF constant, (K_e)               0.1
  Sampling time                       0.02 s
  Simulation duration                    5 s
  Target speed                       100 RPM
  Voltage limit                        ±24 V
  MPC prediction horizon          15 samples

## Simulation outputs

The notebook produces:

1.  **Speed response:** actual speed, estimated speed, noisy measured
    speed, and reference speed.
2.  **Current response:** actual, estimated, and noisy measured armature
    current.
3.  **Armature voltage:** control voltage applied by MPC over time.
4.  **Controller comparison:** motor speed response using direct
    noisy-state MPC and Kalman-state MPC.
5.  **Measurement-noise study:** effect of changing speed measurement
    noise on estimation and tracking RMSE.
6.  **Process-noise study:** effect of changing speed process noise on
    estimation and tracking RMSE.
7.  **Interactive visualization:** plots and performance metrics
    generated from user-selected parameters.

The notebook reports speed estimation RMSE in RPM, current estimation
RMSE in amperes, and speed tracking RMSE in RPM.

## How to interpret the results

-   A lower state-estimation RMSE indicates that the estimate is closer
    to the simulated actual state.
-   A lower speed-tracking RMSE indicates that actual motor speed
    follows the reference more closely.
-   Increasing measurement noise makes raw sensor feedback less
    reliable. A suitably tuned Kalman Filter can reduce the effect of
    noisy measurements.
-   Increasing process noise represents greater uncertainty in the motor
    model or stronger disturbances. The Kalman Filter should generally
    place relatively more weight on measurements as its assumed process
    uncertainty increases.
-   Increasing the MPC speed weight emphasizes speed tracking.
-   Increasing the voltage penalty discourages large control inputs.
-   Increasing the prediction horizon allows MPC to consider a longer
    future interval but increases optimization work.

The expected benefit of combining a Kalman Filter with MPC is cleaner
state feedback for predictive control, especially when sensor
measurements are noisy. The benefit depends on model accuracy,
covariance tuning, controller weights, and the chosen operating
conditions.

## Important implementation notes and limitations

-   This is a simplified linear DC motor model. It does not fully
    represent nonlinear friction, magnetic saturation, dead zones, load
    changes, temperature effects, or parameter uncertainty.
-   The default sensor model measures both speed and current. A
    speed-only sensor configuration would require changing the
    measurement matrix and measurement covariance, and checking
    observability.
-   The Kalman Filter assumes that the model and noise covariance
    assumptions are reasonably representative of the simulated system.
-   MPC performs numerical optimization at every time step. Its
    computation time may be too high for some embedded platforms without
    optimization or a more efficient solver.
-   The simulation uses fixed sampling time and fixed model parameters.
-   The direct-measurement MPC baseline uses noisy measurements
    directly. The notebook's `estimated` state array in that baseline is
    only a model-propagated trace; its state-estimation RMSE should
    **not** be interpreted as a valid direct-measurement estimator
    comparison. The speed-tracking RMSE is the meaningful controller
    comparison metric.
-   The notebook is intended for learning and simulation. It does not
    establish real-time performance or hardware safety.

## Possible future improvements

-   Add load-torque disturbances and motor parameter variations.
-   Compare a speed-only measurement configuration with
    speed-and-current measurements.
-   Add current and voltage constraints to the MPC formulation.
-   Use a more physically appropriate current reference or current
    constraint.
-   Compare different MPC horizons and optimization weights
    systematically.
-   Evaluate average computation time per MPC update.
-   Validate the model against measured data from a real DC motor.
-   Implement the controller on a microcontroller after simulation and
    hardware safety validation.

## Research question

**Does integrating a Kalman Filter with Model Predictive Control improve
DC motor speed regulation when sensor measurements are affected by
noise?**

The simulation addresses this question by comparing state estimation
accuracy and speed-tracking performance under multiple measurement- and
process-noise conditions.

## License

No license has been specified. Add a license file if you intend to
distribute or reuse this project publicly.
