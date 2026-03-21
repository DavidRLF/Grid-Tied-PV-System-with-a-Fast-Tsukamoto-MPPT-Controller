Run the script **`M_MPPT_CONTROLLERS_SIM_V1`** to initialize parameters and simulate the MPPT controller suite. Then open the PV system model **`S_MPPT_CONTROLLERS_SIM_V1`** to explore and run each controller configuration.

For the two-rule Mamdani benchmark introduced in the paper, use **`M_MPPT_CONTROLLERS_SIM_V2`** together with the Simulink model **`M_MPPT_CONTROLLERS_SIM_V2`**, which implements the same control framework with a reduced-rule Mamdani inference stage.

<p align="center">
  <img src="https://github.com/user-attachments/assets/9cce0718-e05e-4d65-95aa-212855f45007" alt="M_MPPT_CONTROLLERS_SIM_V1" width="100%">
  <br><em>Figure 1. Simulation script: <code>M_MPPT_CONTROLLERS_SIM_V1</code>.</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/7579c71e-8c05-4933-89de-7ea386225d80" alt="S_MPPT_CONTROLLERS_SIM_V1" width="100%">
  <br><em>Figure 2. PV system model with controller variants: <code>S_MPPT_CONTROLLERS_SIM_V1</code>.</em>
</p>

---

## Fuzzy MPPT models

Open **`TSUKAMOTO_FUZZY_MODEL_V1`** to inspect the membership functions, rule base, and defuzzification flow for the Tsukamoto controller.

<p align="center">
  <img src="https://github.com/user-attachments/assets/d89231ac-5c52-4729-882f-9c1496a3166e" alt="TSUKAMOTO_FUZZY_MODEL_V1" width="100%">
  <br><em>Figure 3. Tsukamoto fuzzy model: <code>TSUKAMOTO_FUZZY_MODEL_V1</code>.</em>
</p>

---

## ET-only builds on **F28069M** (execution-time measurement)

Use the following models to deploy **execution-time (ET)** builds on TI C2000 **F28069M**. These are intended **only** for ET measurement (GPIO toggle/oscilloscope or Code Execution Profiling):

- **`S_MTET_Tsu_MPPT_CONT_V1`** — Tsukamoto fuzzy MPPT (ET on F28069M)  
- **`S_MTET_Sf_MPPT_CONT_V1`** — State-feedback MPPT (ET on F28069M)  
- **`S_MTET_Pi_MPPT_CONT_V1`** — Proportional integral MPPT (ET on F28069M)  
- **`S_MTET_Ma_MPPT_CONT_V1`** — Mamdani fuzzy MPPT (ET on F28069M)
- **`S_MTET_2RuleMa_MPPT_CONT_V1`** — Two-rule Mamdani benchmark (ET on F28069M)

The two-rule Mamdani model is included exclusively as a computational benchmark to compare execution time with the proposed Tsukamoto controller, as described in the paper.

<p align="center">
  <img src="https://github.com/user-attachments/assets/f99eb780-8102-45b7-8439-b75b833adead" alt="S_MTET_Tsu_MPPT_CONT_V1" width="65%">
  <br><em>Figure 4. ET model — Tsukamoto: <code>S_MTET_Tsu_MPPT_CONT_V1</code>.</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/df888cfb-88ab-46f6-bd79-c00c01b43b68" alt="S_MTET_Sf_MPPT_CONT_V1" width="65%">
  <br><em>Figure 5. ET model — State-feedback: <code>S_MTET_Sf_MPPT_CONT_V1</code>.</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/0e672e90-c2bc-4deb-af6b-cb6ff253d8c9" alt="S_MTET_Pi_MPPT_CONT_V1" width="65%">
  <br><em>Figure 6. ET model — PI: <code>S_MTET_Pi_MPPT_CONT_V1</code>.</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/0d4410f2-45e6-45db-8d28-f4967548be22" alt="S_MTET_Ma_MPPT_CONT_V1" width="65%">
  <br><em>Figure 7. ET model — Mamdani: <code>S_MTET_Ma_MPPT_CONT_V1</code>.</em>
</p>

---

## Quick guide: ET measurement on **F28069M**

Two on-board ET methods are supported:  
**(1) Block Prioritization (BP)** via GPIO toggle + oscilloscope, and  
**(2) Code Execution Profiling (CEP)** via profiling instrumentation.

### Method 1 — Block Prioritization (BP)

1) **Hardware Implementation — set ET parameters (F28069M)**  
Open *Model Configuration Parameters → Hardware Implementation*, select **F28069M**, and enable the ET options as shown.

<p align="center">
  <img src="https://github.com/user-attachments/assets/725450b0-1614-40a1-b01d-cdec5f50ae81" alt="Hardware Implementation parameters" width="60%">
  <br><em>Figure 8. Hardware Implementation parameters for ET builds (F28069M).</em>
</p>

2) **Deploy to Hardware**  
Click **Deploy to Hardware** on the toolbar to build and flash the target.

<p align="center">
  <img src="https://github.com/user-attachments/assets/c05525b4-44d4-4fad-b255-290eacb6a2d1" alt="Deploy to Hardware button" width="55%">
  <br><em>Figure 9. Deploy to Hardware button.</em>
</p>

3) **Connect the board to the serial/COM port**  
Ensure the board is properly connected before deployment.

<p align="center">
  <img src="https://github.com/user-attachments/assets/461a42e1-f0e1-4b0a-9cce-df78797616b1" alt="F28069M connected to serial port" width="70%">
  <br><em>Figure 10. F28069M board connected to the serial/COM port.</em>
</p>

4) **Check the Diagnostic Viewer**  
A successful ET build should appear at the end.

<p align="center">
  <img src="https://github.com/user-attachments/assets/ebc836d2-7125-427b-a907-dc3962ff2bf0" alt="Diagnostic Viewer successful ET build" width="60%">
  <br><em>Figure 11. Successful ET build in Diagnostic Viewer (F28069M).</em>
</p>

5) **Measure ET on the oscilloscope (BP)**  
Use a GPIO toggle around the prioritized block. PWM is **blue**; ET pulse is **yellow**.

<p align="center">
  <img src="https://github.com/user-attachments/assets/bb0e648e-807c-436e-a76f-197b58c0f5f1" alt="On-board ET measurement via BP" width="70%">
  <br><em>Figure 12. On-board ET measurement (BP): PWM (blue) and execution time (yellow).</em>
</p>

---

### Method 2 — Code Execution Profiling (CEP)

- **Repeat the BP build & deploy steps** on **F28069M**.  
- In Simulink, enable **Code Execution Profiling** in *Hardware Implementation → Code profiling / Instrumentation*.

1) **Get profiling data from the target**
```matlab
codertarget.profile.getData('S_MTET_PROP_MPPT_ALG_V1')
```
<p align="center">
  <img src="https://github.com/user-attachments/assets/d6e4c3d1-3695-48a7-92ef-14a8d4795e3c" alt="Get CEP data" width="50%"> 
  <br><em>Figure 13. Retrieving CEP data with <code>codertarget.profile.getData(...)</code>.</em>
</p>

2) **Confirm data availability**
<p align="center">
  <img src="https://github.com/user-attachments/assets/26c7fa11-a5eb-4f8c-9432-8ffc214d3bac" alt="CEP data availability message" width="55%"> 
  <br><em>Figure 14. CEP data availability message.</em>
</p>

3) **Open the CEP report**
```matlab
report(ans)
```
<p align="center">
  <img src="https://github.com/user-attachments/assets/f835e93f-6899-4af4-a6ca-ba5e81377102" alt="Open CEP report" width="65%">
  <br><em>Figure 15. Opening the Code Execution Profiling report.</em>
</p>

4) **Interpret the report (ET)**  
The **execution time (ET)** is shown in the **red** field; multiply it by the **time base** in the **green** field to obtain ET in seconds.
<p align="center">
  <img src="https://github.com/user-attachments/assets/ce5046e9-e839-40ff-8a0b-b8280f45c16b" alt="CEP report — ET and time base" width="80%"> 
  <br><em>Figure 16. CEP report — execution time (red) and time base (green).</em>
</p>

---

## MPPT reference current approximation error analysis

Run **`ErrorMPP_V2`** to evaluate the approximation error of the MPPT reference current derived from Eq. (3) with respect to the full nonlinear PV model under different irradiance and temperature conditions.

This script generates error metrics and visualization surfaces used in the paper.



