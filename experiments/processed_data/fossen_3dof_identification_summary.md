# Identificacion Hibrida: Modelo de Fossen 3-DOF + IA Residual (MLP) — PlaDyPos

## 1. Etapa 1: Parametros Fisicos Identificados por Minimos Cuadrados

- $m_{11} = m - X_{\dot{u}} = 219.87\ \text{kg}$ ($X_{\dot{u}} = -194.87\ \text{kg}$)
- $m_{22} = m - Y_{\dot{v}} = 230.54\ \text{kg}$ ($Y_{\dot{v}} = -205.54\ \text{kg}$)
- $m_{33} = I_z - N_{\dot{r}} = 9.58\ \text{kg}\cdot\text{m}^2$
- **Surge ($u$):** $X_u = 136.22\ \text{N}\cdot\text{s/m}$, $X_{uu} = 77.91\ \text{N}\cdot\text{s}^2/\text{m}^2$
- **Sway ($v$):** $Y_v = 109.90\ \text{N}\cdot\text{s/m}$, $Y_{vv} = 52.40\ \text{N}\cdot\text{s}^2/\text{m}^2$
- **Yaw ($r$):** $N_r = 14.46\ \text{N}\cdot\text{m}\cdot\text{s/rad}$, $N_{rr} = 2.20\ \text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$

## 2. Etapa 2: Red Neuronal Residual (MLP Physics-Informed)

- **Ecuacion Hibrida:** $\hat{\boldsymbol{\nu}}_{\text{Fossen+IA}}(t) = \boldsymbol{\nu}_{\text{Fossen}}(t) + \mathcal{NN}_{\boldsymbol{\phi}}(\mathbf{z}_{\text{causal}}(t))$
- **Arquitectura:** Perceptron Multicapa `43 -> 48 (tanh) -> 24 (tanh) -> 3` entrenado con L-BFGS-B y penalizacion de deriva integral.

## 3. Comparacion de Error RMSE: Fossen (Minimos Cuadrados) vs. Fossen + IA Residual

| Experimento | RMSE $u$ Fossen -> **IA** (m/s) | RMSE $v$ Fossen -> **IA** (m/s) | RMSE $r$ Fossen -> **IA** (rad/s) | RMSE $X$ Fossen -> **IA** (m) | RMSE $Y$ Fossen -> **IA** (m) | RMSE $\psi$ Fossen -> **IA** (deg) |
|---|---:|---:|---:|---:|---:|---:|
| `exp_20261103_082035` | 0.091 -> **0.057** | 0.076 -> **0.034** | 0.149 -> **0.063** | 0.17 -> **0.06** | 0.33 -> **0.05** | 20.5 -> **23.0** |
| `exp_20261103_082139` | 0.075 -> **0.030** | 0.061 -> **0.026** | 0.213 -> **0.056** | 0.32 -> **0.06** | 0.93 -> **0.10** | 93.1 -> **24.1** |
| `exp_20261103_082237` | 0.072 -> **0.030** | 0.085 -> **0.030** | 0.154 -> **0.058** | 1.06 -> **0.03** | 0.71 -> **0.10** | 46.3 -> **10.5** |
| `exp_20261103_082337` | 0.077 -> **0.050** | 0.082 -> **0.035** | 0.252 -> **0.066** | 0.30 -> **0.07** | 0.75 -> **0.27** | 119.7 -> **19.6** |
| `exp_20261103_082428` | 0.062 -> **0.032** | 0.086 -> **0.031** | 0.132 -> **0.072** | 0.15 -> **0.01** | 0.48 -> **0.02** | 26.5 -> **9.0** |
| `exp_20261103_082457` | 0.084 -> **0.047** | 0.080 -> **0.034** | 0.146 -> **0.053** | 0.36 -> **0.07** | 0.29 -> **0.04** | 26.8 -> **11.4** |
