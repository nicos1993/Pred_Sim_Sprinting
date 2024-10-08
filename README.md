# Pred_Sim_Sprinting

This repository provides all the code and materials in relation to the article titled: _Simulations reveal how touchdown kinematic variables affect top sprinting speed: implications for coaching_  
  
BioRxiv link: X  

All optimal control problems were formulated and solved on a Dell laptop (CPU: 11th Gen Intel(R) Core(TM) i9-11900H @ 2.50GHz, cores: 8, RAM: 32 GB) with Windows OS.  

## Necessary software 
- Matlab (version 2022b was used)
- CasADi (installation instructions can be found here: https://web.casadi.org/get/)

## Running the framework
- To reproduce optimal/nominal sprinting simulation you need to run: `MainFunctions\main_pred_sim_sprinting.m` and set the variable `simulation_type` to `'_Nominal'`  
- To reproduce the horizontal touchdown distance or inter-knee touchdown distance simulations you run the same function, but change `simulation_type` accordingly (e.g., `'_HTD_Plus_1'`)  
  
