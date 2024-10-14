# Pred_Sim_Sprinting

This repository provides all the code and materials in relation to the article titled: _Simulations reveal how touchdown kinematic variables affect top sprinting speed: implications for coaching_  
  
BioRxiv link: https://www.biorxiv.org/content/10.1101/2024.10.08.617292v1  

All optimal control problems were formulated and solved on a Dell laptop (CPU: 11th Gen Intel(R) Core(TM) i9-11900H @ 2.50GHz, cores: 8, RAM: 32 GB) with Windows OS.  

## Necessary software 
- Matlab (version 2022b was used)
- CasADi (installation instructions can be found here: https://web.casadi.org/get/)

## Running the framework
- To reproduce optimal/nominal sprinting simulation you need to run: `MainFunctions\main_pred_sim_sprinting.m` and set the variable `simulation_type` to `'_Nominal'`  
- To reproduce the horizontal touchdown distance (HTD) or inter-knee touchdown distance (IKTD) simulations you run the same function, but change `simulation_type` accordingly (e.g., `'_HTD_Plus_6'` will generate the simulation that sets HTD 6 cm greater than in the nominal/optimal simulation)  
  
