Files structure

    lib: Main package, containts standalone codes with current planners
        
        plan_lib: 
            distributedPlanner: Containts LPV implementation
                LPV_Planner.py -> planner
                LPV_val.py  -> model (standalone)
            nonLinDistribPlanner: contains CASAI 
                Nl_Planner_Eu -> Casadi planner with euclidean oa
                Nl_Planner_hp -> Casadi planner with planes oa
                base_nl -> shared files between both planners
    
            config: containts config files for the planners
                base_class.py -> base to generate the planning class file
                LPV
                    config.py -> LPV config class 
    
                NL 
                    config.py -> Casadi config class 
    
            IOmodule: module to handle prints, figures and data saving  
                IOmodule.py
    
            mapManager: module to handle all related to the track
                trackInitialization.py
            
            planes: module to handle all related to the plans
                computePlane.py
    
            plotter: specific module with plotting classes
                plot_tools.py
    
            utilities: general useful functions
                misc.py

        install.sh: bash script to install library 

    scripts: Scripts for the standalone simulations
    matlab scripts: Scripts to generate matlab figures
    drawings: folder to store figures used in the paper 

Installation -> This code makes the library available for the current python environtment 

    execute install.sh while being in the lib directory! 
        