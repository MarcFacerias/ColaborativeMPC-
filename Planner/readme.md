Files structure

    Planner: Main package, containts standalone codes with current planners
        test scripts: some module tests
        matlab scripts: matlab plot scripts
        drawings: drawings from the paper
        data: data from the paper

        planners: Containts the planner files
            distributedPlanner Containts LPV implementation
                LPV_Planner.py -> planner
                LPV_val.py  -> model (standalone)
            nonLinDistribPlanner
                Nl_Planner_Eu -> Casadi planner with euclidean oa
                Nl_Planner_hp -> Casadi planner with planes oa
                base_nl -> shared files between both planners
            
        packages
            config -> containts config files for the planners
                base_class.py -> base to generate the planning class file
                LPV
                    config.py -> LPV config class 

                NL 
                    config.py -> Casadi config class 

            IOmodule -> module to handle prints, figures and data saving  
                IOmodule.py

            mapManager -> module to handle all related to the track
                trackInitialization.py
            
            planes  -> module to handle all related to the plans
                computePlane.py

            plotter -> specific module with plotting classes
                plot_tools.py

            utilities -> general useful functions
                misc.py



        