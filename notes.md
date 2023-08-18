TODO Codi future 

    Revisar formating llibreria
    Revisar variables q sobren 
    Revisar el error handling -> no fa falta ferlo si tenim warnings

TODO Experiments 

    Hiperplans sense Casadi
    Experiments: 
        Casai amb hiperplans -> proved to be computationally too expensive
        
        Cas 1 Highway: 
            Casadi amb Euclidean distance -> Aprox 7/8s horizon (Proved to be computationally expensive for a real time aplication)
            LPV Hiperplanes -> Aprox 7/8s horizon 
            Casai with smaller horizon -> Down to a reasonable foresight horizon
            Discussion: We obtain similar results in terms of performance but with a higher computational time and we need
                        to compromise in order to solve it online with an standard laptop

TODO Paper 

        Expandir una mica la biblio 
        Revisio final post experiments

Resumen setmana 14/08-20/08

        Acabats tests CASADI, tenim una versio q mes o menys
        Escrit el paper
        Encara pasaen algunes coses rares amb ROS CASASI pero de moment ho ignorare
        Estructura dels tests a fer Hiperplanes -> nomes si me la demanen 
            Fer un test per veure si es feasible a nivell de computational times

        Test del botleneck -> nomes si mel demanen

TODO setmana del 21
    
    Repasar el paper 
    Planejar primeres fases de la seguent recerca (deixar la teoria escrita)
    Migrar el projecte a ROS2 i ubuntu 22
    Preparar classe Jan de matlab

TODO setmana del 28
    
    Escriure documentacio
    Ordenar i netejar el git del primer paper (TODO Codi future )
    Preparar classe Jan de matlab
