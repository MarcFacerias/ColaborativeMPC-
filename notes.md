TODO Codi future 

    Revisar formating llibreria
    Revisar variables q sobren 

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

        Cas 2 Bottleneck
            Comparison with both cases (best performing for in the previous experiment)

TODO Paper 

        Expandir una mica la biblio 
        Revisio final post experiments

TODO semanal
    
    Objectiu d'avui: 

        Tenir una versio LPV i una amb CASADI que funcionin be i treure figures
        Afegir lo de ROS

    Dimarts - Dimecres
        Test codis Casadi 
        Test codis LPV 
    
        Estructura dels tests a fer Highway 
            Decidir metriques LPV per a bona performance 
            Intentar mantenir les metriques amb Casadi 
            Baixar performance per a tenir un bon computing time 

        Estructura dels tests a fer Hiperplanes 
            Fer un test per veure si es feasible a nivell de computational times

    Dijous 
        Escriure experiments 
        Trobar una equivalencia de horitzo entre casadi i LPV per veure la diferencia temporal


    Divendres
        Fer test escañaments

TODO setmana del 21
    
    Repasar el paper 
    Planejar primeres fases de la seguent recerca (deixar la teoria escrita)
    Migrar el projecte a ROS2 i ubuntu 22
    Anar a esmatges i al metge de bcn 

TODO setmana del 28
    
    Escriure documentacio
    Ordenar i netejar el git del primer paper (TODO Codi future )
    Preparar classe Jan de matlab

TODO Fantasia: 
    Afegir plan phd al remarkable 
    habilitar la edicio de pdf 