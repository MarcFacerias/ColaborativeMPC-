
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
        
    Mapa amb escañament -> fet, mes o menos tindria que estar be 
    Acabar lo de ROS
    Revisar metriques temporals -> ja esta fet, a priori funciona
    Testejar la nova refactor de la llibreria -> ja esta fet, a priori funciona
    Acabar el codi de ROS
    Escriure documentacio
    Refactor del git -> ja esta fet, a priori funciona, igual tindriam que afegir igore de fitxers ocults
    afegir docker

TODO Paper 

        Expandir una mica la biblio 
        Revisio final post experiments
        Afegir lo de ROS

TODO QoL

    clean redundant variables in the code (labeled with TODO)