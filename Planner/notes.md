TODO Paper 
    
    Continuar amb les revisions diaries
    Expandir una mica la biblio 

TODO Esta setmana

    Dimecres
        Mati: 
            Fer tests aumentant el horitzo del model (com a matlab) 
                Maxim un agent done: un horitzo decent amb un agent 
                Impacte multiples agents: a priori sembla que afecta molt al temps de comput
                Mesurar look ahead al plan local -> afegir a base_class DONE
            investigar pq no va amb horitzons llargs: te pinta de que el model es degenera
            afegir els shades dels markets -> Falta decidir el range de colors 
        
        Tarda: 
            Actualitzar pc casa casadi
            Pujar la bibliografia del nostre paper a mendeley
            triar papers nous que llegir mes o menos
            Fer una mica de plannign de la propera recerca -> de cara a bibliografia tindriam que juntar coses

    Dijous
        Refinar el LPV + pendre les dades
        Revisar CASADI + pendre les dades (tal com esta)
        Bibliografia millora de la alpha

    Divendres
        fer seccio de experiments amb el cas LPV de cara a la setmana que ve

TODO Setmana que ve 
    
    Parlar amb Vicenç seccio experiments
    Plantejar com millorar convergencia OCD
    Començar a llegir papers (3 dies estare fora, me puc imprimir uns pocs)

    Discutir tema condicions inicials amb el Vicenç -> amb horitzons llargs hi ha deadlocks
        Horitzons llargs -> mes realista com a aplicacio i menys freq de calcul f.e ( 3 segons endavant i 1s de computational time)
        Horitzons curts  -> dona menys problemes pero el look ahead es moolt curt 

        Exemple: experiment amb 3 vehicles sobre el mapa "gran" tarda uns 

TODO QoL

    netejar dependencies
    Make more than one lap at a time
    