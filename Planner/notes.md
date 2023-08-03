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
            

    Dijous
        Refinar el LPV + pendre les dades -> ho tinc 
        Revisar CASADI + pendre les dades -> ho tinc 
        Bibliografia millora de la alpha  -> podem mirar de fer servir el update amb el delta enlloc de amb la alpha 

        Buscar com actualitzar la alpha -> Convergeix molt be mantenint antics valors 

        TODO: 

            add recording of the OCD iterations  -> Done
            add error handling to the OCD method -> Done
            
        Plantejar com millorar convergencia OCD, still have some issues with current lambda

    Divendres
        fer seccio de experiments amb el cas LPV de cara a la setmana que ve

TODO Setmana que ve 
    
    Parlar amb Vicenç seccio experiments
    Discutir tema condicions inicials amb el Vicenç -> amb horitzons llargs hi ha deadlocks
        Horitzons llargs -> mes realista com a aplicacio i menys freq de calcul f.e ( 3 segons endavant i 1s de computational time)
        Horitzons curts  -> dona menys problemes pero el look ahead es moolt curt 

        Exemple: experiment amb 3 vehicles sobre el mapa "gran" tarda uns 
 
    Dilluns/dimarts -> Fer vistazo a les practiques per portar algo el dia 8 

TODO QoL

    Make more than one lap at a time
    