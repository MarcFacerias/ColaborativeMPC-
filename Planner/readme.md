plotter: script to plot shapes
Controller Object: Original ROS Controller
Controller Distributed Object: WIP Versions of the ROS controller 
deprecated: Old tests/codes 
Utilities: code that Euge wrote regarding general useful techniques 

LIST OF COMMENTED FILES

QUE SABEM

    QUE SABEM: 
        
        La nova implementacio dels codis i les millores no son els culpables dels problemes 

        LPV 
            El segon problema ve amb 4 agents LPV; sembla que el model no calcula be ey -> imagino q pq el thet_e creix massa?
            Amb el original te menos divergencia pero el problema segueix sent semblant 
            ELs plans son equivalents, no cal distribuirlos 
            El fet de tenir q pujar tant el weight en el Ey per a que funcioni fa que lo de separarse regular
            El problema anterior ere del model, s'hauria de revisar

        CASADI -> ho deixare per despres del LPV 
            El problema anterior ere del model, s'hauria de revisar
            Pujan les weights per a que ey i etheta siguin mes petits millora la performance
            Funciona el model no lineal - LPV -> DESCOBRIR PQ I REPLICARU YEAAH

TESTS FETS I RESULTATS
    
    LPV
        Desfer la aproximacio -> No funciona
        Mantenir la aproximacio -> depen del pes a la ey, si no s'en va a la puta
        Amb la aproximacio el i el ey el model mes o menys funciona pero els agents es junten molt 

    NL
        Amb el model normal (aproximat) sembla que funciona
        Els multiples agents tambe funcionen, pero la convergencia es lenta 

        Experiments model no lineal amb CASADI
            
            Sembla q la inicalitzacio afecta molt inclus amb la solucio anterior no acaba de tenir "continuitat" 
            A priori la inicialitzacio q mes mal fa es la de la X 
            Aquest problema es resol amb la implementaico del model LPV, pero seguim tenint el problema del cosEtheta
            Si provem el model no linear sense la aproximacio del eugenio tampoc sembla funcioni

COSES A FER
    
    Revisar la literatura en busqueda de algun model que tingui mes sentit, tal com es comporten els dos models em fa dubter
    Revisar el matlab per si podem fer servir el mateix model

    LPV
        A priori el esquema del codi funciona, divergeix pel model

    NL
        Trobar un model NL que funcioni  


    Quins improvements voldria fer

        Fer els codis de matlab per a plotejar cosetes maques -> Done 
        Pensar com millorar convergencia
        Tractar el cas on els hiperplans es calculen online 

TODO Paper 
    
    Continuar amb les revisions diaries
    Expandir una mica la biblio 

TODO 

    Canviar el plotter per a que tingui shades -> Faltara mirar com fer que els colors representin el temps + tunejar el spacing
    Fer el track en forma de principito + Escalonarlo -> DONE, keep el track feo de cara al codi pro es maco de cada al usuari



    