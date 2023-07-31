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
            ELs plans son equivalents, no cal distribuirlos 
            El fet de tenir q pujar tant el weight en el Ey per a que funcioni fa que lo de separarse regular

        CASADI -> Sembla que ja funciona ^^


TESTS FETS I RESULTATS
    
    LPV
        El model funciona 

    NL
        Amb el model normal (aproximat) sembla que funciona
        Els multiples agents tambe funcionen, pero la convergencia es lenta 


COSES A FER

    LPV
        Pasar el model del CASADI a LPV

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



    