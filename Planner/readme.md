plotter: script to plot shapes
Controller Object: Original ROS Controller
Controller Distributed Object: WIP Versions of the ROS controller 
deprecated: Old tests/codes 
Utilities: code that Euge wrote regarding general useful techniques 

LIST OF COMMENTED FILES

TODO DEL DESASTRE

    QUE SABEM: 
        LPV 
            El segon problema ve amb 4 agents LPV; sembla que el model no calcula be ey -> imagino q pq el thet_e creix massa?
            Amb el original te menos divergencia pero el problema segueix sent semblant 
            ELs plans son equivalents, no cal distribuirlos 
            El fet de tenir q pujar tant el weight en el Ey per a que funcioni fa que lo de separarse regular

        CASADI -> ho deixare per despres del LPV 
            El problema anterior ere del model, s'hauria de revisar
            Pujan les weights per a que ey i etheta siguin mes petits millora la performance

    Quins testos fer: 
            Mirar el model i veure els Etheta, NO es podem asumir angles petits
            Un cop resolt aixo mirar com respon en el cas de baixar el reward de ey
            escriure el model totalment no linear al CASADI

    Quins improvements voldria fer

        Fer els codis de matlab per a plotejar cosetes maques -> Done 
        Pensar com millorar convergencia
        Tractar el cas on els hiperplans es calculen online 

TODO Paper (A part del codi jajaja salu2)
    
    Continuar amb les revisions diaries
    Expandir una mica la biblio 

TODO DEMA 

    Canviar el plotter per a que tingui shades -> Faltara mirar com fer que els colors representin el temps + tunejar el spacing
    Fer el track en forma de principito + Escalonarlo -> Sa de fer amb un script 
    Revisar el model 
    