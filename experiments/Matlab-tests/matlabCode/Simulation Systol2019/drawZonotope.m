function drawZonotope(P, H, Color)


nP=size(H,2);

Au1=eye(nP);
bu1=ones(nP,1);

Au=[Au1;-Au1];
bu=[bu1;bu1];

HB=Polyhedron('A',Au,'b',bu,'Ae',[],'be',[]);

computeVRep(HB);
V_HB=HB.V';

Vt=repmat(P,1,size(V_HB,2)) + H*V_HB;

V=Vt';


Z=Polyhedron('V',V);
Z.plot('color',Color)




end

