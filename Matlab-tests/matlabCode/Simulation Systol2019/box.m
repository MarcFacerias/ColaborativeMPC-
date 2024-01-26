% function B=box(R)

function B=box(R)
B=diag(sum(abs(R.')));
return

