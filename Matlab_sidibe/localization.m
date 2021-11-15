function q = move_new(p,u)

pCorrect = 0.8;
pOverShoot = 0.1;
pUnderShoot = 0.1;

q = pCorrect * circshift(p',[u,0]);
q = q + pOverShoot * circshift(p',[u+1,0]);
q = q + pUnderShoot * circshift(p',[u-1,0]);

q = q';





    

