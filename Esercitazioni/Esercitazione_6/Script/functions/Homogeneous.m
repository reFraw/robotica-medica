function T = Homogeneous(DH_row)

   a = DH_row(1);
   alfa = DH_row(2);
   d = DH_row(3);
   theta = DH_row(4);

   ca = cos(alfa);
   sa = sin(alfa);

   ct = cos(theta);
   st = sin(theta);

   firstColumn = [
       ct,
       st,
       0,
       0];

   secondColumn = [
       -st*ca,
       ct*ca,
       sa,
       0];

   thirdColumn = [
       st*sa,
       -ct*sa,
       ca,
       0];

   fourthColumn = [
       a*ct,
       a*st,
       d,
       1];

   T = [firstColumn secondColumn thirdColumn fourthColumn];

end