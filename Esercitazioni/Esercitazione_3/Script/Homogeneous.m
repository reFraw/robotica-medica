function T = Homogeneous(DH_row)

% This function calculates the homogeneous transformation matrix between
% two sequential triplets from the associated row of the
% Denavit-Hartenberg table.

% INPUT : 1x4 array representing the row of interest
%         in the Denavit-Hartenberg table in the form [a, alfa, d, theta]

% OUTPUT : 4x4 matrix representing the homogeneous transformation between
%          two sequential joints.

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