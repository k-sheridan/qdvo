clear all
ih = IndexHandler();

ih.addLandmark(1, 1);
ih.addLandmark(1, 2);
ih.addLandmark(1, 3);
ih.addLandmark(1, 4);
ih.addLandmark(1, 5);
ih.addImustate(1);
ih.addImustate(2);
ih.addImustate(3);

A = round(rand(ih.dimensions()) * 20);
x = round(rand(ih.dimensions(), 1) * 20);
diff = A*x;

indicesOld = ih.getImustateIndices(2);

[Anew, xnew] = ih.moveVariableToTop('2->0', A, x);

indicesNew = ih.getImustateIndices(2);

diffnew = Anew*xnew;
