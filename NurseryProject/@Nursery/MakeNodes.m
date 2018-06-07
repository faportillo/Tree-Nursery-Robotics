function [nodes, DMAT] = MakeNodes(C)
  %implement defining the locations of each node and the cost of traveling
  %between them

X = 1;
Y = 2;
endI = C.endI;
swX = C.swX; swY = C.swY;
xs = ((C.W / 2):C.W:(C.W / 2 + C.W * (C.K - 1))) + swX;
nodes = [C.start(X), xs, xs, C.start(X);
  C.start(Y), (zeros(1, C.K) + swY), (zeros(1, C.K) + C.RL + swY), C.start(Y)];

fprintf('Start and End: (%0.2f, %0.2f) (%0.2f, %0.2f)\n', nodes(X, 1), nodes(Y, 1),...
  nodes(X, endI), nodes(Y, endI))
fprintf('Bottom Nodes: ')
for i = 2:(C.K + 1)
  fprintf('(%0.2f, %0.2f) ', nodes(X, i), nodes(Y, i))
end
fprintf('\nTop Nodes: ')
for i = (C.K + 2):(endI - 1)
  fprintf('(%0.2f, %0.2f) ', nodes(X, i), nodes(Y, i))
end
fprintf('\n')

DMAT = zeros(endI);

for i = 2:(C.K + 1)               %all lower nodes
  for j = (C.K + 2):(2 * C.K + 1) %visit upper nodes
    if (j - i) == C.K             %nodes are in same row
      DMAT(i, j) = -C.MULT;       %cost to go through row is 0
      DMAT(j, i) = -C.MULT;
    else                    %diagonal traversal not allowed
      DMAT(i, j) = C.HUGE;  %cost is unreasonably large
      DMAT(j, i) = C.HUGE;
    end
  end
end

%headlands turning costs
for i = 2:C.K
  for j = (i + 1):(C.K + 1)   
    d = abs(i - j);
    if d * C.W >= 2 * C.Rmin   %Pi turn
      DMAT(i, j) = C.MULT * (d * C.W + (pi - 2) * C.Rmin);
    else                       %Omega turn
      DMAT(i, j) = C.MULT * (3 * pi * C.Rmin - 2 * C.Rmin *...
        acos(1 - (2 * C.Rmin + d * C.W) ^ 2 / (8 * C.Rmin ^ 2)));
    end
    
    DMAT(j, i) = DMAT(i, j);
    DMAT(i + C.K, j + C.K) = DMAT(i, j);
    DMAT(j + C.K, i + C.K) = DMAT(i, j);
  end
end

%Start and end points
for j = 2:(endI - 1)
  DMAT(1, j) = abs(nodes(X, 1) - nodes(X, j)) + abs(nodes(Y, 1) - nodes(Y, j));
  DMAT(j, 1) = DMAT(1, j);
  DMAT(endI, j) = abs(nodes(X, endI) - nodes(X, j)) + ...
    abs(nodes(Y, endI) - nodes(Y, j));
  DMAT(j, endI) = DMAT(endI, j);
end

DMAT(1, endI) = C.HUGE;
DMAT(endI, 1) = C.HUGE;


end