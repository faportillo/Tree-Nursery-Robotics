function good = ValidPath(route, C)
  for i = 3:2:(length(route) - 1)
    a = route(i - 1);
    b = route(i);
    if abs(a - b) ~= C.N
      fprintf('BAD\n');
    end
  end
end