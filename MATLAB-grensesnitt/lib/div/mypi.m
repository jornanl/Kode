function p = mypi( n )
o = 0;
for k = 1:n
    x = rand;
    y = rand;
    l = sqrt(rand^2 + rand^2);
    if l <= 1
        o = o + 1/n;
    end
    p = 4*o;
end