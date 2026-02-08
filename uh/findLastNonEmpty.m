function n = findLastNonEmpty(states)
    n = 1;
    while ~isempty(states{n:n})
        n = n +1;
        if n > length(states)
            break
        end
    end
    n = n -1;
end