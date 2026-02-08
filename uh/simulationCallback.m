function [model, states, reports, solver, ok] = simulationCallback(model, states, reports, solver, schedule, simtime)
    PV = sum(model.rock.poro .* model.G.cells.volumes);
    last_n = findLastNonEmpty(states);
    Q = 0;
    for i = 1:last_n
        Q = Q + states{i}.wellSol(1).qWs .* schedule.step.val(i);
    end

    if Q > 3*PV
        ok = false;
    else
        ok = true;
    end
end