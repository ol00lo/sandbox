function result = run_calculation(G, rock, fluid, schedule, state)
% Запускает расчёт сценария и возвращает результаты

    model = TwoPhaseOilWaterModel(G, rock, fluid);

    [states, reports] = simulateScheduleAD(state, model, schedule,  'AfterStepFn', @simulationCallback);
    time_vec = cumsum(schedule.step.val) / day();

    states = states(1:findLastNonEmpty(states));
    reports = reports(1:findLastNonEmpty(reports));
    time_vec = time_vec(1:length(states));
    nstep = numel(states);

    qI  = zeros(nstep,1);
    qPo = zeros(nstep,1);
    qP  = zeros(nstep,1);
    wcP = zeros(nstep,1);

    for it = 1:nstep
        ws = states{it};

        WI = strcmp({ws.name}, 'WI');
        WP = strcmp({ws.name}, 'WP');

        qI(it)  = sum([ws(WI).qWs]);
        qPo(it) = sum([ws(WP).qOs]);
        qPw     = sum([ws(WP).qWs]);
        qP(it)  = sum([ws(WP).qWs] + [ws(WP).qOs]);

        wcP(it) = qPw / qP(it);
    end

    QI  = cumtrapz(time_vec*day(), qI);
    QPo = cumtrapz(time_vec*day(), qPo);

    result.states   = states;
    result.reports  = reports;
    result.time     = time_vec;
    result.QI       = QI;
    result.QPo      = QPo;
    result.wcP      = wcP;
    result.qI       = qI;
    result.qP       = qP;
    result.qPo      = qPo;
end
