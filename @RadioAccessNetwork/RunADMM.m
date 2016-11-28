function RunADMM(this, objective)
switch objective
    case {Objective.MAX_CONCURRENT,Objective.MAX_MIN, Objective.MAX_THROUGHPUT}
        admm_multi_flow(this, objective);
        plot(this.fobj.rate);
        lim = axis;
        axis([0, this.NumberActiveUser+1, lim(3)*0.8, lim(4)*1.2]);
        xlabel('node id');
        ylabel('rate(Mbps)');
        grid on;
end
end