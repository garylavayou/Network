function Run(this, objective, max_rate)
%RUN Summary of this function goes here
%   Detailed explanation goes here
% TODO function Run(this, method, objective, max_rate)
%   optm = @method;
switch objective
    case {Objective.MAX_CONCURRENT,Objective.MAX_MIN, Objective.MAX_THROUGHPUT}
        lp_multi_flow_path(this, objective);  
        plot(this.fobj.rate);
        lim = axis;
        axis([0, this.NumberActiveUser+1, lim(3)*0.8, lim(4)*1.2]);
        xlabel('node id');
        ylabel('rate(Mbps)');
        grid on;
    case {Objective.PROPORTION_FAIRNESS}
        lp_multi_flow_path(this, objective);
    case {Objective.MAX_THROUGHPUT_LIMIT}
        lp_multi_flow_path(this, Objective.MAX_THROUGHPUT);
        r1 = this.fobj.rate;
        if nargin <= 2
            lp_multi_flow_path(this, objective);
        else
            lp_multi_flow_path(this, objective, max_rate);
        end
        r2 = this.fobj.rate;
        plot(1:this.NumberActiveUser, r1,'--', 1:this.NumberActiveUser, r2);
        lim = axis;
        axis([0, this.NumberActiveUser+1, lim(3)*0.8, lim(4)*1.2]);
        xlabel('node id');
        ylabel('rate(Mbps)');
        legend({'max throughput', 'max limited throughput'});
        grid on;
    case {Objective.MAX_THROUGHPUT_LIMIT_T}
        lp_multi_flow_path(this, Objective.MAX_THROUGHPUT);
        r1 = this.fobj.rate;
        if nargin <= 2
            lp_multi_flow_path(this, objective);
        else
            lp_multi_flow_path(this, objective, max_rate);
        end
        r2 = this.fobj.rate0;
        r3 = this.fobj.rate;
        plot(1:this.NumberActiveUser, r1,'--', 1:this.NumberActiveUser, r2,'-.',...
            1:this.NumberActiveUser, r3);
        lim = axis;
        axis([0, this.NumberActiveUser+1, lim(3)*0.8, lim(4)*1.2]);
        xlabel('node id');
        ylabel('rate(Mbps)');
        legend({'max throughput', 'max limited throughput', 'max limited throughput+'});
        grid on;        
    case {Objective.MAX_MIN_T,Objective.MAX_MIN_WEIGHT_T,Objective.PROPORTION_FAIRNESS_T}
        obj1 = bitand(objective, 15);
        lp_multi_flow_path(this, obj1);
        r1 = this.fobj.rate;
        lp_multi_flow_path(this, objective);
        r2 =this.fobj.rate;
        plot(1:this.NumberActiveUser, r1,'--', 1:this.NumberActiveUser, r2);
        lim = axis;
        axis([0, this.NumberActiveUser+1, lim(3)*0.8, lim(4)*1.2]);
        xlabel('node id');
        ylabel('rate(Mbps)');
        legend({'max-min user rate', 'max-min & throughput'});
        grid on;
    otherwise
        optm_multi_flow(graph, FlowTable, OBJ);
end

end

