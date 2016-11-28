function load_sample_graph(this, model)
if strcmp(model,'SD-RAN')
    BSs=[-550 250;
        -520 -100;
        -440 -90;
        -430 -200;
        -380 100;
        -360 200;
        -330 -110;
        -330 -300;
        -310 110;
        -300 -10;
        -280 300;
        -260 270;
        -250 -500;
        -230 285;
        -180 -250;
        -170 -450;
        -120 -260;
        -120 200;
        -100 300;
        -80 80;
        -60 -80;
        -20 600;
        -10 -400;
        40 -600;
        40 350;
        50 550;
        80 -620;
        85 -250;
        90 300;
        100 100;
        120 -80;
        155 310;
        160 10;
        170 -250;
        200 -300;
        250 280;
        260 -240;
        290 -20;
        320 350;
        340 -200;
        350 0;
        430 -150;
        480 -200;
        500 -20;
        500 300;
        530 260];
    GatewayBS = [...
        -410 150;
        -390 -200;
        -300 -350;
        -140 350;
        -80 -150;
        70 450;
        80 -10;
        160 -450;
        190 250;
        380 350;
        410 -100];
    router_nodes = [...
        -120 2250;
        20 2650;
        30 1450;
        130 1300;
        170 2850;
        180 2200;
        195 2550;
        230 2950;
        230 1900;
        260 2500;
        400 2900;
        430 1500];
    %% wired link specification
    this.link =[...
        13 59 1;
        14 15 3;
        15 16 2;
        16 60 1;
        17 59 1;
        18 21 2;
        18 59 1;
        19 22 2;
        19 60 1;
        20 60 1;
        20 61 1;
        21 22 3;
        23 24 3;
        24 26 2;
        25 28 2;
        25 61 1;
        26 62 1;
        27 29 2;
        29 63 1;
        30 31 2;
        30 32 3;
        31 62 1;
        32 33 2;
        33 63 1;
        34 38 2;
        35 66 1;
        36 39 2;
        37 41 2;
        37 64 1;
        38 64 1;
        39 66 1;
        40 43 2;
        41 44 2;
        42 45 2;
        42 67 1;
        43 65 1;
        44 67 1;
        45 65 1;
        46 47 2;
        47 49 2;
        47 66 1;
        48 51 2;
        48 67 1;
        49 52 2;
        50 53 2;
        51 68 1;
        52 69 1;
        53 69 1;
        54 55 2;
        54 56 2;
        54 69 1;
        57 58 2;
        57 68 1;
        59 1  0;
        60 6  0;
        61 8  0;
        62 12 0;
        63 3  0;
        64 9  0;
        65 2  0;
        66 4  0;
        67 7  0;
        68 7  0;
        69 10 0;
        1  2  0;
        1  3  0;
        1  6  0;
        1  11 0;
        2  5  0;
        2  7  0;
        3  4  0;
        4  9  0;
        4  12 0;
        5  7  0;
        5  8  0;
        6  7  0;
        6  9  0;
        7  10 0;
        8  10 0;
        8  11 0;
        9  12 0;
        11 12 0];
    this.index.Router = 1:size(router_nodes,1);
    this.index.NBS = (1:size(BSs,1)) + length(this.index.Router);
    this.index.GBS = length(this.index.Router) + length(this.index.NBS) + ...
        (1:size(GatewayBS,1));
    this.node = [[ router_nodes; BSs; GatewayBS ] ...
        [ this.index.Router'; this.index.NBS'; this.index.GBS' ]];
    
    CORE_MAX =size(router_nodes,1);
    NBS_MAX = this.index.NBS(end);
    b_core_link = this.link(:,2)<=CORE_MAX;
    this.link(b_core_link,4) = 1000;
    this.link(b_core_link,3) = 0;
    b_hop_1_link = (this.link(:,2)>NBS_MAX) & (this.link(:,1)>CORE_MAX);
    this.link(b_hop_1_link,4) = 100;
    this.link(b_hop_1_link,3) = 1;
    hop_1_node_id = unique(this.link(b_hop_1_link,1));
    residual_link_id = find(this.link(:,4)==0);
    residual_link_flag = zeros(size(residual_link_id));
    for i = 1:length(residual_link_id)
        if ~isempty(find((hop_1_node_id==this.link(residual_link_id(i),1)) |...
                (hop_1_node_id==this.link(residual_link_id(i),2)),1))
            % link(i,4) = randi([10,50]);
            residual_link_flag(i) = 1;
        end
    end
    hop_2_link_id = residual_link_id(residual_link_flag~=0);
    this.link(hop_2_link_id,4) = randi([10,50],length(hop_2_link_id),1);
    this.link(hop_2_link_id,3) = 2;
    
    residual_link_id = find(this.link(:,4)==0);
    visited_node_id = unique([this.link(hop_2_link_id,1);this.link(hop_2_link_id,2)]);
    residual_link_flag = zeros(size(residual_link_id));
    for i = 1:length(residual_link_id)
        if ~isempty(find((visited_node_id==this.link(residual_link_id(i),1)) |...
                (visited_node_id==this.link(residual_link_id(i),2)),1))
            % link(i,4) = randi([10,50]);
            residual_link_flag(i) = 1;
        end
    end
    hop_3_link_id = residual_link_id(residual_link_flag~=0);
    this.link(hop_3_link_id,4) = randi([2,5],length(hop_3_link_id),1);
%     this.link(hop_3_link_id,4) = 0;
    this.link(hop_3_link_id,3) = 3;
    this.link(this.link(:,4)==0,3) = 4;
end

this.graph = DualLayerGraph(this);
if ~Graph.VerifyConnectivity(this.graph.Adjacent)
    error('error: The graph is not all connected.');
end
end