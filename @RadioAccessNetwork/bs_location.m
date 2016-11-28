%% BS and router location
function bs_location(this, model) 
if strcmpi(model,'SD-RAN')
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
this.wire_link =[...
    1 47 1;
    2 3  3;
    3 4  2;
    4 48 1;
    5 47 1;
    6 9  2;
    6 47 1;
    7 10 2;
    7 48 1;
    8 48 1;
    8 49 1;
    9 10 3;
    11 12 3;
    12 14 2;
    13 16 2;
    13 49 1;
    14 50 1;
    15 17 2;
    17 51 1;
    18 19 2;
    18 20 3;
    19 50 1;
    20 21 2;
    21 51 1;
    22 26 2;
    23 54 1;
    24 27 2;
    25 29 2;
    25 52 1;
    26 52 1;
    27 54 1;
    28 31 2;
    29 32 2;
    30 33 2;
    30 55 1;
    31 53 1;
    32 55 1;
    33 53 1;
    34 35 2;
    35 37 2;
    35 54 1;
    36 39 2;
    36 55 1;
    37 40 2;
    38 41 2;
    39 56 1;
    40 57 1;
    41 57 1;
    42 43 2;
    42 44 2;
    42 57 1;
    45 46 2;
    45 56 1;
    47 58 0;
    48 63 0;
    49 65 0;
    50 69 0;
    51 60 0;
    52 66 0;
    53 59 0;
    54 61 0;
    55 64 0;
    56 64 0;
    57 67 0;
    58 59 0;
    58 60 0;
    58 63 0;
    58 68 0;
    59 62 0;
    59 64 0;
    60 61 0;
    61 66 0;
    61 69 0;
    62 64 0;
    62 65 0;
    63 64 0;
    63 66 0;
    64 67 0;
    65 67 0;
    65 68 0;
    66 69 0;
    68 69 0];
elseif strcmpi(model,'SingleBS')
    BSs = [];
    GatewayBS = [0 0];
    router_nodes = [0 1000];
    this.wire_link = [1 2];
    this.coverage = 200;
elseif strcmpi(model,'DoubleBS')
    BSs = [];
    GatewayBS = [-100 0;
                  100 0];
    router_nodes = [0 1000];
    this.wire_link = [1 2;
            1 3;
            2 3];
elseif strcmpi(model, 'DoubleBSDoubleRouter')
    BSs = [];
    GatewayBS = [-100 0;
                  100 0];
    router_nodes = [-50 1000; 50 1000];
    this.wire_link = [1 2;
            1 3;
            1 4;
            2 3;
            2 4;
            3 4];
elseif strcmpi(model, 'grid9')
    BSs = [-200 200;
            200 200;
            0   0;
            -200 -200;
            200 -200;];
    GatewayBS = [0 200
                 -200 0;
                  200 0;
                  0 -200];
    router_nodes = [...
        -50 1500;
        -100 1300;
        100 1300;
        50 1500;
        ];
    this.wire_link = [1 6;
            1 7;
            2 6;
            2 8;
            3 6;
            3 7;
            3 8;
            3 9;
            4 7;
            4 9;
            5 8;
            5 9;
            6 10;
            7 11;
            8 13;
            9 12;
            10 11;
            10 13;
            11 12;
            12 13];
else
    error('error: invalid parameter.');
end
this.index.NBS = (1:size(BSs,1))';
this.index.GBS = (1:size(GatewayBS,1))'+size(BSs,1);
this.index.Router = (1:size(router_nodes,1))' + this.NumberBS;
this.wire_node = [[BSs; GatewayBS; router_nodes] ...
    [this.index.NBS; this.index.GBS; this.index.Router]];

BS_MAX =size(BSs,1);
b_core_link = (this.wire_link(:,1)>BS_MAX) & (this.wire_link(:,2)>BS_MAX); 
this.wire_link(b_core_link,4) = 1000;
this.wire_link(b_core_link,3) = 0;
b_hop_1_link = (this.wire_link(:,1)<=BS_MAX) & (this.wire_link(:,2)>BS_MAX); 
this.wire_link(b_hop_1_link,4) = 100;
this.wire_link(b_hop_1_link,3) = 1;
hop_1_node_id = unique(this.wire_link(b_hop_1_link,1));
residual_link_id = find(this.wire_link(:,4)==0);
residual_link_flag = zeros(size(residual_link_id));
for i = 1:length(residual_link_id)
    if ~isempty(find((hop_1_node_id==this.wire_link(residual_link_id(i),1)) |...
            (hop_1_node_id==this.wire_link(residual_link_id(i),2)),1))
        % link(i,4) = randi([10,50]);
        residual_link_flag(i) = 1;
    end
end
hop_2_link_id = residual_link_id(residual_link_flag~=0);
this.wire_link(hop_2_link_id,4) = randi([10,50],length(hop_2_link_id),1);
this.wire_link(hop_2_link_id,3) = 2;

residual_link_id = find(this.wire_link(:,4)==0);
visited_node_id = unique([this.wire_link(hop_2_link_id,1);this.wire_link(hop_2_link_id,2)]);
residual_link_flag = zeros(size(residual_link_id));
for i = 1:length(residual_link_id)
    if ~isempty(find((visited_node_id==this.wire_link(residual_link_id(i),1)) |...
            (visited_node_id==this.wire_link(residual_link_id(i),2)),1))
        % link(i,4) = randi([10,50]);
        residual_link_flag(i) = 1;
    end
end
hop_3_link_id = residual_link_id(residual_link_flag~=0);
this.wire_link(hop_3_link_id,4) = randi([2,5],length(hop_3_link_id),1);
this.wire_link(hop_3_link_id,3) = 3;
this.wire_link(this.wire_link(:,4)==0,3) = 4;
end