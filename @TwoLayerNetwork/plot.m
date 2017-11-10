function plot(this, color, gridon)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if nargin == 1 || isempty(color)
    color.node = {[0 0.5 0], [0.85 0.33 0.1], [0 0.45 0.74]};
    color.edge = {[0 0 0]};
end
if nargin <= 2
    gridon = false;
end
figure('Name', 'Topology of Core and Backhaul Networks');
h_node = plot(this.node(this.index.NBS,1), this.node(this.index.NBS,2),'^', ...
    this.node(this.index.GBS,1), this.node(this.index.GBS,2),'s',...
    this.node(this.index.Router,1), this.node(this.index.Router,2),'d');
for i = 1:length(h_node)
    h_node(i).Color = color.node{i};
    h_node(i).MarkerFaceColor=color.node{i};
    h_node(i).MarkerEdgeColor=color.node{i};
end

hold on;
for i = 1:this.IndirectedLinkNumber
    src = this.link(i,1);
    dest = this.link(i,2);
    px = this.node([src dest], 1);
    py = this.node([src dest], 2);
%     switch this.link(i,3)
%         case 0
%             plot(px, py, 'r-', 'LineWidth',1.5);
%         case 1
%             plot(px, py, 'b-', 'LineWidth',1.5);
%         case 2
%             plot(px, py, 'k-');
%         case 3
%             plot(px, py, 'g-');
%         otherwise
%             plot(px, py, 'c.-');
%     end
    plot(px, py,'-', 'Color', color.edge{1});
end
% Only sepecifies the node legend.
if isempty(this.index.NBS)
    legend({'Gateway BS', 'Router'}, 'Location', 'northwest');
else
    legend({'Normal BS', 'Gateway BS', 'Router'}, 'Location', 'northwest');
end
% Set the nodes at the top layer
Ax = h_node(1).Parent;
Ax.Children = [Ax.Children((end-2):end); Ax.Children(1:(end-3))];

if gridon
    grid on;
else
    grid off;
end
% title('Topology of Core and Backhaul Networks');
xlabel('Distance (meters)');
ylabel('Distance (meters)');
end
