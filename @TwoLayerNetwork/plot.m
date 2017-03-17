function plot(this)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
figure('Name', 'Topology of Core and Backhaul Networks');
h = plot(this.node(this.index.NBS,1), this.node(this.index.NBS,2),'k^', ...
    this.node(this.index.GBS,1), this.node(this.index.GBS,2),'ks',...
    this.node(this.index.Router,1), this.node(this.index.Router,2),'kd');
h(1).MarkerFaceColor='black';
h(2).MarkerFaceColor='black';
h(3).MarkerFaceColor='black';
if isempty(this.index.NBS)
    legend({'Gateway BS', 'Router'}, 'Location', 'northwest');
else
    legend({'Normal BS', 'Gateway BS', 'Router'}, 'Location', 'northwest');
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
    plot(px, py,'k-');
end
grid on;
% title('Topology of Core and Backhaul Networks');
xlabel('Distance (meters)');
ylabel('Distance (meters)');


end

