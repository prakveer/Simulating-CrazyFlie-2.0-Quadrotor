function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

%can comment out if needed
astar = true;
map.margin=0.32;

rows=size(map.occgrid,1);
cols=size(map.occgrid,2);
depth=size(map.occgrid,3);

p1=map.xyzToSub(start);
s=map.xyzToInd(start);

p2=map.xyzToSub(goal);
try
    g=map.xyzToInd(goal);
catch
    path=[];
    num_expanded =0;
    return
end
q=1:numel(map.occgrid); %unvisited nodes

if(map.occgrid(s)==1 || map.occgrid(g)==1)
    disp('Start or goal located in cell with obstacle')
    path = [];
    num_expanded=0;
    return;
end

cost=Inf([numel(map.occgrid),1]);
parent=zeros([numel(map.occgrid),1]);
cost(s)=0;

tree=zeros([numel(map.occgrid),3]);
tree(1,:)=p1;
treecost=Inf([numel(map.occgrid),1]);
treecost(s)=0;
[a,b]=(min(treecost));
current =tree(1,:) ;
ratio=map.res_xyz(3)/map.res_xyz(1);

while((isempty(find(q==g, 1))~=1 ) && (a<Inf))
    [valid,dist]=neighbourhood(ratio,current(1),current(2),current(3),rows,cols,depth,p2,astar);
    ind=map.xyzToInd(map.subToXYZ(valid));
    dist=dist+a;
    l=dist<cost(ind) & (map.occgrid(ind)==0);
    parent(ind(l))=b(1);
    cost(ind(l))=dist(l);
    tree(ind(l),:)=valid(l,:);
    treecost(ind(l),:)=dist((l));

    %reset the node
    treecost(b(1))=Inf;
    q(b(1))=0;  

    [a,b]=(min(treecost));
    current =tree(b(1),:) ;

end

num_expanded = sum(q==0);

if(a<Inf)
    table=g;
    %backtrack
    while(parent(table(end))~=0)
        table=[table,parent(table(end))];
    end
    table=flip(table);
    path=map.indToXYZ(table');
    path=[start;path;goal];
    %path = [];
else
    path=[];
end

pts=path;
res=0.1;
i=1;
while(i < size(pts,1)-2)
    j=size(pts,1); 
    while( j > i+1)
        length= floor(norm(pts(i,:)-pts(j,:))/res);
        v=(pts(j,:)-pts(i,:))/length;            
        summ=1;            
        if(length~=0)
            summ=0;
            for xx=1:length
            summ=summ+map.collide((pts(i,:)+xx*v));
            end
        end
         if(round(summ,3)==0) 
                pts(i+1:j-1,:)=[];
                break;
         else
            j=j-1;
         end


    end
        i=i+1;
end  
    
path=pts;

function [n,dist]= neighbourhood(ratio,i,j,k,rows,cols,depth,p2,astar)

    [Rr, Cr, Pr] = ndgrid(i-1:i+1, j-1:j+1, k-1:k+1);
    neighbours = [Rr(:), Cr(:), Pr(:)];
     
   neighbours(14,:) = [];     %remove center    
    
    valider= neighbours(:,1) >= 1 & neighbours(:,2) >= 1 & neighbours(:,3) >= 1 ...
    & neighbours(:,1) <= rows & neighbours(:,2) <= cols...
    &  neighbours(:,3) <= depth;
    n=neighbours(valider,:);
    nei=n;
    nei(:,3)=ratio.*n(:,3);
    d=(nei-[i,j,ratio*k]);
    dist=zeros([size(d,1),1]);
    if(astar==false)
       for i =1:size(dist,1)
        dist(i)=norm(d(i,:));
       end         
    else
        heu=(nei-p2);
        for i =1:size(dist,1)
        dist(i)=norm(d(i,:))+norm(heu(i,:));
       end     
    end

end
    
end

