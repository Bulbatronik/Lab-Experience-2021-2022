function [Fr] = frame(scan,R_max)

% grid resolution
x_max = R_max + 1; 
x_min = - x_max;
y_max = x_max; 
y_min = x_min;

delta = 0.5;%10 cm

dim = ceil((2*R_max+2)/delta);

Fr = zeros(dim,dim);

[xf, yf] = meshgrid(x_min:delta:x_max, y_min:delta:y_max);

%Pf = [xf, yf];
Pf = [reshape(xf.',1,[]); reshape(yf.',1,[])]';

% sum((Pf - scan(p,:)).^2,"all")
for p = 1:length(scan) 
    d = sum((Pf - scan(p,:)).^2, 2);%pdist2(Pf, scan(p,:));
    [~, ind_min] = min(d);
    [r, c] = ind2sub(size(Fr), ind_min);
    Fr(r, c) = Fr(r, c) + 1;
end
%disp(size(Fr))
 %figure;
 %image(Fr,'CDataMapping','scaled')
 %imagesc(Fr)
 %axis('equal');
end

