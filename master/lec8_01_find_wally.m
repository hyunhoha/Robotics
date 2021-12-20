close all; clear; clc
%%

crowd = double(imread('wheres-wally.png'))./255;
wally = double(imread('wally.png'))./255;

tic
    S=isimilarity(wally,crowd,@zncc);
toc

maximum = max(max(S));

[y,x]=find(S==maximum);
% (y,x) == Wally's position

figure(1);

imshow(crowd); hold on;
plot(x,y,'ro','MarkerSize',40,'LineWidth',2); hold off;
axis on;

function S = isimilarity(T, im, metric)

%TODO add all the other similarity metrics, including rank and census

    % if no metric input, use @zncc
    if nargin < 3
        metric = @zncc;
    end
    [nr,nc] = size(im);
    
    if rem(size(T,2), 2) == 0 || rem(size(T,1), 2) == 0
        error('isimilarity:badarg', 'template must have odd dimensions');
    end
    
    hc = floor( (size(T,2)-1)/2 );
    hr = floor( (size(T,1)-1)/2 );
    hr1 = hr+1;
    hc1 = hc+1;

    % Sliding window and calculate zncc and record it with its position
    S = NaN(size(im));
    for c=hc1:nc-hc
        for r=hr1:nr-hr
            S(r,c) = metric(T, im(r-hr:r+hr, c-hc:c+hc));
        end
    end
end

% current zncc
function m = zncc(w1, w2)

	w1 = w1 - mean(w1(:));
	w2 = w2 - mean(w2(:));
	denom = sqrt( sum(sum(w1.^2))*sum(sum(w2.^2)) );

	if denom < 1e-10
		m = 0;
	else
		m = sum(sum((w1.*w2))) / denom;
    end
end