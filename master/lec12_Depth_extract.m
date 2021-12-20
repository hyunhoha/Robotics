function depth=lec12_Depth_extract(boxes,depth_image)

sz=size(boxes);
depth=[];
for i=1:sz(1)
    xs=boxes(i,1);
    xe=xs+boxes(i,3)-2;
    ys=boxes(i,2);
    ye=ys+boxes(i,4)-2;
    temp_depth = depth_image(ys:ye,xs:xe);
    depth=[depth ; mean(temp_depth,'all','omitnan')];
end


end


