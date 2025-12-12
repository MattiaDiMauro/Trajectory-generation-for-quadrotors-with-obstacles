function create_box(res, pos_x, pos_y, pos_z,length, width, height, map)
%   Function that creates boxes, used as obstacles. It takes as inputs:
%   - The resolution of the map, res. 
%   - The x and y-position of the box in the world frame.
%   - The length, width and height of the box
%   - The map in which insert the box, map
%   - The length and width of the ground plane.

x_inter = pos_x - length/2+res/2:res:pos_x + length/2-res/2;
y_inter = pos_y - width/2+res/2:res:pos_y + width/2-res/2;
z_inter = pos_z - height/2+res/2:res:pos_z + height/2-res/2;



for x = x_inter
    for y = y_inter
        for z = z_inter
            xyz = [x y z];
            setOccupancy(map, xyz, 1)
        end
    end
end
end
