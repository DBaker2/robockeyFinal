%% Initialize device

m2 = serial('COM7', 'BAUDRATE', 9600);  % Fill in correct COM Port # here
fopen(m2)
m2.ReadAsyncMode = 'continuous'; % Continuously poll M2 for available data
n = 1


%% Send data request to M2
while 1
    read = m2.BytesAvailable;
    if read > 0
        data = fscanf(m2);
        fields = strsplit(data,'\t'); % fields(1) = x_pos, fields(2) = y_pos, fields(3) = orientation
        values(1) = str2num(fields{1});
        values(2) = str2num(fields{2});
        values(3) = str2num(fields{3});
        for i = 1:3
            if values(i)>127
                values(i) = values(i) - 256;
            end
        end
        position = [values(1) values(2)];
        orientation = values(3)*((2*pi)/127); % in radians
        
        % Plot position and orientation
        plot(position(1),position(2),'*');
        hold on
        quiver(position(1),position(2),-25*sin(orientation),25*cos(orientation));
        axis equal;
        axis([-150 150 -100 100]);
        
        % Draw rink
        plot([-97 97],[60 60],'k') % top line of rink
        plot([115 115],[42 -42],'k') % right line
        plot([-97 97],[-60 -60],'k') % bottom line
        plot([-115 -115],[-42 42],'k') % left line
        t1 = linspace(0,pi/2);
        t2 = linspace(pi/2,pi);
        t3 = linspace(pi,3/2*pi);
        t4 = linspace(3/2*pi,2*pi);
        plot((18*cos(t1)+97),(18*sin(t1)+42),'k'); % top right corner
        plot((18*cos(t2)-97),(18*sin(t2)+42),'k'); % top left
        plot((18*cos(t3)-97),(18*sin(t3)-42),'k'); % bottom left
        plot((18*cos(t4)+97),(18*sin(t4)-42),'k'); % bottom right
        
        % Display position and orientation values on plot
        strX = num2str(position(1));
        strY = num2str(position(2));
        strO = num2str(orientation);
        string = ['x = ' strX ', y = ' strY ', orientation = ' strO];
        text(0, 50, string, 'HorizontalAlignment', 'center');
        drawnow
        hold off
        
        n = n+1
    end
end
fclose(m2)
