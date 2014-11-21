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
        values(1) = str2double(fields{1});
        values(2) = str2double(fields{2});
        values(3) = str2double(fields{3});
        position = [values(1) values(2)];
        orientation = values(3)*((2*pi)/127); % in radians
        
        % Plot position and orientation
        plot(position(1),position(2),'*');
        hold on
        quiver(position(1),position(2),-25*sin(orientation),25*cos(orientation));
        axis equal;
        axis([-115 115 -60 60]);
        
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
