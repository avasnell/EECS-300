%Shows temperatures in C,threshold, the # of blobs and the count
%each hot spot outlined by a cyan ocloured line

%Before running have to close platformio serial monitor(only one program can use the port) 
%and have to set the port on line7. to see your ports, type serialportlist("available") in command window
clear; close all;

PORT = "/dev/cu.usbserial-0001";
BAUD = 460800;%must match Serial.begin(460800) in thermal cpp

s= serialport(PORT,BAUD);%opens USB port so matlab can read from esp32
configureTerminator(s, "LF");%line of text ends at a new line

%make figure for live view
fig=figure('Name','Thermal Camera');
ax= axes(fig);
img= imagesc(ax,zeros(24, 32));%blank 24 row by 32 column picture
axis(ax,'image'); 
colormap(ax, hot);%sets color so cold to hot is black then red then yellow then white  
clim(ax,[20 36]);%20 degrees C is darkest color and 36C is the brightest 
colorbar(ax);
hold(ax,'on');%keep whatever already on picture so new things are drawn on top of the picture
xlabel(ax,'column (left = exit, right = enter)'); 
ylabel(ax,'row');

%numbers read from the text,unknown until the first frame arrives
count= NaN; 
nBlobs= NaN; 
nHot= NaN; 
baseline =NaN; 
thr=NaN;
outline=[];%the cyan hot outline

while isvalid(fig)%repeat until figure window is closed
    try
        msg = strtrim(readline(s));%read one line of text from the esp32
    catch
        continue%nothing arrived, then try again
    end

if startsWith(msg, "FRAME,")%checks if this is line with all the temps
    parts = split(msg, ",");%cuts the line at every comma into separate pieces
    v = str2double(parts(2:end));%skips the first piece which is the word "frame' and turns the rest into numbers
    if numel(v) == 768% only continues if it got all 768 numbers
        T = reshape(v, 32, 24)';% arranges them into 24 rows by 32 columns(in degrees C)
        set(img, 'CData', T);%puts them into the picture

        delete(outline); outline = [];%removes the old cyan hot spot outline
        if ~isnan(thr) && min(T(:)) < thr && max(T(:)) > thr
    %only draaws new cyan outline if all true: 
    % threshold is known(isnan means it is unknown),
    %&& min(T(:)),, and coldest pixel in figure is below threshold
    %T(:) turns each pixel into temp number in C
    %&& max(T(:)) > thr,,,and hottest pixel is above threshold
            [~, outline] = contour(ax, T, [thr thr], 'c','LineWidth',1.5);%draws new cyan outline
        end

            title(ax, sprintf('Count: %g   Blobs: %g   Hot pixels: %g   Baseline: %.2f   Threshold: %.2f', ...
                  count, nBlobs, nHot, baseline, thr));
            drawnow limitrate;%updates figyre
            if s.NumBytesAvailable > 30000, flush(s); end%if too far behind then skip ahead,,stops picture from lagging
        end
    elseif startsWith(msg, "Count:")
        count = str2double(extractAfter(msg, "Count:"));
    elseif startsWith(msg, "Number of blobs:")
        nBlobs = str2double(extractAfter(msg, "Number of blobs:"));
    elseif startsWith(msg, "Hot pixels count:")
        nHot = str2double(extractAfter(msg, "Hot pixels count:"));
    elseif startsWith(msg, "Baseline:")
        baseline = str2double(extractAfter(msg, "Baseline:"));
    elseif startsWith(msg, "Threshold:")
        thr = str2double(extractAfter(msg, "Threshold:"));
    end
end

clear s;%close the port so VS code can use port again to reprogram esp