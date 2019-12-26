r = 0.013 + 0.02921

x0 = -0.70779
y0 = -0.027605

//teta = 0:15:90
teta = 90:15:180
//teta = 180:15:270
//teta = 270:15:360

tetarad = teta * %pi / 180;

x = r * cos(tetarad) + x0
y = r * sin(tetarad) + y0

disp([x' y'])
