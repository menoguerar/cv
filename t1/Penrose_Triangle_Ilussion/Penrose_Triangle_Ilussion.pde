color bg, sc;
float e = 110;
float w = 60;
color c1 = color(13, 213 , 203);
color c2 = color( 254 , 84 , 84);
color c3 = color( 252 ,241, 196);
void setup(){
  size(500, 500);
  strokeWeight(1);
  bg = color(255, 255, 255);
  sc = color(0, 0, 0);
}


void draw(){
  background(bg);
  stroke(sc);
  translate(width / 2, height * 3 / 5);
  fill(c1);
  drawSide();
  rotate(TWO_PI / 3);
  fill(c2);
  drawSide();
  rotate(TWO_PI / 3);
  fill(c3);
  drawSide();
}

void drawSide(){
  float sin30, sin60, cos30, cos60;
  sin30 = cos60 = sin(PI / 6);
  cos30 = sin60 = cos(PI / 6);
  beginShape();
  float x1 = -e * cos60;
  float y1 = (e * cos60) / sqrt(3);
  vertex(x1, y1);
  float x2 = x1 - w;
  float y2 = y1;
  vertex(x2, y2);
  float x3 = x2 + (e + 3.0 * w) * cos60;
  float y3 = y2 - (e + 3.0 * w) * sin60;
  vertex(x3, y3);
  float x4 = x3 + (e + 4.0 * w) * sin30;
  float y4 = y3 + (e + 4.0 * w) * cos30;
  vertex(x4, y4);
  float x5 = x4 - w * cos60;
  float y5 = y4 + w * sin60;
  vertex(x5, y5);
  float x6 = x5 - (e + 3 * w) * cos60;
  float y6 = y5 - (e + 3 * w) * sin60;
  vertex(x6, y6);
  endShape(CLOSE);
}
