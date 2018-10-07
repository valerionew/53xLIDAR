
import processing.serial.*;

int bgcolor = 0xff;     // Background color
int fgcolor = 0;  // Fill color
Serial myPort;       // The serial port

float points[] = new float[16];
int pointsColor[] = new color[16];

class Point2D {
  float x, y;
  Point2D( float x, float y ) {
    this.x = x;
    this.y = y;
  }
}

class Point2DColored extends Point2D {
  int r;
  int g;
  int b;
  Point2DColored(float x, float y, int r, int g, int b) {
    super(x, y);
    this.r = r;
    this.g = g;
    this.b = b;
  }
}


void setup() {
  //fullScreen(P2D);

  size(1000, 1000);  // Stage size

  frameRate(30);

  surface.setResizable(true);
  noStroke();      // No border on the next thing drawn

  // Print a list of the serial ports, for debugging purposes:
  printArray(Serial.list());

  //String portName = Serial.list()[0];
  String portName = "/dev/ttyUSB0";
  myPort = new Serial(this, portName, 115200);

  //                       thread("loop");
}

void loop() {
  String[] lines = loadStrings("logTest.txt");
  for (int i = 200; i < lines.length; i++) {
    for (char c : lines[i].toCharArray()) {
      analize((byte)c);
    }
    analize((byte)'\n');
    delay(20);
  }
  exit();
}

void draw() {
  background(bgcolor);
  fill(fgcolor);
  println("draw");

  // Set the starting position of the point
  int xpos = width/2;
  int ypos = height/2;
 

  int xOffset = 0, yOffset = 0;
  if (width < height) {
    xpos = width/2;
    ypos = width/2;
    yOffset = (height - width)/2;
  } else {
    xpos = height/2;
    ypos = height/2;
    xOffset = (width - height)/2;
  }

  float divider = 0.1;
  
  
  textSize(32);
  fill(255, 0, 0);
  text("1 circle = 1 meter", 80, ypos);
  fill(0, 0, 0);
  text("1 circle = 0.1 meter", 80, ypos - 32);

  /* DRAW ONE CICLE EVERY METER */
  noFill();
  stroke(0, 0, 0);
  for (int i=0; i < 10; i++) {
    float distance = sqrt(100*i) / divider;
    ellipse(xpos+xOffset, ypos+yOffset, distance*2, distance*2);
  }
  
  for (int i=0; i <= 3; i++) {
    float distance = sqrt(1000*i) / divider;
    stroke(255, 0, 0);
    ellipse(xpos+xOffset, ypos+yOffset, distance*2, distance*2);
  }

  /* Draw the lines*/
  stroke(220, 220, 220);
  float angleLines = 0;
  for (int i=0; i < points.length; i++) {
    float x = xpos*cos(-radians(angleLines)) + xpos + xOffset;
    float y = ypos*sin(-radians(angleLines)) + ypos + yOffset;
    float x1 = -xpos*cos(-radians(angleLines)) + xpos + xOffset;
    float y1 = -ypos*sin(-radians(angleLines)) + ypos + yOffset;
    line(x1, y1, x, y);
    if (i == 0)
      x -= 20;
    if (i == 4)
      y += 30;
      
    text((i + 1)+"", x, y);
    angleLines += (360.0/points.length);
  }

  // Draw the points
  for (int i=0; i < points.length; i++) {
    float distance;
    if ( Float.isNaN(points[i]) || pointsColor[i] == 0 ) {
      distance = Math.max(xpos, ypos)*2;
      distance=0;
      stroke(213, 166, 166);
      fill(213, 166, 166);
    } else {
      distance = sqrt(points[i]) / divider;//sqrt(points[i]);
      //distance = sqrt(points[i] / divider * xpos);
      println("distance: "+distance);
      stroke(0, 0, 128);
      pointsColor[i] -= 20;
      fill(0, 0, 255, pointsColor[i]);
      
    }
    float angle = (float)(360.0 / points.length) * i;
    float fill = (float)(360.0 / points.length)/2;

    arc(xpos+xOffset, ypos+yOffset, distance*2, distance*2, radians(-angle-fill), radians(-angle+fill));
    float x = distance * cos( -radians(angle) );
    float y = distance * sin( -radians(angle) );
    /*
    stroke(0, 128, 0);
    fill(0, 255, 0);
    ellipse(x+xpos+xOffset, y+ypos+yOffset, Math.max(points[i] / divider * 10, 4), Math.max(points[i] / divider * 10, 4));
    */
    //ellipse(x+xpos+xOffset, y+ypos+yOffset, 4,4);
  }
}

void serialEvent(Serial myPort) {
  try {
    int inByte = myPort.read();
    if (inByte == -1) {
      return;
    }
    analize((byte)inByte);
  }
  catch(Exception e) {
    e.printStackTrace();
  }
}

float valuesSum[] = new float[16];
int valuesNum[] = new int[16];

boolean valid = false;
int total = 0;
char message[] = new char[100];
void analize(byte inByte) {
  // read a byte from the serial port:

  if ('\n' == inByte) {

    if (!valid) {
      println("valid!");
      valid = true;
      total = 0;
      return;
    }
    if (total < 4) {
      println("invalid len! "+total);
      total = 0;
      return;
    }
    if (message[1] != '\t' || message[2] == '-') {
      println("invalid - "+message[1]+" - " +message[2] + new String(message));
      total = 0;
      return;
    }

    int sensor;
    if (message[0] >= 'a') {
      sensor = message[0] - 'a' + 10;
    } else {
      sensor = message[0] - '0';
    }

    int val = 0;
    for (int i = 2; i < total; i++) {
      val *= 10;
      val += message[i] - '0';
    }
    println("value "+sensor+" "+val);
    
    
    valuesSum[sensor] += val;
    valuesNum[sensor]++;
    
    points[sensor] = valuesSum[sensor] / valuesNum[sensor];
    pointsColor[sensor] = 255;
    
    if (valuesNum[sensor] > 2){
      valuesSum[sensor] -= points[sensor];
      valuesNum[sensor]--;
    }
    
    total = 0;
    return;
  }

  if (!valid)
    return;

  message[total] = (char)inByte;
  total += 1;
  if (total >= message.length) {
    println("invalid too long " + new String(message));
    total = 0;
    valid = false;
  }
}
