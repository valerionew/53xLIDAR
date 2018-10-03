 
 import processing.serial.*;
 
 int bgcolor = 0xff;     // Background color
 int fgcolor = 0;  // Fill color
 Serial myPort;       // The serial port
 
 float points[] = new float[16];
 
 class Point2D{
	 float x, y;
	 Point2D( float x, float y ){
		 this.x = x;
		 this.y = y; 
	 }
 }
 
 class Point2DColored extends Point2D{
	 int r;
	 int g;
	 int b;
	 Point2DColored(float x, float y, int r, int g, int b){
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
	 
	 // I know that the first port in the serial list on my mac
	 // is always my  FTDI adaptor, so I open Serial.list()[0].
	 // On Windows machines, this generally opens COM1.
	 // Open whatever port is the one you're using.
	 
	 //String portName = Serial.list()[0];
	 String portName = "/dev/ttyUSB0";
	 myPort = new Serial(this, portName, 115200);
 }
 
 void draw() {
	 println("draw!");
	 background(bgcolor);
	 fill(fgcolor);
	 
	 // Set the starting position of the point
	 int xpos = width/2;
	 int ypos = height/2;
	 
	 fill(255, 0, 0);
	 textSize(32);
	 text("1 circle = 0.1 meter", 10, ypos); 
	 
	 int xOffset = 0, yOffset = 0;
	 if (width < height){
		 xpos = width/2;
		 ypos = width/2;
		 yOffset = (height - width)/2; 
	 }else{
		 xpos = height/2;
		 ypos = height/2;
		 xOffset = (width - height)/2; 
	 }
	 
	 float divider = 6000;
	 
	 /* DRAW ONE CICLE EVERY METER */
	 noFill();
	 stroke(0,0,0);
	 for (int i=0; i <= divider; i+=1000){
		 ellipse(xpos+xOffset, ypos+yOffset, i/divider*xpos*2, i/divider*ypos*2);
	 }
	 
	 stroke(220,220,220);
	 for (float i=0; i < 180; i+=(180.0/points.length*2)){
		 float x = xpos*cos(-radians(i)) + xpos + xOffset;
		 float y = ypos*sin(-radians(i)) + ypos + yOffset;
		 float x1 = -xpos*cos(-radians(i)) + xpos + xOffset;
		 float y1 = -ypos*sin(-radians(i)) + ypos + yOffset;
		 line(x1, y1, x, y);
	 }
	 
	 // Draw the points
	 for (int i=0; i < points.length; i++){
		 float distance;
		 if ( Float.isNaN(points[i]) ){
			 distance = Math.max(xpos,ypos)*2;
			 distance=0;
			 stroke(213, 166, 166);
			 fill(213, 166, 166);
		 }else{
			 distance = points[i] / divider * xpos;
			 stroke(0,0,128);
			 fill(0, 0, 128);
		 }
		 float angle = (float)(360.0 / points.length) * i;
		 float fill = (float)(360.0 / points.length)/2;
		 
		 arc(xpos+xOffset, ypos+yOffset, distance*2, distance*2, radians(-angle-fill), radians(-angle+fill));
		 float x = distance * cos( -radians(angle) );
		 float y = distance * sin( -radians(angle) );
		 stroke(0,128,0);
		 fill(0, 255, 0);
		 ellipse(x+xpos+xOffset, y+ypos+yOffset, Math.max(points[i] / divider * 10, 4), Math.max(points[i] / divider * 10, 4));
		 //ellipse(x+xpos+xOffset, y+ypos+yOffset, 4,4);
	 }
 }
 
 void serialEvent(Serial myPort) {
	 try{
		 lol();
	 }catch(Exception e){
		 e.printStackTrace();
	 }
 }
 
 boolean valid = false;
 int total = 0;
 char message[] = new char[10];
 void lol(){
	 // read a byte from the serial port:
	 int inByte = myPort.read();
	 if (inByte == -1){
		 return;
	 }
	 if ('\n' == inByte){
		 
		 if (!valid){
			 println("valid!");
			 valid = true;
			 total = 0;
			 return;
		 }
		 if (total < 4){
			 println("invalid len! "+total);
			 total = 0;
			 return;
		 }
		 if (message[1] != '\t' || message[2] == '-'){
			 println("invalid - "+message[1]+" - " +message[2] + new String(message));
			 total = 0;
			 return;
		 }
		 
		 int sensor;
		 if (message[0] >= 'a'){
			 sensor = message[0] - 'a' + 10;
		 }else{
			 sensor = message[0] - '0';
		 }
		 
		 int val = 0;
		 for (int i = 2; i < total; i++){
			 val *= 10;
			 val += message[i] - '0';
		 }
		 println("value "+sensor+" "+val);
		 points[sensor] = val;
		 total = 0;
		 return;
	 }
	 
	 if (!valid)
		 return;
	 
	 message[total] = (char)inByte;
	 total += 1;
	 if (total > 9){
		 println("invalid too long " + new String(message));
		 total = 0;
		 valid = false;
	 }
 }
