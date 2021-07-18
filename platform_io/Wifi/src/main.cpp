/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

accel_x = String(accel_event.acceleration.x);
    accel_y  = String(accel_event.acceleration.y);
    accel_z = String(accel_event.acceleration.z);

        gyro_x = String(gDPS.x); gyro_y = String(gDPS.y); gyro_z = String(gDPS.z);

  fusion_pitch = String(orientation.pitch);
      fusion_roll  = String(orientation.roll);
      fusion_heading = String(orientation.heading);

filter_roll = String(angleF_roll); filter_pitch = String(angleF_pitch);

bat_perc_str = String(bat_perc);

// Load Wi-Fi library
#include <WiFi.h>

// Replace with your network credentials
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

String accel_value = "123.4";
String gyro_value = "9.81";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;

void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            // client.println("h0 { position:absolute; left:300px; top:50px;}");
            // client.println("h1 { position:absolute; left:100px; top:100px;}");
            // client.println("h2 { position:absolute; left:100px; top:300px;}");
            // client.println("h3 { position:absolute; left:100px; top:400px;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}");

            // CSS definition for the table: 
            client.println("table {width:100%;}");
            client.println("table, th, td {border: 1px solid black; border-collapse: collapse;}");
            client.println("th, td {padding: 15px; text-align: left;}");
            client.println("#t01 tr:nth-child(even) {background-color: #eee;}");
            client.println("#t01 tr:nth-child(odd) {background-color: #fff;}");
            client.println("#t01 th: {background-color: black; color: white;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>INERTIAL SYSTEM MANAGEMENT</h1>");

            // HTML table design
            client.println("<p><table id=\"t01\"><tr><th>ACCELEROMETER</th><th> X Axis: " + accel_value + "</th><th> Y Axis: " + accel_value + "</th><th> Z Axis: " + accel_value + "</th></tr>");
            client.println("<tr><th>GYROSCOPE</th><th> Pitch: " + gyro_value + "</th><th> Roll: " + gyro_value + "</th><th> Yaw: " + gyro_value + "</th></tr></table></p>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            // client.println("<p>TWISTING THE PLATFORM</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">CLOCKWISE ON</button></a>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">CLOCKWISE OFF</button></a>");
            } 
            
            // Display current state, and ON/OFF buttons for GPIO 27  
            //client.println("<h2>ANTICLOCKWISE TWIST " + output27State + "</h2>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<a href=\"/27/on\"><button class=\"button\">ANTICLOCKWISE ON</button></a></p>");
            } else {
              client.println("<a href=\"/27/off\"><button class=\"button button2\">ANTICLOCKWISE OFF</button></a></p>");
            }

            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}