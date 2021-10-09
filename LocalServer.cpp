
#if defined(ESP32)
#include <Arduino.h>
#include "LocalServer.h"

// #include <iostream>

// WebServer server(80);
// LocalServer local_server(server);

// WebServer server(80);
// LocalServer local_server(server);

LocalServer::LocalServer()
{
    
    ssid = "ESP";
    password = "12345678";
    //server = _server;
    WiFi.softAP(ssid, password);
    IPAddress local_ip(192,168,1,1);
    IPAddress gateway(192,168,1,1);
    IPAddress subnet(255,255,255,0);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    // _server.on("/", connect_cb);
    // _server.on("/hover", hover_cb);
    // _server.on("/takeoff", takeoff_cb);
    // _server.on("/land", land_cb);
    // _server.onNotFound(not_found);
    // _server.begin();

}


String send_html(String mode_){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>LED Control</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";


  /*
   if(mode_ == "land")
  {ptr +="<p>MODE: ON</p><a class=\"button button-off\" href=\"/mode_\">LAND</a>\n";}
  else if (mode_ == "takeoff)
  {ptr +="<p>MODE: OFF</p><a class=\"button button-on\" href=\"/led1on\">ON</a>\n";}

  if(led2stat)
  {ptr +="<p>LED2 Status: ON</p><a class=\"button button-off\" href=\"/led2off\">OFF</a>\n";}
  else
  {ptr +="<p>LED2 Status: OFF</p><a class=\"button button-on\" href=\"/led2on\">ON</a>\n";}
  */
  
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}


// void LocalServer::run_server()
// {
//   Serial.println("IN START OF RUN_SERVER");


//     //WebServer my_server = *((WebServer*)parameters);
//     while (true)
//     {
//       Serial.println("IN WHILE LOOP IN RUN_SERVER");
//         server.handleClient();
//         //vTaskDelay(200);
//     } 
// }




#endif
