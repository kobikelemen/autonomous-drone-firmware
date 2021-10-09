#ifndef SERVER_H
#define SERVER_H

#include <Arduino.h>
//#include <String>
#include <WiFi.h>
#if defined(ESP32)
#include <WebServer.h>


// #include <iostream>


class LocalServer
{
public:

    String mode_;
    WebServer server;
    LocalServer();
    IPAddress local_ip;
    IPAddress gateway;
    IPAddress subnet;
    
    
    // void run_server();
    
    //static int run_server(void * parameters);
private:
    const char* ssid;
    const char* password;    
};
String send_html(String mode_);
// server = WebServer(80);
// local_server = LocalServer(server);
// WebServer* server;
// LocalServer* local_server;

// void connect_cb();
// void hover_cb();
// void takeoff_cb();
// void land_cb();
// void not_found();

#endif
#endif