

#ifndef SERVER_H
#define SERVER_H

#include <Arduino.h>
#include <WiFi.h>
#if defined(ESP32)
#include <WebServer.h>


class LocalServer
{
public:

    String mode_;
    WebServer server;
    LocalServer();
    IPAddress local_ip;
    IPAddress gateway;
    IPAddress subnet;
    
private:
    const char* ssid;
    const char* password;    
};

String send_html(String mode_);


#endif
#endif

