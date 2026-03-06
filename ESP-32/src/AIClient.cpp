#include "AIClient.h"
#include <HTTPClient.h>

String askAI(const String& text)
{
    HTTPClient http;

    http.begin("http://192.168.1.100:5000/ai"); // 改成你的电脑IP

    http.addHeader("Content-Type","application/json");

    String body = "{\"text\":\""+text+"\"}";

    int code = http.POST(body);

    if(code > 0)
    {
        String payload = http.getString();
        http.end();
        return payload;
    }

    http.end();
    return "{\"reply\":\"AI request failed\"}";
}