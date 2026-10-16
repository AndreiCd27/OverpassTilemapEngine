#pragma once

#include <iostream>
#include <string>
#include <curl/curl.h>

// Callback function to handle the data returned by the server
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t totalSize = size * nmemb;
    // Append the response characters to our std::string target
    ((std::string*)userp)->append((char*)contents, totalSize);
    return totalSize;
}

class CurlWrapper {
    CURL* curl = nullptr;
public:
    CurlWrapper();
    ~CurlWrapper();

    std::string PostUrl(const std::string& url, const std::string& jsonData);
};

