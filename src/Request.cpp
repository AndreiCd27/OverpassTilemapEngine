
#include "Request.h"

CurlWrapper::CurlWrapper() {
	// Initialize an "easy" curl session handle
	this->curl = curl_easy_init();
}
CurlWrapper::~CurlWrapper() {
    // Clean up the easy handle
    curl_easy_cleanup(curl);
}

std::string CurlWrapper::PostUrl(const std::string& url, const std::string& jsonData) {
    if (this->curl) {
        std::string readBuffer;

        // Target URL
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        // Configure as POST
        curl_easy_setopt(curl, CURLOPT_POST, 1L);

        // JSON Body for the query
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonData.c_str());

        // Set Headers for json
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // Callback
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);

        // Execute the query
        CURLcode res = curl_easy_perform(curl);

        curl_slist_free_all(headers);

        // Check for any errors
        if (res != CURLE_OK) {
            readBuffer = "";
            std::cerr << "curl_easy_perform() failed: "
                << curl_easy_strerror(res) << std::endl;
        }
        else {
            //std::cout << "Response from localhost:\n" << readBuffer << std::endl;
        }
        return readBuffer;
    }
    return "";
}