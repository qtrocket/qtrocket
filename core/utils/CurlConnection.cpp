
/// \cond
// C headers
// C++ headers
// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/CurlConnection.h"
#include "utils/Logger.h"


namespace
{
size_t curlCallback(void* content, size_t size, size_t nmemb, std::string* buffer)
{
    buffer->append(static_cast<char*>(content), size*nmemb);
    return size*nmemb;
}
}

namespace utils
{

CurlConnection::CurlConnection()
{
    // Both global init and cleanup are reference-counted, so per-instance
    // pairing is safe even with multiple CurlConnections.
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
}

CurlConnection::~CurlConnection()
{
    curl_easy_cleanup(curl);
    curl_global_cleanup();
}

std::string CurlConnection::get(const std::string& url)
{
    if(!curl)
    {
        Logger::getInstance()->error("curl_easy_init failed; cannot perform GET " + url);
        return std::string();
    }

    std::string str_result;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &str_result);
    curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
    // These calls run synchronously on the caller's thread (including the GUI
    // thread), so a hung connection must time out rather than wedge the app.
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    CURLcode res = curl_easy_perform(curl);
    if(res != CURLE_OK)
    {
        Logger::getInstance()->error(std::string("HTTP GET failed: ") +
                                               curl_easy_strerror(res) + " (" + url + ")");
        return std::string();
    }

    long httpCode = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
    if(httpCode < 200 || httpCode >= 300)
    {
        Logger::getInstance()->error("HTTP GET returned status " +
                                               std::to_string(httpCode) + " (" + url + ")");
        return std::string();
    }

    return str_result;
}

} // namespace utils
