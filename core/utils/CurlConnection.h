#ifndef UTILS_CURLCONNECTION_H
#define UTILS_CURLCONNECTION_H

/// \cond
// C headers
#include <curl/curl.h>

// C++ headers
#include <string>

// 3rd party headers
/// \endcond

namespace utils {

class CurlConnection
{
public:
    CurlConnection();
    ~CurlConnection();

    // Owns the CURL easy handle, so copying would double-free it.
    CurlConnection(const CurlConnection&) = delete;
    CurlConnection& operator=(const CurlConnection&) = delete;

    /**
     * @brief Perform an HTTP GET and return the response body.
     * @return The body on success; an empty string on any failure
     *         (transport error, timeout, or non-2xx HTTP status).
     */
    std::string get(const std::string& url);

private:
    CURL* curl;
};

} // namespace utils

#endif // UTILS_CURLCONNECTION_H
