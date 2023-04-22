#ifndef UTILS_CURLCONNECTION_H
#define UTILS_CURLCONNECTION_H

/// \cond
// C headers
#include <curl/curl.h>

// C++ headers
#include <string>
#include <vector>

// 3rd party headers
/// \endcond

namespace utils {

class CurlConnection
{
public:
   CurlConnection();

   std::string get(const std::string& url, const std::vector<std::string>& extraHttpHeaders);


private:
   CURL* curl;
   CURLcode res;

};

} // namespace utils

#endif // UTILS_CURLCONNECTION_H
