#ifndef UTILS_CURLCONNECTION_H
#define UTILS_CURLCONNECTION_H

#include <vector>
#include <string>
#include <curl/curl.h>

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
