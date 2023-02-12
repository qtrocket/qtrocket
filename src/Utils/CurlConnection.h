#ifndef _APPRESETAPI_H_
#define _APPRESETAPI_H_

#include <string>
#include <curl/curl.h>
#include <vector>

namespace Utils
{

class CurlConnection
{
public:
   CurlConnection();

   std::string get(const std::string& url, const std::vector<std::string>& extraHttpHeaders);


private:
   CURL* curl;
   CURLcode res;

};

void initCurl(const std::string& host);

} // namespace utils

#endif // _APPRESETAPI_H_