
/// \cond
// C headers
// C++ headers
#include <iostream>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/CurlConnection.h"


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
   curl_global_init(CURL_GLOBAL_ALL);
   curl = curl_easy_init();
}

std::string CurlConnection::get(const std::string& url,
                                const std::vector<std::string>& extraHttpHeaders)
{
   std::string str_result;
   std::string post_data = "";
   std::string action = "GET";
   if(curl)
   {
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &str_result);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false);
      curl_easy_setopt(curl, CURLOPT_ENCODING, "gzip");

      if(!extraHttpHeaders.empty())
      {
         struct curl_slist *chunk = NULL;
         for(size_t i = 0; i < extraHttpHeaders.size(); ++i)
         {
            chunk = curl_slist_append(chunk, extraHttpHeaders[i].c_str());
         }
         curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
      }

      if(post_data.size() > 0 || action == "POST" || action == "PUT" || action == "DELETE")
      {
         if(action == "PUT" || action == "DELETE")
         {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, action.c_str());
         }
         curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_data.c_str());
      }
      res = curl_easy_perform(curl);

      if(res != CURLE_OK)
      {
         std::cout << "There was an error: " << curl_easy_strerror(res) << "\n";
      }
   }

   return str_result;
}

} // namespace utils
