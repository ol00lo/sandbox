#ifndef HTTP_RESPONSE
#define HTTP_RESPONSE
#include <boost/asio.hpp>
#include <map>
#include <string>

class HttpResponse
{
public:
    HttpResponse(const std::string& http_response);
    std::map<std::string, std::string> get_header();
    std::string get_contents();
    std::string image_extension();
    size_t content_length();

private:
    std::map<std::string, std::string> _header_map;
    std::string _content;
    void make_header_map(std::string header);
};

HttpResponse send_get_request(const std::string& host, const std::string& path);

#endif