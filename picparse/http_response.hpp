#ifndef HTTP_RESPONSE
#define HTTP_RESPONSE
#include <boost/asio.hpp>
#include <string>
class Http_response
{
public:
    Http_response(const std::string& host, const std::string& path);
    std::string get_header();
    std::string get_contents();
    std::string image_extension();
    size_t content_length();

private:
    std::string _header;
    std::string _content;
};

#endif