#ifndef HTTP_RESPONSE
#define HTTP_RESPONSE
#include <map>
#include <string>

class HttpResponse
{
public:
    HttpResponse(const std::string& http_response);
    const std::map<std::string, std::string>& get_header() const;
    const std::string& get_contents() const;
    std::string image_extension() const;
    size_t content_length() const;

private:
    std::map<std::string, std::string> _header_map;
    std::string _content;
    void make_header_map(const std::string& header);
};

HttpResponse send_get_request(const std::string& host, const std::string& path);

#endif