#include "http_response.hpp"

HttpResponse send_get_request(const std::string& host, const std::string& path)
{
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::resolver resolver(io_service);
    boost::asio::ip::tcp::resolver::query query(host, "80");
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::ip::tcp::socket socket(io_service);
    boost::asio::connect(socket, endpoint_iterator);

    boost::asio::streambuf request;
    std::ostream request_stream(&request);
    request_stream << "GET " << path << " HTTP/1.0\r\n";
    request_stream << "Host: " << host << "\r\n";
    request_stream << "Accept: */*\r\n";
    request_stream << "Connection: close\r\n\r\n";
    boost::asio::write(socket, request);

    boost::asio::streambuf response;
    boost::system::error_code e;
    boost::asio::read(socket, response, boost::asio::transfer_all(), e);
    std::string http_response =
        std::string((std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>());
    return HttpResponse(http_response);
}

HttpResponse::HttpResponse(const std::string& http_response)
{
    size_t pos = http_response.find("\r\n\r\n");
    std::string header = http_response.substr(0, pos);
    _content = http_response.substr(pos + 4);

    size_t space1 = header.find(' ') + 1;
    size_t space2 = header.find(' ', space1);
    int status_code = std::stoi(header.substr(space1, space2 - space1));
    if (status_code != 200)
    {
        throw std::runtime_error("Failed with status: " + std::to_string(status_code));
    }
    make_header_map(header);
}

std::map<std::string, std::string> HttpResponse::get_header()
{
    return _header_map;
}

std::string HttpResponse::get_contents()
{
    return _content;
}

std::string HttpResponse::image_extension()
{
    std::string content_type = get_header()["Content-Type"];

    if (content_type.find("image/") == 0)
    {
        auto extension = content_type.substr(6);
        return extension;
    }
    else
    {
        throw std::runtime_error("not image: " + content_type);
    }
}

size_t HttpResponse::content_length()
{
    std::string content_length_str = get_header()["Content-Length"];
    return std::stoul(content_length_str);
}

void HttpResponse::make_header_map(std::string header)
{
    size_t start = 0;
    while (true)
    {
        size_t end = header.find('\n', start);
        if (end == std::string::npos)
        {
            break;
        }
        std::string line = header.substr(start, end - start);
        size_t colonpos = line.find(':');
        if (colonpos != std::string::npos)
        {
            std::string key = line.substr(0, colonpos);
            std::string value = line.substr(colonpos + 2);
            value.erase(value.size() - 1);
            _header_map[key] = value;
        }
        start = end + 1;
    }
}