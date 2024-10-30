#include "http_response.hpp"

Http_response::Http_response(const std::string& host, const std::string& path)
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

    std::string http_response = std::string((std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>());
    size_t pos = http_response.find("\r\n\r\n");
    _header = http_response.substr(0, pos);
    _content = http_response.substr(pos + 4);

    size_t space1 = _header.find(' ') + 1;
    size_t space2 = _header.find(' ', space1);
    int status_code = std::stoi(_header.substr(space1, space2 - space1));
    if (status_code != 200)
    {
        throw std::runtime_error("Failed with status: " + std::to_string(status_code));
    }
}

std::string Http_response::get_header()
{
    return _header;
}
std::string Http_response::get_contents()
{
    return _content;
}

std::string Http_response::image_extension()
{
    std::string content_type_header = "Content-Type: ";
    size_t pos = _header.find(content_type_header);
    if (pos == std::string::npos)
    {
        throw std::runtime_error("Content-type not found");
    }
    pos += content_type_header.length();

    size_t end_pos = _header.find_first_of("\r\n", pos);
    std::string content_type = _header.substr(pos, end_pos - pos);

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

size_t Http_response::content_length()
{
    const std::string content_length_header = "Content-Length: ";
    size_t header_start = _header.find(content_length_header);

    header_start += content_length_header.length();

    size_t header_end = _header.find("\r\n", header_start);
    std::string content_length_str = _header.substr(header_start, header_end - header_start);

    return std::stoul(content_length_str);
}