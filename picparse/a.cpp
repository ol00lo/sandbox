#include <boost/asio.hpp>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>

std::string request(const std::string& host, const std::string& path)
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

    std::string http_response((std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>());
    size_t space1 = http_response.find(' ') + 1;
    size_t space2 = http_response.find(' ', space1);
    int status_code = std::stoi(http_response.substr(space1, space2 - space1));
    if (status_code != 200)
    {
        throw std::runtime_error("Failed with status: " + std::to_string(status_code));
    }

    return http_response;
}

std::vector<std::string> get_links(const std::string& html, int count)
{
    std::vector<std::string> links;
    std::regex link_regex("<img class=\"DS1iW\" alt=\"\" src=\"([^\"]*)\"/>");
    auto begin = std::sregex_iterator(html.begin(), html.end(), link_regex);
    auto end = std::sregex_iterator();
    for (auto i = begin; i != end && links.size() < count; ++i)
    {
        std::string url = (*i)[1].str();
        if (url.find("http://") == 0)
        {
            url.erase(0, 7);
        }

        links.push_back(url);
    }

    return links;
}

std::string image_extension(std::string http_response)
{
    std::string content_type_header = "Content-Type: ";
    size_t pos = http_response.find(content_type_header);

    pos += content_type_header.length();

    size_t end_pos = http_response.find_first_of("\r\n", pos);
    std::string content_type = http_response.substr(pos, end_pos - pos);

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

int content_length(const std::string& http_response)
{
    const std::string content_length_header = "Content-Length: ";
    size_t header_start = http_response.find(content_length_header);

    header_start += content_length_header.length();

    size_t header_end = http_response.find("\r\n", header_start);
    std::string content_length_str = http_response.substr(header_start, header_end - header_start);

    return std::stoul(content_length_str);
}

void downloadImage(const std::string& imageUrl, std::string& filename)
{
    auto pos = imageUrl.find('/');
    std::string host = imageUrl.substr(0, pos);
    std::string path = imageUrl.substr(pos);

    std::string http_response = request(host, path);
    filename += "." + image_extension(http_response);

    std::ofstream file(filename, std::ios::binary);
    if (file)
    {
        size_t pos = http_response.find("\r\n\r\n");
        if (pos != std::string::npos)
        {
            size_t length = content_length(http_response); 
            file.write(http_response.data() + pos + 4, length);
        }
        file.close();
    }
    else
    {
        std::cerr << "Create error: " << filename << std::endl;
    }
}

void run(int argc, char* argv[])
{
    if (argc < 2)
    {
        throw std::runtime_error("missing input");
    }
    std::string site = "www.google.com";
    std::string q = argv[1];
    for (int i = 2; i < argc; i++)
    {
        q += "+";
        q += argv[i];
    }
    std::string req = "/search?q=" + q + "&tbm=isch";
    std::string response = request(site, req);

    auto links = get_links(response, 3);
    for (size_t i = 0; i < links.size(); ++i)
    {
        std::string filename = "pic" + std::to_string(i);
        try
        {
            downloadImage(links[i], filename);
            std::cout << "Image saved to file: " << filename << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error downloading from " << links[i] << " : " << e.what() << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    try
    {
        run(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}