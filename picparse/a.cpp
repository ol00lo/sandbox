#include <boost/asio.hpp>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>

std::vector<std::string> get_links(const std::string& html, int count)
{
    std::vector<std::string> links;
    std::regex link_regex("<img class=\"DS1iW\" alt=\"\" src=\"([^\"]*)\"/>");
    auto begin = std::sregex_iterator(html.begin(), html.end(), link_regex);
    auto end = std::sregex_iterator();
    for (auto i = begin; i != end && links.size() < count; ++i)
    {
        std::string url =(*i)[1].str();
        if (url.find("http://") == 0)
        {
            url.erase(0, 7);
        }

        links.push_back(url);
    }

    return links;
}

void downloadImage(const std::string& imageUrl, const std::string& filename)
{
    auto pos = imageUrl.find('/');
    std::string host = imageUrl.substr(0, pos);
    std::string path = imageUrl.substr(pos);

    boost::asio::io_service io_service;
    boost::asio::ip::tcp::resolver resolver(io_service);
    boost::asio::ip::tcp::resolver::query query(host, "80");
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::ip::tcp::socket socket(io_service);
    boost::asio::connect(socket, endpoint_iterator);
    boost::asio::streambuf request;
    std::ostream request_stream(&request);
    request_stream << "GET " + path + " HTTP/1.0\r\n";
    request_stream << "Host: " + host + "\r\n\r\n";
    boost::asio::write(socket, request);

    boost::asio::streambuf response;
    boost::system::error_code e;
    boost::asio::read(socket, response, boost::asio::transfer_all(), e);
    std::ofstream file(filename, std::ios::binary);
    if (file)
    {
        std::string http_response((std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>());
        size_t pos = http_response.find("\r\n\r\n");
        if (pos != std::string::npos)
        {
            file.write(http_response.data() + pos + 4, http_response.size() - pos - 4);
        }
        file.close();
    }
    else
    {
        std::cerr << "Create error: " << filename << std::endl;
    }
}

int main(int argc, char* argv[])
{
    try
    {
        setlocale(LC_ALL, "RUS");
        std::string site = "www.google.com";
        std::string q = argv[1];
        std::string req = "/search?q=" + q + "&tbm=isch";
        boost::asio::io_service io_service;

        boost::asio::ip::tcp::resolver resolver(io_service);
        boost::asio::ip::tcp::resolver::query query(site, "80");
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::ip::tcp::socket socket(io_service);
        boost::asio::connect(socket, endpoint_iterator);

        boost::asio::streambuf request;
        std::ostream request_stream(&request);
        request_stream << "GET " << req << " HTTP/1.0\r\n";
        request_stream << "Host: " << site << "\r\n";
        request_stream << "Accept: */*\r\n";
        request_stream << "Connection: close\r\n";
        request_stream << "\r\n";
        boost::asio::write(socket, request);

        boost::asio::streambuf response;
        boost::system::error_code e;
        boost::asio::read(socket, response, boost::asio::transfer_all(), e);
        std::string s((std::istreambuf_iterator<char>(&response)), std::istreambuf_iterator<char>());
        //std::cout << s << std::endl << std::endl;

        auto links = get_links(s, 3);
        for (size_t i = 0; i < links.size(); ++i)
        {
            std::string filename = "pic" + std::to_string(i) + ".jpg";
            downloadImage(links[i], filename);
            std::cout << "Image saved to file: " << filename << std::endl;
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}