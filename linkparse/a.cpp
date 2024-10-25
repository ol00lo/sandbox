﻿#include <iostream>
#include <boost/asio.hpp>
#include <regex>
#include <unordered_map>

void decodeHTML(std::string& html)
{
    std::unordered_map<std::string, std::string> entities = {{"&lt;", "<"},   {"&gt;", ">"},   {"&amp;", "&"},
                                                             {"&quot", "\""}, {"&apos;", "'"}, {"&nbsp;", " "},
                                                             {"&tilde;", "~"}};
    for (const auto& entity : entities)
    {
        size_t pos = 0;
        while ((pos = html.find(entity.first, pos)) != std::string::npos)
        {
            html.replace(pos, entity.first.length(), entity.second);
            pos += entity.second.length();
        }
    }
}

std::string decodeurl(const std::string& link)
{
    std::string ret;
    ret.reserve(link.length());

    for (int i = 0; i < link.length(); ++i)
    {
        if (link[i] != '%')
        {
            ret += link[i];
        }
        else
        {
            if (i + 2 < link.length())
            {
                int value =
                    (std::isdigit(link[i + 1]) ? link[i + 1] - '0' : std::tolower(link[i + 1]) - 'a' + 10) * 16 +
                    (std::isdigit(link[i + 2]) ? link[i + 2] - '0' : std::tolower(link[i + 2]) - 'a' + 10);
                ret += static_cast<char>(value);
                i += 2;
            }
        }
    }
    return ret;
}

std::vector<std::string> get_links(const std::string& html)
{
    std::vector<std::string> links;

    std::regex remove_class_bneawe("<span class=\"BNeawe\"><a href=\"/url\\?q=(https://[^\"]+)");
    std::string modified_html = std::regex_replace(html, remove_class_bneawe, "");

    std::regex link_regex("<a href=\"/url\\?q=(https://[^\"]+)");
    auto begin = std::sregex_iterator(html.begin(), html.end(), link_regex);
    auto end = std::sregex_iterator();

    for (auto i = begin; i != end && links.size() < 3; ++i)
    {
        std::string link = (*i)[1].str();
        size_t position = link.find("&sa");
        if (position != std::string::npos)
        {
            link = link.substr(0, position);
        }
        links.push_back(link);
    }
    return links;
}

int main(int argc, char* argv[])
{
    std::string site = "www.google.com";
    if (argc < 1)
    {
        throw std::runtime_error("missing input");
    }
    std::string q = argv[1];
    std::string req = "/search?q=" + q;

    for (int i = 2; i < argc; i++)
    {
        req += "+";
        req += argv[i];
    }

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
    decodeHTML(s);
    auto links = get_links(s);
    for (const auto& link : links)
    {
        std::cout << decodeurl(link) << std::endl << std::endl;
    }
}