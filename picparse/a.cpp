#include "http_response.hpp"
#include <fstream>
#include <iostream>
#include <regex>

std::vector<std::string> get_links(const std::string& html)
{
    std::vector<std::string> links;
    std::regex link_regex("<img class=\"DS1iW\" alt=\"\" src=\"([^\"]*)\"/>");
    auto begin = std::sregex_iterator(html.begin(), html.end(), link_regex);
    auto end = std::sregex_iterator();
    for (auto i = begin; i != end; ++i)
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

void downloadImage(const std::string& imageUrl, std::string& filename)
{
    auto pos = imageUrl.find('/');
    if (pos == std::string::npos)
    {
        throw std::runtime_error("Incorrect url: " + imageUrl);
    }
    std::string host = imageUrl.substr(0, pos);
    std::string path = imageUrl.substr(pos);

    HttpResponse res = send_get_request(host, path);

    filename += "." + res.image_extension();
    std::ofstream file(filename, std::ios::binary);
    if (file)
    {
        file.write(res.get_contents().data(), res.content_length());
        file.close();
    }
    else
    {
        throw std::runtime_error("Create error: " + filename);
    }
}

std::string build_request(int argc, char* argv[])
{
    if (argc < 2)
    {
        throw std::runtime_error("missing input");
    }
    std::string q = argv[1];
    for (int i = 2; i < argc; i++)
    {
        q += "+";
        q += argv[i];
    }
    return "/search?q=" + q + "&tbm=isch";
}

void run(int argc, char* argv[])
{
    std::string site = "www.google.com";
    std::string req = build_request(argc, argv);
    HttpResponse res = send_get_request(site, req);
    auto links = get_links(res.get_contents());

    int i_pic = 0;
    int n_pic = 3;
    while (i_pic < n_pic)
    {
        try
        {
            std::string filename = "pic" + std::to_string(i_pic);
            downloadImage(links[i_pic], filename);
            std::cout << "Image saved to file: " << filename << std::endl;
            i_pic++;
        }
        catch (const std::exception& e)
        {
            std::cout << "Error downloading from " << links[i_pic] << " : " << e.what() << std::endl;
            i_pic++;
            n_pic++;
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