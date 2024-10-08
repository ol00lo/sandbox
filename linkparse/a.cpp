#include <iostream>
#include <boost/asio.hpp>
#include <regex>
#include <string>

std::vector<std::string> get_links(const std::string& html) {
	std::vector<std::string> links; 
	std::regex link_regex("<a href=\"/url\\?q=(https://[^\"]+)");
	auto begin = std::sregex_iterator(html.begin(), html.end(), link_regex);
	auto end = std::sregex_iterator();
	for (auto i = begin; i != end && links.size() < 3; ++i) {
		links.push_back((*i)[1].str());
	}
	return links;
}

int main() {
	std::string site = "www.google.com";
	std::string req = "/search?q=Dachshund";
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
	auto links = get_links(s);
	for (const auto& link : links) {
		std::cout << link << std::endl<<std::endl;
	}
}