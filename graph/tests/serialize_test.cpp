#include "catch2/catch.hpp"
#include "data_node.hpp"
#include "i_node.hpp"
#include "model.hpp"
#include "tensor.hpp"
#include <iostream>
#include <nlohmann/json.hpp>

namespace
{
class A
{
public:
    int a;
    double b;
};
class B : public A
{
public:
    char c;
};
} // namespace
namespace nlohmann
{
template <typename T>
struct adl_serializer<T, std::enable_if_t<std::is_base_of<A, T>::value>>
{
    static void to_json(json& j, const A& a)
    {
        j = json{{"a", a.a}, {"b", a.b}};
    }

    static void from_json(const json& j, A& a)
    {
        j.at("a").get_to(a.a);
        j.at("b").get_to(a.b);
    }
};
} // namespace nlohmann

TEST_CASE("a test", "[a_test]")
{
    nlohmann::json js;
    A a{5, 1.12};
    B b{5, 1.12, 'a'};
    js["B"] = b;
    js["A"] = a;
    CHECK(js["A"]["a"] == 5);
    CHECK(js["A"]["b"] == a.b);
    CHECK(js["B"]["a"] == js["A"]["a"]);
    CHECK(js["B"]["b"] == js["A"]["b"]);
};

TEST_CASE("tensor serialize", "[tensor_serialize]")
{
    g::Tensor t({2, 3}, {1, 2, 3, 4, 5, 6});

    nlohmann::json js = t;
    g::Tensor b(js.get<g::Tensor>());

    CHECK(js["shape"].get<std::string>() == "(1, 1, 2, 3)");
    CHECK(b == t);
}

TEST_CASE("node serialize", "[node_serialize]")
{
    std::shared_ptr<g::DataNode> x = std::make_shared<g::DataNode>("xx");
    std::shared_ptr<g::DataNode> A = std::make_shared<g::DataNode>("AA");
    x->set_value(g::Tensor({1}));
    A->set_value(g::Tensor({1.2}));
    std::shared_ptr<g::INode> a1 = g::INode::factory("MultNode", "1");
    g::set_dep(a1, {A, x});

    nlohmann::json js = a1;
    nlohmann::json js1 = x;
    CHECK(js["classname"] == "MultNode");
    CHECK(js1["classname"] == "DataNode");
}

TEST_CASE("model serialize", "[model_serialize]")
{
    std::shared_ptr<g::DataNode> x = std::make_shared<g::DataNode>("xxxx");
    std::shared_ptr<g::DataNode> A = std::make_shared<g::DataNode>("AAAA");
    x->set_value(g::Tensor({1}));
    A->set_value(g::Tensor({1.2}));
    std::shared_ptr<g::INode> a1 = g::INode::factory("MultNode", "aaaa");
    g::set_dep(a1, {A, x});
    g::Model model({x}, {a1});
    nlohmann::json js = model;
    CHECK(js["nodes"][0]["classname"] == "DataNode");
    auto model2 = js.get<g::Model>();
    CHECK(model2.compute({g::Tensor({2})}) == model.compute({g::Tensor({2})}));
}
