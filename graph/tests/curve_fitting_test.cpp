#include "data_node.hpp"
#include "generator.hpp"
#include "i_node.hpp"
#include "model.hpp"
#include "optimization_worker.hpp"
#include <catch2/catch.hpp>

using namespace g;

TEST_CASE("generator test", "[gt]")
{
    std::vector<std::vector<Tensor>> inp;
    std::vector<std::vector<Tensor>> outp;
    for (int j = 0; j < 3; ++j)
    {
        int i = j;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({Tensor({double(2 * i + 2)})});
    }
    SimpleDataGenerator train_data(inp, outp);
    train_data.next_epoch(false);
    CHECK(train_data.next_input()[0][0] == 0);
    CHECK(train_data.next_gt()[0][0] == 2);
    CHECK(train_data.next_input()[0][0] == 1);
    CHECK(train_data.next_gt()[0][0] == 4);
    CHECK(train_data.is_epoch_end() == false);
    CHECK(train_data.next_input()[0][0] == 2);
    CHECK(train_data.next_gt()[0][0] == 6);
    CHECK(train_data.is_epoch_end() == true);

    train_data.next_epoch(true);
    CHECK(train_data.next_input()[0][0] != 0);
    CHECK(train_data.is_epoch_end() == false);
}

TEST_CASE("train test", "[tt]")
{
    std::shared_ptr<DataNode> y = std::make_shared<DataNode>("y");
    std::shared_ptr<DataNode> L = std::make_shared<DataNode>("L");
    std::shared_ptr<DataNode> K = std::make_shared<DataNode>("K");
    L->set_value(Tensor({5}));
    K->set_value(Tensor({5}));
    INode::ptr_t b1 = INode::factory("MultNode", "b1");
    g::set_dep(b1, {L, y});
    INode::ptr_t out = INode::factory("PlusNode", "out");
    g::set_dep(out, {b1, K});
    Model model({y}, {out});
    OptimizationWorker wrk(model, LossType::MSE);
    wrk.set_optimizer(std::make_shared<SGDOptimizer>(1e-2));

    double loss = wrk.train({Tensor({1})}, {Tensor({4})});
    CHECK(loss == 36);
    loss = wrk.train({Tensor({2})}, {Tensor({6})});
    CHECK(loss == Approx(74.65));
    CHECK(K->get_value() == Tensor({5}));
    wrk.commit();
    CHECK(K->get_value()[0] != 5);
}

namespace
{
const double PI = 3.1415926;
const double a = 1;
const double b = 2;
const double c = 0;
const double d = 0.5;
const double e = 1;
Tensor compute_out(const Tensor& inp)
{
    Tensor out = g::scalar_mult(b, inp);
    out.add(Tensor({c}));
    out.apply_oper([](double x) { return std::sin(x); });
    out.mult(Tensor({a}));
    Tensor Dx = g::mult(inp, Tensor({d}));
    out.add(Dx);
    out.add(Tensor({e}));
    return out;
}
} // namespace

TEST_CASE("sin test", "[st]")
{
    //==================================================
    // A*sin(B*x + C) + D*x + E = out
    //==================================================
    g::get_logger()->set_level(spdlog::level::off);

    std::shared_ptr<DataNode> x = std::make_shared<DataNode>("x");
    std::shared_ptr<DataNode> A = std::make_shared<DataNode>("A");
    std::shared_ptr<DataNode> B = std::make_shared<DataNode>("B");
    std::shared_ptr<DataNode> C = std::make_shared<DataNode>("C");
    std::shared_ptr<DataNode> D = std::make_shared<DataNode>("D");
    std::shared_ptr<DataNode> E = std::make_shared<DataNode>("E");
    A->set_value(Tensor({1.0}));
    B->set_value(Tensor({1.0}));
    C->set_value(Tensor({0.0}));
    D->set_value(Tensor({0.1}));
    E->set_value(Tensor({0.0}));
    INode::ptr_t a1 = INode::factory("MultNode", "");
    g::set_dep(a1, {B, x});
    INode::ptr_t a2 = INode::factory("PlusNode", "");
    g::set_dep(a2, {a1, C});
    INode::ptr_t a3 = INode::factory("CosNode", "");
    g::set_dep(a3, {a2});
    INode::ptr_t a4 = INode::factory("MultNode", "");
    g::set_dep(a4, {a3, A});
    INode::ptr_t a5 = INode::factory("MultNode", "");
    g::set_dep(a5, {D, x});
    INode::ptr_t a6 = INode::factory("PlusNode", "");
    g::set_dep(a6, {a4, a5});
    INode::ptr_t out = INode::factory("PlusNode", "");
    g::set_dep(out, {a6, E});

    Model model({x}, {out});

    OptimizationWorker wrk(model, LossType::MSE);
    wrk.set_optimizer(std::make_shared<SGDOptimizer>(1e-2));

    std::vector<std::vector<Tensor>> inp;
    std::vector<std::vector<Tensor>> outp;
    for (int j = 0; j < 10; ++j)
    {
        double i = j;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({compute_out(Tensor({double(i)}))});
        i /= 2;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({compute_out(Tensor({double(i)}))});
        i /= 2;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({compute_out(Tensor({double(i)}))});
        i *= 3;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({compute_out(Tensor({double(i)}))});
    }
    SimpleDataGenerator train_data(inp, outp);
    inp.clear();
    outp.clear();
    for (int j = 0; j < 10; ++j)
    {
        double i = double(j) / 3;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({compute_out(Tensor({double(i)}))});
    }
    SimpleDataGenerator val_data(inp, outp);

    for (int iepoch = 0; iepoch < 100; ++iepoch)
    {
        std::cout << "epoch = " << iepoch << std::endl;
        train_data.next_epoch(true);
        double sum_loss = 0;
        int n_steps = 0;
        while (!train_data.is_epoch_end())
        {
            auto inp = train_data.next_input();
            auto gt = train_data.next_gt();
            double train_loss = wrk.train(inp, gt);
            sum_loss += train_loss;
            n_steps += 1;
        }
        wrk.commit();
        std::cout << "Train loss= " << sum_loss / double(n_steps) << std::endl;

        // validate step
        val_data.next_epoch(false);
        double sum_loss_val = 0;
        n_steps = 0;
        while (!val_data.is_epoch_end())
        {
            auto inp = val_data.next_input();
            auto gt = val_data.next_gt();
            double val_loss = wrk.validate(inp, gt);
            sum_loss_val += val_loss;
            n_steps += 1;
        }
        sum_loss_val /= double(n_steps);
        std::cout << "Val loss= " << sum_loss_val << std::endl;
        if (sum_loss_val <= 0.001)
            break;
    }
    CHECK(A->get_value()[0] == Approx(a).epsilon(0.02));
    CHECK(B->get_value()[0] == Approx(b).epsilon(0.01));
    //CHECK(C->get_value()[0] == Approx(c).epsilon(0.1));
    CHECK(D->get_value()[0] == Approx(d).epsilon(0.01));
    CHECK(E->get_value()[0] == Approx(e).epsilon(0.01));
}

TEST_CASE("simmple test 1", "[st1]")
{
    //==================================================
    // L*y + K = out
    //==================================================
    g::get_logger()->set_level(spdlog::level::off);

    std::shared_ptr<DataNode> y = std::make_shared<DataNode>("");
    std::shared_ptr<DataNode> L = std::make_shared<DataNode>("");
    std::shared_ptr<DataNode> K = std::make_shared<DataNode>("");
    L->set_value(Tensor({5}));
    K->set_value(Tensor({5}));
    INode::ptr_t b1 = INode::factory("MultNode", "");
    g::set_dep(b1, {L, y});
    INode::ptr_t out = INode::factory("PlusNode", "");
    g::set_dep(out, {b1, K});
    Model model({y}, {out});

    OptimizationWorker wrk(model, LossType::MSE);
    wrk.set_optimizer(std::make_shared<SGDOptimizer>(1e-2));

    std::vector<std::vector<Tensor>> inp;
    std::vector<std::vector<Tensor>> outp;
    for (int j = 0; j < 10; ++j)
    {
        int i = j;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({Tensor({double(2 * i + 2)})});
        i /= 3;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({Tensor({double(2 * i + 2)})});
        i *= 2;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({Tensor({double(2 * i + 2)})});
    }
    SimpleDataGenerator train_data(inp, outp);
    inp.clear();
    outp.clear();
    for (int i = 10; i < 15; ++i)
    {
        inp.push_back({Tensor({double(i)})});
        outp.push_back({Tensor({double(2 * i + 2)})});
    }
    SimpleDataGenerator val_data(inp, outp);

    for (int iepoch = 0; iepoch < 50; ++iepoch)
    {
        train_data.next_epoch(false);
        double sum_loss = 0;
        int n_steps = 0;
        while (!train_data.is_epoch_end())
        {
            auto inp = train_data.next_input();
            auto gt = train_data.next_gt();
            double train_loss = wrk.train(inp, gt);
            sum_loss += train_loss;
            n_steps += 1;
        }
        wrk.commit();
        std::cout << "epoch = " << iepoch << ", Train loss= " << sum_loss / double(n_steps) << std::endl;

        // validate step
        val_data.next_epoch(false);
        double sum_loss_val = 0;
        n_steps = 0;
        while (!val_data.is_epoch_end())
        {
            auto inp = val_data.next_input();
            auto gt = val_data.next_gt();
            double val_loss = wrk.validate(inp, gt);
            sum_loss_val += val_loss;
            n_steps += 1;
        }
        sum_loss_val /= double(n_steps);
        std::cout << "epoch = " << iepoch << ", Val loss= " << sum_loss_val << std::endl;
        if (sum_loss_val <= 0.001)
            break;
    }
    CHECK(K->get_value()[0] == Approx(2).epsilon(0.03));
    CHECK(L->get_value()[0] == Approx(2).epsilon(0.01));
}