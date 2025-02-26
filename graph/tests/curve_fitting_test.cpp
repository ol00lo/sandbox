#include "data_node.hpp"
#include "generator.hpp"
#include "i_node.hpp"
#include "model.hpp"
#include "optimizer.hpp"
#include <catch2/catch.hpp>

using namespace g;

TEST_CASE("simmple test 1", "[st1]")
{
    //==================================================
    // L*y + K = out
    //==================================================
    // g::set_log_debug();

    std::shared_ptr<DataNode> y = std::make_shared<DataNode>("y");
    std::shared_ptr<DataNode> L = std::make_shared<DataNode>("L");
    std::shared_ptr<DataNode> K = std::make_shared<DataNode>("K");
    L->set_value(Tensor({5}));
    K->set_value(Tensor({5}));
    INode::PNode b1 = INode::factory("MultNode", "b1");
    g::set_dep(b1, {L, y});
    INode::PNode out = INode::factory("PlusNode", "out");
    g::set_dep(out, {b1, K});
    Model model({y}, {out});

    OptimizationWorker wrk(model, "MSE");
    wrk.set_optimizer(std::make_shared<SGDOptimizer>(Tensor({1e-2})));

    std::vector<std::vector<Tensor>> inp;
    std::vector<std::vector<Tensor>> outp;
    for (int i = 1; i < 8; ++i)
    {
        inp.push_back({Tensor({double(i)})});
		outp.push_back({Tensor({double(2 * i + 2)})});
    }
    SimpleDataGenerator train_data(inp, outp);
    inp.clear();
	outp.clear();
    for (int i = 8; i < 10; ++i)
    {
        inp.push_back({Tensor({double(i)})});
		outp.push_back({Tensor({double(2 * i + 2)})});
    }
    SimpleDataGenerator val_data(inp, outp);

    for (int iepoch = 0; iepoch < 10; ++iepoch)
    {
        train_data.next_epoch(false);
        Tensor sum_loss({0});
        int n_steps = 0;
        while (!train_data.is_epoch_end())
        {
            auto inp = train_data.next_input();
            auto gt = train_data.next_gt();
            Tensor train_loss = wrk.train(inp, gt);
            sum_loss.add(train_loss);
            n_steps += 1;
        }
        sum_loss.div(Tensor({double(n_steps)}));
        wrk.commit();
        std::cout << "Train loss= " << sum_loss << std::endl;
        
        // validate step
        val_data.next_epoch(false);
        Tensor sum_loss_val({0});
		n_steps = 0;
        while (!val_data.is_epoch_end())
        {
            auto inp = val_data.next_input();
            auto gt = val_data.next_gt();
            Tensor val_loss = wrk.validate(inp, gt);
            sum_loss_val.add(val_loss);
            n_steps += 1;
        }
		sum_loss_val.div(Tensor({double(n_steps)}));

        std::cout << "Val loss= " << sum_loss_val << std::endl;

    }
}
//
//TEST_CASE("cf", "[cf_t]")
//{
//    std::shared_ptr<DataNode> x = std::make_shared<DataNode>("x");
//    std::shared_ptr<DataNode> gt = std::make_shared<DataNode>("gt");
//    std::shared_ptr<DataNode> A = std::make_shared<DataNode>("A");
//    std::shared_ptr<DataNode> B = std::make_shared<DataNode>("B");
//    std::shared_ptr<DataNode> C = std::make_shared<DataNode>("C");
//    std::shared_ptr<DataNode> D = std::make_shared<DataNode>("D");
//    std::shared_ptr<DataNode> E = std::make_shared<DataNode>("E");
//    x->set_value(Tensor({1}));
//    gt->set_value(Tensor({6}));
//    A->set_value(Tensor({0.2}));
//    B->set_value(Tensor({0.2}));
//    C->set_value(Tensor({0.2}));
//    D->set_value(Tensor({0.2}));
//    E->set_value(Tensor({0.2}));
//    INode::PNode b1 = INode::factory("MultNode", "b1");
//    g::set_dep(b1, {B, x});
//    INode::PNode b2 = INode::factory("PlusNode", "b2");
//    g::set_dep(b2, {b1, C});
//    INode::PNode b3 = INode::factory("MultNode", "b3");
//    g::set_dep(b3, {A, b2});
//    INode::PNode b4 = INode::factory("MultNode", "b4");
//    g::set_dep(b4, {D, x});
//    INode::PNode b5 = INode::factory("PlusNode", "b5");
//    g::set_dep(b5, {b4, b3});
//    INode::PNode b6 = INode::factory("PlusNode", "b6");
//    g::set_dep(b6, {b5, E});
//
//    Model model({x}, {b6});
//
//    //========================================================
//    INode::PNode inpf = INode::factory("DataNode", "inpf");
//    INode::PNode loss = INode::factory("SqrNode", "loss");
//
//    Model loss_function({inpf, gt}, {loss});
//
//    OptimizationWorker wrk(model, loss_function);
//
//    wrk.set_optimizer(std::make_shared<SGDOptimizer>(Tensor({1e-2})));
//
//    // SimpleDataGenerator train_data(inp, outp);
//    // SimpleDataGenerator val_data(inp, outp);
//
//    // for (int iepoch = 0; iepoch < 10; ++iepoch)
//    //{
//    //     // train-step
//    //     train_data.next_epoch(true);
//    //     double sum_loss = 0;
//    //     int n_steps = 0;
//    //     while (!train_data.is_epoch_end())
//    //     {
//    //         double train_loss = wrk.train(train_data.next_input(), train_data.next_gt());
//    //         sum_loss += train_loss;
//    //         n_steps += 1;
//    //     }
//    //     std::cout << "Train loss=" << sum_loss / n_steps << std::endl;
//
//    //    // validate step
//    //    val_data.next_epoch(false);
//    //    // ....
//    //    std::cout << "Val loss=" << sum_loss / n_steps << std::endl;
//
//    //    wrk.commit();
//    //}
//}
