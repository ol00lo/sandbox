import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "build", "python", "Debug"))

import py_graph_pr as graph

class TestTensor(unittest.TestCase):
    def test_basic_ops(self):
        t = graph.Tensor([1.0, 2.0, 3.0])

        self.assertEqual(list(t.shape), [1, 1, 1, 3])
        self.assertEqual(list(t.data), [1.0, 2.0, 3.0])

        t.scalar_mult(2.0)
        self.assertEqual(list(t.data), [2.0, 4.0, 6.0])

class SimpleTestNodes(unittest.TestCase):
    def test_data_node_set_get(self):
        x = graph.DataNode("x")
        x.set_value(graph.Tensor([1.0, 2.0]))

        val = x.value()
        self.assertEqual(list(val.data), [1.0, 2.0])

    def test_factory_and_set_dep(self):
        x = graph.DataNode("x")
        x.set_value(graph.Tensor([3.0]))
        y = graph.DataNode("y")
        y.set_value(graph.Tensor([3.0]))

        plus_node = graph.INode.factory("PlusNode", "plus1")

        graph.set_dep(plus_node, [x, y])

        result = plus_node.value()
        self.assertTrue(hasattr(result, "data"))
        self.assertIsInstance(result.data, list)
        self.assertEqual(len(result.data), 1)
        self.assertEqual(result.data[0], 6.0)

class TestArithmeticNodes(unittest.TestCase):
    def test_plus_node(self):
        a = graph.DataNode("a")
        b = graph.DataNode("b")
        a.set_value(graph.Tensor([3.0]))
        b.set_value(graph.Tensor([2.0]))
        node = graph.INode.factory("PlusNode", "plus")
        graph.set_dep(node, [a, b])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 + 2.0, places=6)

    def test_minus_node(self):
        a = graph.DataNode("a")
        b = graph.DataNode("b")
        a.set_value(graph.Tensor([3.0]))
        b.set_value(graph.Tensor([2.0]))
        node = graph.INode.factory("MinusNode", "minus")
        graph.set_dep(node, [a, b])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 - 2.0, places=6)

    def test_mult_node(self):
        a = graph.DataNode("a")
        b = graph.DataNode("b")
        a.set_value(graph.Tensor([3.0]))
        b.set_value(graph.Tensor([2.0]))
        node = graph.INode.factory("MultNode", "mult")
        graph.set_dep(node, [a, b])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 * 2.0, places=6)

    def test_square_node(self):
        a = graph.DataNode("a")
        a.set_value(graph.Tensor([3.0]))
        node = graph.INode.factory("SqrNode", "square")
        graph.set_dep(node, [a])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 ** 2, places=6)

    def test_sin_node(self):
        a = graph.DataNode("a")
        a.set_value(graph.Tensor([3.0]))
        node = graph.INode.factory("SinNode", "sin")
        graph.set_dep(node, [a])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], math.sin(3.0), places=6)

    def test_cos_node(self):
        a = graph.DataNode("a")
        a.set_value(graph.Tensor([3.0]))
        node = graph.INode.factory("CosNode", "cos")
        graph.set_dep(node, [a])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], math.cos(3.0), places=6)

class TestSinModel(unittest.TestCase):
    def test_sin_model_training(self):
        a = 1.0
        b = 2.0
        c = 0.0
        d = 0.5
        e = 1.0

        def compute_out(x):
            return a * math.sin(b * x + c) + d * x + e

        x = graph.DataNode("x")
        A = graph.DataNode("A"); A.set_value(graph.Tensor([1.0]))
        B = graph.DataNode("B"); B.set_value(graph.Tensor([1.0]))
        C = graph.DataNode("C"); C.set_value(graph.Tensor([0.0]))
        D = graph.DataNode("D"); D.set_value(graph.Tensor([0.1]))
        E = graph.DataNode("E"); E.set_value(graph.Tensor([0.0]))

        a1 = graph.INode.factory("MultNode", "a1")
        graph.set_dep(a1, [B, x])
        a2 = graph.INode.factory("PlusNode", "a2")
        graph.set_dep(a2, [a1, C])
        a3 = graph.INode.factory("CosNode", "a3")
        graph.set_dep(a3, [a2])
        a4 = graph.INode.factory("MultNode", "a4")
        graph.set_dep(a4, [a3, A])
        a5 = graph.INode.factory("MultNode", "a5")
        graph.set_dep(a5, [D, x])
        a6 = graph.INode.factory("PlusNode", "a6")
        graph.set_dep(a6, [a4, a5])
        out = graph.INode.factory("PlusNode", "out")
        graph.set_dep(out, [a6, E])

        model = graph.Model([x], [out])
        worker = graph.OptimizationWorker(model, graph.LossType.MSE)
        worker.set_optimizer(graph.SGDOptimizer(0.01))

        inp, outp = [], []
        for j in range(10):
            for scale in [1.0, 0.5, 0.25, 0.75]:
                val = j * scale
                inp.append([graph.Tensor([val])])
                outp.append([graph.Tensor([compute_out(val)])])

        train_data = graph.SimpleDataGenerator(inp, outp)

        val_inputs, val_targets = [], []
        for j in range(10):
            val = j / 3.0
            val_inputs.append([graph.Tensor([val])])
            val_targets.append([graph.Tensor([compute_out(val)])])

        val_data = graph.SimpleDataGenerator(val_inputs, val_targets)

        for epoch in range(100):
            train_data.next_epoch(True)
            while not train_data.is_epoch_end():
                inp = train_data.next_input()
                gt  = train_data.next_gt()
                worker.train(inp, gt)
            worker.commit()

            val_data.next_epoch(False)
            total_loss, steps = 0.0, 0
            while not val_data.is_epoch_end():
                inp = val_data.next_input()
                gt  = val_data.next_gt()
                total_loss += worker.validate(inp, gt)
                steps += 1
            if total_loss / steps <= 0.001:
                break

        self.assertAlmostEqual(A.value().data[0], a, delta=0.02)
        self.assertAlmostEqual(B.value().data[0], b, delta=0.015)
        # self.assertAlmostEqual(C.value()[0], c, delta=0.1)  # при желании
        self.assertAlmostEqual(D.value().data[0], d, delta=0.01)
        self.assertAlmostEqual(E.value().data[0], e, delta=0.01)


if __name__ == "__main__":
    unittest.main()


if __name__ == "__main__":
    unittest.main()
