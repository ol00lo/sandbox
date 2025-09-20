import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "build", "python", "Debug"))

import py_graph_pr

class TestTensor(unittest.TestCase):
    def test_basic_ops(self):
        t = py_graph_pr.Tensor([1.0, 2.0, 3.0])

        self.assertEqual(list(t.shape), [1, 1, 1, 3])
        self.assertEqual(list(t.data), [1.0, 2.0, 3.0])

        t.scalar_mult(2.0)
        self.assertEqual(list(t.data), [2.0, 4.0, 6.0])

class SimpleTestNodes(unittest.TestCase):
    def test_data_node_set_get(self):
        x = py_graph_pr.DataNode("x")
        x.set_value(py_graph_pr.Tensor([1.0, 2.0]))

        val = x.value()
        self.assertEqual(list(val.data), [1.0, 2.0])

    def test_factory_and_set_dep(self):
        x = py_graph_pr.DataNode("x")
        x.set_value(py_graph_pr.Tensor([3.0]))
        y = py_graph_pr.DataNode("y")
        y.set_value(py_graph_pr.Tensor([3.0]))

        plus_node = py_graph_pr.INode.factory("PlusNode", "plus1")

        py_graph_pr.set_dep(plus_node, [x, y])

        result = plus_node.value()
        self.assertTrue(hasattr(result, "data"))
        self.assertIsInstance(result.data, list)
        self.assertEqual(len(result.data), 1)
        self.assertEqual(result.data[0], 6.0)

class TestArithmeticNodes(unittest.TestCase):
    def test_plus_node(self):
        a = py_graph_pr.DataNode("a")
        b = py_graph_pr.DataNode("b")
        a.set_value(py_graph_pr.Tensor([3.0]))
        b.set_value(py_graph_pr.Tensor([2.0]))
        node = py_graph_pr.INode.factory("PlusNode", "plus")
        py_graph_pr.set_dep(node, [a, b])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 + 2.0, places=6)

    def test_minus_node(self):
        a = py_graph_pr.DataNode("a")
        b = py_graph_pr.DataNode("b")
        a.set_value(py_graph_pr.Tensor([3.0]))
        b.set_value(py_graph_pr.Tensor([2.0]))
        node = py_graph_pr.INode.factory("MinusNode", "minus")
        py_graph_pr.set_dep(node, [a, b])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 - 2.0, places=6)

    def test_mult_node(self):
        a = py_graph_pr.DataNode("a")
        b = py_graph_pr.DataNode("b")
        a.set_value(py_graph_pr.Tensor([3.0]))
        b.set_value(py_graph_pr.Tensor([2.0]))
        node = py_graph_pr.INode.factory("MultNode", "mult")
        py_graph_pr.set_dep(node, [a, b])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 * 2.0, places=6)

    def test_square_node(self):
        a = py_graph_pr.DataNode("a")
        a.set_value(py_graph_pr.Tensor([3.0]))
        node = py_graph_pr.INode.factory("SqrNode", "square")
        py_graph_pr.set_dep(node, [a])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], 3.0 ** 2, places=6)

    def test_sin_node(self):
        a = py_graph_pr.DataNode("a")
        a.set_value(py_graph_pr.Tensor([3.0]))
        node = py_graph_pr.INode.factory("SinNode", "sin")
        py_graph_pr.set_dep(node, [a])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], math.sin(3.0), places=6)

    def test_cos_node(self):
        a = py_graph_pr.DataNode("a")
        a.set_value(py_graph_pr.Tensor([3.0]))
        node = py_graph_pr.INode.factory("CosNode", "cos")
        py_graph_pr.set_dep(node, [a])

        res = node.value()
        self.assertTrue(hasattr(res, "data"))
        self.assertAlmostEqual(res.data[0], math.cos(3.0), places=6)


if __name__ == "__main__":
    unittest.main()


if __name__ == "__main__":
    unittest.main()
