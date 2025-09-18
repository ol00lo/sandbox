import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "build", "python", "Debug"))

import py_graph_pr

class TestTensor(unittest.TestCase):
    def test_basic_ops(self):
        t = py_graph_pr.Tensor([1.0, 2.0, 3.0])

        self.assertEqual(list(t.shape), [1, 1, 1, 3])
        self.assertEqual(list(t.data), [1.0, 2.0, 3.0])

        t.scalar_mult(2.0)
        self.assertEqual(list(t.data), [2.0, 4.0, 6.0])


class TestNodes(unittest.TestCase):
    def test_data_node_set_get(self):

        x = py_graph_pr.DataNode("x")
        t = py_graph_pr.Tensor([1.0, 2.0])
        x.set_value(t)

        val = x.value()
        self.assertEqual(list(val.data), [1.0, 2.0])

    def test_factory_and_set_dep(self):
        x = py_graph_pr.DataNode("x")
        x.set_value(py_graph_pr.Tensor([3.0]))

        plus_node = py_graph_pr.INode.factory("PlusNode", "plus1")

        py_graph_pr.set_dep(plus_node, [x])

        result = plus_node.value()
        self.assertTrue(hasattr(result, "data"))
        self.assertIsInstance(result.data, list)



if __name__ == "__main__":
    unittest.main()
