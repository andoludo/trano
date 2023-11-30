from networkx import draw
import matplotlib.pyplot as plt


def test_network(network):
    space_1 = [node for node in list(network.graph.nodes) if node.name == "space_1"][0]
    space_2 = [node for node in list(network.graph.nodes) if node.name == "space_2"][0]
    network.merge_spaces(space_1, space_2)
    draw(network.graph, with_labels=True)
    plt.draw()
    plt.show()


def test_network_info(network):
    space = list(network.graph.nodes)[0]
    space.get_neighhors(network.graph)
    draw(network.graph, with_labels=True)
    plt.draw()
    plt.show()
