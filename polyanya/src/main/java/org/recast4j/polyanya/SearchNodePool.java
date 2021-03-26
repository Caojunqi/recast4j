package org.recast4j.polyanya;

import org.recast4j.detour.Node;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Caojunqi
 * @date 2021-03-24 18:32
 */
public class SearchNodePool {

    private final Map<Long, List<SearchNode>> m_map = new HashMap<>();
    private final ArrayList<SearchNode> m_nodes = new ArrayList<>();

    public SearchNodePool() {

    }

    public void clear() {
        m_nodes.clear();
        m_map.clear();
    }

    List<SearchNode> findNodes(long id) {
        List<SearchNode> nodes = m_map.get(id);
        if (nodes == null) {
            nodes = new ArrayList<>();
        }
        return nodes;
    }

    SearchNode findNode(long id) {
        List<SearchNode> nodes = m_map.get(id);
        if (nodes != null && !nodes.isEmpty()) {
            return nodes.get(0);
        }
        return null;
    }

    SearchNode findNode(long id, int state) {
        List<SearchNode> nodes = m_map.get(id);
        if (nodes != null) {
            for (SearchNode node : nodes) {
                if (node.state == state) {
                    return node;
                }
            }
        }
        return null;
    }

    SearchNode getNode(long id, int state) {
        List<SearchNode> nodes = m_map.get(id);
        if (nodes != null) {
            for (SearchNode node : nodes) {
                if (node.state == state) {
                    return node;
                }
            }
        }
        return create(id, state);
    }

    protected SearchNode create(long id, int state) {
        SearchNode node = new SearchNode(m_nodes.size() + 1);
        node.id = id;
        node.state = state;
        m_nodes.add(node);
        List<SearchNode> nodes = m_map.get(id);
        if (nodes == null) {
            nodes = new ArrayList<>();
            m_map.put(id, nodes);
        }
        nodes.add(node);
        return node;
    }

    public int getNodeIdx(SearchNode node) {
        return node != null ? node.index : 0;
    }

    public SearchNode getNodeAtIdx(int idx) {
        return idx != 0 ? m_nodes.get(idx - 1) : null;
    }

    public int getNodeCount() {
        return m_nodes.size();
    }

    public SearchNode getNode(long ref) {
        return getNode(ref, 0);
    }

    public Map<Long, List<SearchNode>> getNodeMap() {
        return m_map;
    }
}
