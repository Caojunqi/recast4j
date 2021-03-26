package org.recast4j.polyanya;

import org.recast4j.detour.Node;

import java.util.PriorityQueue;

/**
 * @author Caojunqi
 * @date 2021-03-24 18:24
 */
public class SearchNodeQueue {
    private final PriorityQueue<SearchNode> m_heap = new PriorityQueue<>();

    public void clear() {
        m_heap.clear();
    }

    public SearchNode top() {
        return m_heap.peek();
    }

    public SearchNode pop() {
        return m_heap.poll();
    }

    public void push(SearchNode node) {
        m_heap.offer(node);
    }

    public void modify(SearchNode node) {
        m_heap.remove(node);
        m_heap.offer(node);
    }

    public boolean isEmpty() {
        return m_heap.isEmpty();
    }
}
