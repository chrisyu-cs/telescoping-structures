using UnityEngine;
using System.Collections;

public class UnionFind<UFNode> where UFNode : UnionFindNode, new()
{
    UFNode[] nodes;

    public int NumNodes { get { return nodes.Length; } }

    public UFNode this[int index]
    {
        get
        {
            return nodes[index];
        }
    }

    public UnionFind(int numElements)
    {
        nodes = new UFNode[numElements];
        for (int i = 0; i < nodes.Length; i++)
        {
            nodes[i] = new UFNode();
        }
    }

    public UnionFindNode Find(UnionFindNode node)
    {
        // Path compression: point directly to root of tree once found.
        if (node.Parent != node)
        {
            node.Parent = Find(node.Parent);
        }
        return node.Parent;
    }

    public UnionFindNode Find(int x)
    {
        return Find(nodes[x]);
    }

    public void Union(UnionFindNode x, UnionFindNode y)
    {
        UnionFindNode xRoot = Find(x);
        UnionFindNode yRoot = Find(y);

        if (xRoot == yRoot) return;

        // Union-by-rank: make the smaller set the child of the larger set.
        if (xRoot.Rank < yRoot.Rank)
        {
            xRoot.Parent = yRoot;
            yRoot.Merge(xRoot);
        }
        else if (xRoot.Rank > yRoot.Rank)
        {
            yRoot.Parent = xRoot;
            xRoot.Merge(yRoot);
        }
        else
        {
            // If same size, break ties either way and increase rank.
            yRoot.Parent = xRoot;
            xRoot.Rank++;
            xRoot.Merge(yRoot);
        }
    }

    public void Union(int x, int y)
    {
        Union(nodes[x], nodes[y]);
    }
}

public class UnionFindNode
{
    public UnionFindNode Parent;
    public int Rank;

    public UnionFindNode()
    {
        Parent = this;
        Rank = 0;
    }

    /// <summary>
    /// Merge the other information of the child into the parent (this).
    /// </summary>
    /// <param name="child"></param>
    public virtual void Merge(UnionFindNode child) { }
}
