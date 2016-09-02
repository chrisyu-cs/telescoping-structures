using UnityEngine;
using System.Collections.Generic;

using UnityDDG;
using Priority_Queue;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Telescopes
{
    public partial class MeshCollapserQEM : MonoBehaviour
    {
        public enum CollapseMode
        {
            ShortestEdge, QuadricError
        }

        Mesh mesh;
        public HalfEdgeMesh heMesh;
        ErrorQuadric[] vertQuadrics;
        FaceNode[] nodes;

        FastPriorityQueue<FaceNode> PQ;
        UnionFind<VertexNode> uf;

        bool doCollapse;
        float[] edgeCosts;

        public CollapseMode mode;

        void Start()
        {
            doCollapse = false;
            heMesh = GetComponent<HalfEdgeMesh>();
            mesh = GetComponentInChildren<MeshFilter>().mesh;
        }

        void InitQuadrics()
        {
            Matrix<double>[] faceQuadrics = new Matrix<double>[heMesh.Faces.Length];

            // Compute quadrics for each face
            for (int i = 0; i < faceQuadrics.Length; i++)
            {
                Face f = heMesh.Faces[i];

                if (f.IsBoundary) continue;

                Vector3 normal = f.Normal;
                Vector3 point = f.anyHalfEdge.tailVertex.position;

                float d = -Vector3.Dot(normal, point);

                DenseVector v = DenseVector.OfArray(new double[] { normal.x, normal.y, normal.z, d });

                faceQuadrics[i] = v.OuterProduct(v);
            }

            // Compute quadrics for each vertex by adding face quadrics
            vertQuadrics = new ErrorQuadric[heMesh.Vertices.Length];

            for (int i = 0; i < vertQuadrics.Length; i++)
            {
                vertQuadrics[i] = new ErrorQuadric();
                Vertex v = heMesh.Vertices[i];

                HalfEdge he = v.anyHalfEdge;
                HalfEdge start = he;

                do
                {
                    Face f = he.face;
                    if (!f.IsBoundary) vertQuadrics[i].AddQuadric(faceQuadrics[f.Index]);
                    he = he.flip.next;
                }
                while (he != start);
            }

            edgeCosts = new float[heMesh.Edges.Length];
            // Compute initial error values for each edge
            for (int i = 0; i < heMesh.Edges.Length; i++)
            {
                ErrorQuadric Qsum = new ErrorQuadric();
                int v1 = heMesh.Edges[i].anyHalfEdge.tailVertex.Index;
                int v2 = heMesh.Edges[i].anyHalfEdge.headVertex.Index;
                Qsum.AddQuadric(vertQuadrics[v1]);
                Qsum.AddQuadric(vertQuadrics[v2]);

                Vector3 opt = Qsum.OptimalPoint(heMesh.Vertices[v1].position, heMesh.Vertices[v2].position);
                edgeCosts[i] = Qsum.ErrorAtPoint(opt);
            }
        }

        Edge ShortestEdge(Face f)
        {
            HalfEdge e1 = f.anyHalfEdge;
            HalfEdge e2 = e1.next;
            HalfEdge e3 = e2.next;

            HalfEdge shortestHE = e1;
            float shortest = (e1.headVertex.position - e1.tailVertex.position).magnitude;
            float e2len = (e2.headVertex.position - e2.tailVertex.position).magnitude;
            float e3len = (e3.headVertex.position - e3.tailVertex.position).magnitude;

            if (e2len < shortest)
            {
                shortest = e2len;
                shortestHE = e2;
            }
            if (e3len < shortest)
            {
                shortest = e3len;
                shortestHE = e3;
            }

            return shortestHE.edge;
        }

        float LengthOfEdge(Edge e)
        {
            HalfEdge he = e.anyHalfEdge;
            return (he.headVertex.position - he.tailVertex.position).magnitude;
        }

        float QuadricCostOfEdge(Edge e)
        {
            int v1 = e.anyHalfEdge.tailVertex.Index;
            int v2 = e.anyHalfEdge.headVertex.Index;

            ErrorQuadric Qsum = new ErrorQuadric();
            Qsum.AddQuadric(uf.Find(v1).Quadric);
            Qsum.AddQuadric(uf.Find(v2).Quadric);

            Vector3 optPoint = Qsum.OptimalPoint(heMesh.Vertices[v1].position,
                heMesh.Vertices[v2].position);
            return Qsum.ErrorAtPoint(optPoint);
        }

        float Cost(Edge e)
        {
            if (mode == CollapseMode.ShortestEdge) return LengthOfEdge(e);
            else return QuadricCostOfEdge(e);
        }

        Edge OptimalEdge(Face f)
        {
            if (mode == CollapseMode.ShortestEdge) return ShortestEdge(f);
            else return LowestQuadricCostEdge(f);
        }

        Edge LowestQuadricCostEdge(Face f)
        {
            HalfEdge e1 = f.anyHalfEdge;
            HalfEdge e2 = e1.next;
            HalfEdge e3 = e2.next;

            Edge minEdge = e1.edge;
            float minCost = edgeCosts[e1.edge.Index];

            float cost2 = edgeCosts[e2.edge.Index];
            if (cost2 < minCost)
            {
                minCost = cost2;
                minEdge = e2.edge;
            }

            float cost3 = edgeCosts[e3.edge.Index];
            if (cost3 < minCost)
            {
                minCost = cost3;
                minEdge = e3.edge;
            }

            return minEdge;
        }

        void InitPQ()
        {
            Debug.Log("Init pq");
            uf = VertexNode.MakeUnionFind(heMesh);

            for (int i = 0; i < uf.NumNodes; i++)
            {
                uf[i].Quadric = vertQuadrics[i];
            }

            PQ = new FastPriorityQueue<FaceNode>(heMesh.Faces.Length);

            nodes = new FaceNode[heMesh.Faces.Length];

            for (int i = 0; i < heMesh.Faces.Length; i++)
            {
                Face f = heMesh.Faces[i];
                if (f.IsBoundary) continue;
                nodes[i] = new FaceNode(f);

                Edge lowestCost = OptimalEdge(f);
                float cost = Cost(lowestCost);
                
                PQ.Enqueue(nodes[i], cost);
            }

            Debug.Log(PQ);
        }

        void CollapseEdge(Edge e)
        {
            int v1 = e.anyHalfEdge.tailVertex.Index;
            int v2 = e.anyHalfEdge.headVertex.Index;

            // Mark faces on either side as being collapsed
            e.anyHalfEdge.face.Collapsed = true;
            e.anyHalfEdge.flip.face.Collapsed = true;

            // Collapse this edge
            uf.Union(v1, v2);

            // Write back all vertex positions
            Vector3[] verts = mesh.vertices;
            foreach (int v in uf.Find(v1).MergedPoints)
            {
                heMesh.Vertices[v].position = uf.Find(v1).Position;
                verts[v] = uf.Find(v1).Position;
            }
            mesh.vertices = verts;
        }

        bool CollapseNextEdge()
        {
            if (PQ.Count == 0) return false;

            FaceNode nextFace = PQ.Dequeue();
            // If this face is already degenerate from another edge collapse, skip it.
            if (nextFace.face.Collapsed) return true;

            Edge minEdge = OptimalEdge(nextFace.face);
            int v1 = minEdge.anyHalfEdge.tailVertex.Index;

            CollapseEdge(minEdge);

            // Update priorities around each merged vertex
            foreach (int v in uf.Find(v1).MergedPoints)
            {
                FixPQAroundVertex(heMesh.Vertices[v]);
            }

            return true;
        }

        void FixPQAroundVertex(Vertex v)
        {
            HalfEdge start = v.anyHalfEdge;
            HalfEdge he = start;

            // Update edge costs for affected (adjacent) edges
            do
            {
                Edge e = he.edge;
                edgeCosts[e.Index] = QuadricCostOfEdge(e);

                he = he.flip.next;
            }
            while (he != start);

            start = v.anyHalfEdge;
            he = start;

            // Iterate over all adjacent edges
            do
            {
                Face f = he.face;
                if (!f.IsBoundary && !f.Collapsed)
                {
                    FaceNode updateNode = nodes[f.Index];
                    Edge minEdge = OptimalEdge(f);
                    float cost = Cost(minEdge);

                    PQ.UpdatePriority(updateNode, cost);
                }

                he = he.flip.next;
            }
            while (he != start);
        }

        void MakeSkeleton()
        {
            ConstructAdjacencyLists();

            // Find the vertex with highest degree
            int maxDeg = 0;
            int maxVert = 0;
            for (int i = 0; i < heMesh.Vertices.Length; i++)
            {
                int degree = adjacency[i].Count;
                if (degree > maxDeg)
                {
                    maxVert = i;
                    maxDeg = degree;
                }
            }

            DepthFirstSearch(maxVert);

            doCollapse = false;
        }

        void CollapseOne()
        {
            bool keepGoing = CollapseNextEdge();
            if (!keepGoing)
            {
                MakeSkeleton();
            }
        }

        void CollapseAll()
        {
            bool keepGoing = true;
            while (keepGoing)
            {
                keepGoing = CollapseNextEdge();
            }
            MakeSkeleton();
        }

        void Update()
        {
            if (Input.GetKeyDown("h"))
            {
                doCollapse = true;
                InitQuadrics();
                InitPQ();

                float start = Time.realtimeSinceStartup;
                CollapseAll();
                float end = Time.realtimeSinceStartup;
                float diff = end - start;
                Debug.Log("Collapsed all triangles in " + diff + " seconds");
            }

            if (doCollapse)
            {
                //CollapseOne();
            }
        }
    }

    struct ErrorPair
    {
        public EdgePair edgePair;
        public float error;

        public ErrorPair(EdgePair ep, float er)
        {
            edgePair = ep;
            error = er;
        }
    }

    class FaceNode : FastPriorityQueueNode
    {
        public Face face;
        public bool Done;

        public FaceNode(Face f)
        {
            face = f;
            Done = false;
        }
    }
}
