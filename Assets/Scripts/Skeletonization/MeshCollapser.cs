using UnityEngine;
using System.Collections.Generic;

using UnityDDG;
using Priority_Queue;

namespace Telescopes
{
    public class MeshCollapser : MonoBehaviour
    {
        public HalfEdgeMesh heMesh;
        public MeshFilter meshFilter;
        Mesh mesh;

        List<Triangle> triangles;
        Dictionary<EdgePair, Triangle> edgeToTri;
        HashSet<int>[] identifiedPoints;

        UnionFind<VertexNode> quadricUF;

        TriangleNode[] triNodes;
        
        HashSet<int>[] adjacency;

        FastPriorityQueue<TriangleNode> triQueue;

        bool doCollapse;

        List<LineRenderer> lines;

        SplineCanvas outputCanvas;

        // Use this for initialization
        void Start()
        {
            if (!meshFilter) meshFilter = GetComponentInChildren<MeshFilter>();
            heMesh = GetComponent<HalfEdgeMesh>();

            triangles = new List<Triangle>();
            edgeToTri = new Dictionary<EdgePair, Triangle>();
            
            mesh = meshFilter.mesh;
            int numTriangles = mesh.triangles.Length / 3;

            for (int i = 0; i < numTriangles; i++)
            {
                Triangle t = new Triangle(mesh.triangles[3 * i],
                    mesh.triangles[3 * i + 1],
                    mesh.triangles[3 * i + 2], i);
                triangles.Add(t);

                edgeToTri.Add(new EdgePair(mesh.triangles[3 * i + 0], mesh.triangles[3 * i + 1]), t);
                edgeToTri.Add(new EdgePair(mesh.triangles[3 * i + 1], mesh.triangles[3 * i + 2]), t);
                edgeToTri.Add(new EdgePair(mesh.triangles[3 * i + 2], mesh.triangles[3 * i + 0]), t);
            }

            identifiedPoints = new HashSet<int>[mesh.vertices.Length];
            for (int i = 0; i < mesh.vertices.Length; i++)
            {
                HashSet<int> init = new HashSet<int>();
                init.Add(i);
                identifiedPoints[i] = init;
            }

            doCollapse = false;
        }

        float LengthOfEdgePair(EdgePair ep)
        {
            Vector3 vec = mesh.vertices[ep.v2] - mesh.vertices[ep.v1];
            return vec.magnitude;
        }

        EdgePair ShortestEdgeOfTriangle(Triangle tri)
        {
            Vector3 e2 = mesh.vertices[tri.v3] - mesh.vertices[tri.v2];
            Vector3 e3 = mesh.vertices[tri.v1] - mesh.vertices[tri.v3];

            EdgePair shortest = new EdgePair(tri.v1, tri.v2);
            
            if (e2.magnitude < LengthOfEdgePair(shortest))
            {
                shortest = new EdgePair(tri.v2, tri.v3);
            }
            if (e3.magnitude < LengthOfEdgePair(shortest))
            {
                shortest = new EdgePair(tri.v3, tri.v1);
            }

            return shortest;
        }

        void CollapseEdge(EdgePair ep)
        {
            Vector3[] verts = mesh.vertices;
            Triangle leftTri, rightTri;

            if (edgeToTri.TryGetValue(ep, out leftTri))
            {
                leftTri.collapsed = true;
            }
            if (edgeToTri.TryGetValue(new EdgePair(ep.v2, ep.v1), out rightTri))
            {
                rightTri.collapsed = true;
            }
            
            HashSet<int> pts1 = identifiedPoints[ep.v1];
            HashSet<int> pts2 = identifiedPoints[ep.v2];

            // Do a weighted average based on how many vertices have
            // been merged on each side
            int num1 = identifiedPoints[ep.v1].Count;
            int num2 = identifiedPoints[ep.v2].Count;
            Vector3 midpoint = (num1 * verts[ep.v1] + num2 * verts[ep.v2]) / (num1 + num2);

            pts1.UnionWith(pts2);
            
            foreach (int i in pts1)
            {
                identifiedPoints[i] = pts1;
            }

            foreach (int i in pts1)
            {
                verts[i] = midpoint;
            }

            mesh.vertices = verts;
        }

        /// <summary>
        /// Take the next triangle off the priority queue and collapse its shortest edge.
        /// </summary>
        /// <returns></returns>
        bool CollapseShortestPQ()
        {
            if (triQueue.Count == 0) return false;
            TriangleNode tNode = triQueue.Dequeue();
            if (tNode.Collapsed) return true;

            EdgePair shortestEdge = ShortestEdgeOfTriangle(tNode.triangle);
            CollapseEdge(shortestEdge);

            UpdatePQAroundMerged(shortestEdge.v1);

            return true;
        }

        void UpdatePQAroundMerged(int vertIndex)
        {
            HashSet<int> points = identifiedPoints[vertIndex];

            foreach (int v in points)
            {
                UpdatePQAroundVertex(v);
            }
        }

        void UpdatePQAroundVertex(int vertIndex)
        {
            Vertex vert = heMesh.Vertices[vertIndex];

            HalfEdge start = vert.anyHalfEdge;
            HalfEdge he = vert.anyHalfEdge;

            // Iterate over all surrounding triangles.
            do
            {
                // Get the triangle node for this face.
                Face face = he.face;
                TriangleNode node = triNodes[face.Index];

                // If the triangle is still valid, update its key to be its new shortest edge length.
                if (!node.Collapsed)
                {
                    float shortest = LengthOfEdgePair(ShortestEdgeOfTriangle(node.triangle));
                    triQueue.UpdatePriority(node, shortest);
                }

                he = he.next;
            }
            while (he != start);
        }

        void SetUpPriorityQueue()
        {
            triQueue = new FastPriorityQueue<TriangleNode>(triangles.Count);
            triNodes = new TriangleNode[triangles.Count];

            foreach (Triangle t in triangles)
            {
                TriangleNode tNode = new TriangleNode(t);
                triNodes[t.index] = tNode;
                float shortestLength = LengthOfEdgePair(ShortestEdgeOfTriangle(t));

                triQueue.Enqueue(tNode, shortestLength);
            }
        }

        int CanonicalElement(HashSet<int> set)
        {
            int canonicalPoint = 0;
            foreach (int p in set)
            {
                canonicalPoint = p;
                break;
            }
            return canonicalPoint;
        }

        void ConstructAdjacencyLists()
        {
            adjacency = new HashSet<int>[identifiedPoints.Length];
            for (int i = 0; i < identifiedPoints.Length; i++)
            {
                HashSet<int> points = identifiedPoints[i];

                // Get a canonical element for this equivalence class.
                int canonicalPoint = CanonicalElement(points);
                // If the adjacency set for this equiv. class hasn't been
                // created yet, create it.
                if (adjacency[canonicalPoint] == null)
                {
                    HashSet<int> adj = new HashSet<int>();
                    // We make sure that every point in this class references
                    // the same set.
                    foreach (int p in points)
                    {
                        adjacency[p] = adj;
                    }
                }

                // Add all neighbors from all points in the equiv. class
                foreach (int p in points)
                {
                    Vertex thisVert = heMesh.Vertices[p];
                    HalfEdge startHE = thisVert.anyHalfEdge;
                    HalfEdge he = thisVert.anyHalfEdge;
                    do
                    {
                        int neighborInd = he.headVertex.Index;
                        int neighbor = CanonicalElement(identifiedPoints[neighborInd]);
                        // Only add the neighbor if it isn't in the same equiv. class.
                        if (!points.Contains(neighbor))
                        {
                            adjacency[p].Add(neighbor);
                        }
                        he = he.flip.next;
                    }
                    while (he != startHE);
                }
            }
        }
        
        void DepthFirstSearch(int initial)
        {
            bool[] visited = new bool[identifiedPoints.Length];

            int numVisited = 0;
            foreach (bool b in visited)
            {
                if (b) numVisited++;
            }
            Debug.Log("Initial " + numVisited + " / " + visited.Length);

            Stack<IntPair> dfsStack = new Stack<IntPair>();

            dfsStack.Push(new IntPair(initial, initial));

            // Keep track of the current chain of points
            List<int> currentPoints = null;

            while (dfsStack.Count > 0)
            {
                IntPair nextPair = dfsStack.Pop();
                int next = nextPair.num;

                if (visited[next]) continue;

                // Mark this point and all points merged with it as visited
                foreach (int p in identifiedPoints[next])
                {
                    visited[p] = true;
                }

                // Add all neighbors of this point to the list
                // if they haven't been visited already
                HashSet<int> neighbors = adjacency[next];
                foreach (int neighbor in neighbors)
                {
                    if (!visited[neighbor]) dfsStack.Push(new IntPair(neighbor, next));
                }

                // If this is the first point, start a new line segment.
                if (currentPoints == null)
                {
                    currentPoints = new List<int>();
                    currentPoints.Add(next);
                }
                // If this point has degree 2 (i.e. is a part of a straight line segment)
                // then just continue the current segment.
                else if (neighbors.Count == 2)
                {
                    currentPoints.Add(next);
                }
                // If this point has degree 1, then we just end the current segment.
                else if (neighbors.Count == 1)
                {
                    currentPoints.Add(next);
                    // Can filter out single dead-ends to try to remove noise

                    //if (currentPoints.Count > 2)
                        AddLine(currentPoints);

                    // Start a new segment at the predecessor of the most recent branch point.
                    currentPoints = new List<int>();
                    if (dfsStack.Count > 0) currentPoints.Add(dfsStack.Peek().predecessor);
                }
                // If the point has degree greater than 2, then it is a branch, and we
                // start a new segment at the current point.
                else
                {
                    currentPoints.Add(next);
                    AddLine(currentPoints);

                    currentPoints = new List<int>();
                    currentPoints.Add(next);
                }
            }

            numVisited = 0;
            foreach (bool b in visited)
            {
                if (b) numVisited++;
            }
            Debug.Log("Visited " + numVisited + " / " + visited.Length);
        }

        static float hue = 0;

        void AddLine(List<int> pointIndices)
        {
            if (!outputCanvas)
            {
                outputCanvas = FindObjectOfType<SplineCanvas>();
                if (!outputCanvas)
                {
                    GameObject canvasObj = new GameObject();
                    canvasObj.name = name + "-canvas";
                    outputCanvas = canvasObj.AddComponent<SplineCanvas>();
                }
            }

            Vector3[] points = new Vector3[pointIndices.Count];
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 pos = mesh.vertices[pointIndices[i]];
                points[i] = pos;
            }

            GameObject lineObj = new GameObject();
            lineObj.transform.parent = transform;
            lineObj.transform.localPosition = Vector3.zero;
            lineObj.transform.localRotation = Quaternion.identity;
            lineObj.name = "skeleton-spline";

            
            // Create spline from points
            // Transform to world space
            for (int i = 0; i < points.Length; i++)
            {
                points[i] = transform.rotation * points[i] + transform.position;
            }

            CatmullRomSpline crs = lineObj.AddComponent<CatmullRomSpline>();
            crs.points = new List<Vector3>();
            Vector3 firstDiff = points[1] - points[0];
            Vector3 first = points[0] - 0.01f * firstDiff.normalized;

            Vector3 lastDiff = points[points.Length - 2] - points[points.Length - 1];
            Vector3 last = points[points.Length - 1] - 0.01f * lastDiff.normalized;

            crs.points.Add(first);
            crs.points.AddRange(points);
            crs.points.Add(last);

            outputCanvas.AddExistingSpline(crs);

            /*
            LineRenderer line = lineObj.AddComponent<LineRenderer>();
            line.useWorldSpace = false;

            line.SetVertexCount(points.Length);
            line.SetPositions(points);
            line.SetWidth(0.1f, 0.1f);
            line.material = DesignerController.instance.defaultLineMaterial;
            Color c = Color.HSVToRGB(hue, 1, 1);
            line.SetColors(c, c);
            hue = Mathf.Repeat(hue + 0.3f, 1);
            */
        }

        void CollapseOne()
        {
            // Collapse edges until there are no more triangles.
            bool hasMore = CollapseShortestPQ();

            if (!hasMore)
            {
                // Now construct adjacency lists.
                ConstructAdjacencyLists();

                // Find the vertex with highest degree
                int maxDeg = 0;
                int maxVert = 0;
                for (int i = 0; i < identifiedPoints.Length; i++)
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
        }

        void CollapseAll()
        {
            float startTime, endTime, diff;
            startTime = Time.realtimeSinceStartup;

            // Collapse edges until there are no more triangles.
            bool hasMore = true;
            do
            {
                hasMore = CollapseShortestPQ();
            }
            while (hasMore);

            endTime = Time.realtimeSinceStartup;
            diff = endTime - startTime;
            Debug.Log("Edge collapsing finished in " + diff + " seconds");

            // Now construct adjacency lists.
            startTime = Time.realtimeSinceStartup;
            ConstructAdjacencyLists();
            endTime = Time.realtimeSinceStartup;
            diff = endTime - startTime;
            Debug.Log("Adjacency structure finished in " + diff + " seconds");

            // Find the vertex with highest degree
            int maxDeg = 0;
            int maxVert = 0;
            for (int i = 0; i < identifiedPoints.Length; i++)
            {
                int degree = adjacency[i].Count;
                if (degree > maxDeg)
                {
                    maxVert = i;
                    maxDeg = degree;
                }
            }

            DepthFirstSearch(maxVert);
        }

        void Update()
        {
            if (!doCollapse && Input.GetKey("left shift") && Input.GetKeyDown("h"))
            {
                float startTime, endTime, diff;

                doCollapse = true;
                startTime = Time.realtimeSinceStartup;
                SetUpPriorityQueue();
                endTime = Time.realtimeSinceStartup;
                diff = endTime - startTime;
                Debug.Log("Set up PQ finished in " + diff + " seconds");

                startTime = Time.realtimeSinceStartup;
                quadricUF = VertexNode.MakeUnionFind(heMesh);
                endTime = Time.realtimeSinceStartup;
                diff = endTime - startTime;
                Debug.Log("Make union-find finished in " + diff + " seconds");

                //CollapseAll();
            }
            if (doCollapse)
            {
                CollapseOne();
            }
        }
    }

    struct IntPair
    {
        public int num;
        public int predecessor;

        public IntPair(int next, int prev)
        {
            num = next;
            predecessor = prev;
        }
    }

    class TriangleNode : FastPriorityQueueNode
    {
        public Triangle triangle;
        public TriangleNode(Triangle t)
        {
            triangle = t;
        }

        public bool Collapsed { get { return triangle.collapsed; } }
    }

    class Triangle
    {
        public int v1, v2, v3;
        public bool collapsed;
        public int index;

        public Triangle(int x1, int x2, int x3, int ind)
        {
            v1 = x1;
            v2 = x2;
            v3 = x3;
            collapsed = false;

            index = ind;
        }
    }

    class EdgePair
    {
        public int v1;
        public int v2;

        public EdgePair(int a, int b)
        {
            v1 = a;
            v2 = b;
        }

        public override int GetHashCode()
        {
            return ((3373 * v1) ^ v2);
        }

        public override bool Equals(object obj)
        {
            if (obj is EdgePair)
            {
                EdgePair other = (EdgePair)obj;
                return (v1 == other.v1) && (v2 == other.v2);
            }
            else return false;
        }
    }
}