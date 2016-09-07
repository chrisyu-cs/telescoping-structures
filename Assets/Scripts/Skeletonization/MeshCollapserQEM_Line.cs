using UnityEngine;
using System.Collections.Generic;

using UnityDDG;

namespace Telescopes
{
    public partial class MeshCollapserQEM
    {
        HashSet<int>[] adjacency;
        SplineCanvas outputCanvas;

        void ConstructAdjacencyLists()
        {
            adjacency = new HashSet<int>[heMesh.Vertices.Length];
            for (int i = 0; i < heMesh.Vertices.Length; i++)
            {
                VertexNode root = uf.Find(i);
                HashSet<int> points = root.MergedPoints;

                // If the adjacency set for this equiv. class hasn't been
                // created yet, create it.
                if (adjacency[root.Index] == null)
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
                        int neighborRoot = uf.Find(neighborInd).Index;
                        // Only add the neighbor if it isn't in the same equiv. class.
                        if (!points.Contains(neighborRoot))
                        {
                            adjacency[p].Add(neighborRoot);
                        }
                        he = he.flip.next;
                    }
                    while (he != startHE);
                }
            }
        }

        void GetCanvas()
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
        }

        void DepthFirstSearch(int initial)
        {
            GetCanvas();
            bool[] visited = new bool[heMesh.Vertices.Length];

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

            Dictionary<int, DraggablePoint> bulbDict = new Dictionary<int, DraggablePoint>();

            DraggablePoint bulbAtStart = null;

            while (dfsStack.Count > 0)
            {
                IntPair nextPair = dfsStack.Pop();
                int next = nextPair.num;

                if (visited[next]) continue;

                HashSet<int> equivClass = uf.Find(next).MergedPoints;

                // Mark this point and all points merged with it as visited
                foreach (int p in equivClass)
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
                    Vector3 pos = transform.rotation * mesh.vertices[next] + transform.position;
                    bulbAtStart = outputCanvas.AddBulb(pos);
                    bulbAtStart.SetSize(0.1f);

                    bulbDict.Add(next, bulbAtStart);

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
                    CatmullRomSpline spline = AddLine(currentPoints);
                    spline.StartBulb = bulbAtStart;

                    // Start a new segment at the predecessor of the most recent branch point.
                    currentPoints = new List<int>();
                    if (dfsStack.Count > 0)
                    {
                        currentPoints.Add(dfsStack.Peek().predecessor);
                        DraggablePoint bulb;
                        if (bulbDict.TryGetValue(dfsStack.Peek().predecessor, out bulb))
                        {
                            bulbAtStart = bulb;
                        }
                        else bulbAtStart = null;
                    }
                }
                // If the point has degree greater than 2, then it is a branch, and we
                // start a new segment at the current point.
                else
                {
                    Vector3 pos = transform.rotation * mesh.vertices[next] + transform.position;
                    DraggablePoint bulb = outputCanvas.AddBulb(pos);
                    bulb.SetSize(0.1f);
                    bulbDict.Add(next, bulb);

                    currentPoints.Add(next);
                    CatmullRomSpline spline = AddLine(currentPoints);
                    spline.EndBulb = bulb;
                    spline.StartBulb = bulbAtStart;

                    // New segments should start at the bulb we just inserted.
                    bulbAtStart = bulb;

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

        CatmullRomSpline AddLine(List<int> pointIndices)
        {

            Vector3[] points = new Vector3[pointIndices.Count];
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 pos = mesh.vertices[pointIndices[i]];
                points[i] = pos;
            }

            GameObject lineObj = new GameObject();
            lineObj.transform.parent = outputCanvas.transform;
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

            return crs;
        }
    }
}
