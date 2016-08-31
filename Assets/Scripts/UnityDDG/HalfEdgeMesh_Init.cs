using UnityEngine;
using System.Collections.Generic;

using Telescopes;

namespace UnityDDG
{
    public partial class HalfEdgeMesh
    {
        public void InitFromMesh(Mesh m)
        {
            Dictionary<EdgePair, TriangleTriple> edgesToTriangles = new Dictionary<EdgePair, TriangleTriple>();
            Dictionary<EdgePair, HalfEdge> edgesToHEs = new Dictionary<EdgePair, HalfEdge>();

            Vector3[] meshVertices = m.vertices;
            int[] meshTriangles = m.triangles;
            Vector3[] meshNormals = m.normals;

            List<EdgePair>[] vertsToEdges = new List<EdgePair>[meshVertices.Length];

            int numTriangles = meshTriangles.Length / 3;

            // Map directed edges to the triangles that contain them.
            for (int i = 0; i < numTriangles; i++)
            {
                int i1 = meshTriangles[3 * i];
                int i2 = meshTriangles[3 * i + 1];
                int i3 = meshTriangles[3 * i + 2];

                TriangleTriple tri = new TriangleTriple(i1, i2, i3);
                EdgePair e1 = new EdgePair(i1, i2);
                EdgePair e2 = new EdgePair(i2, i3);
                EdgePair e3 = new EdgePair(i3, i1);

                edgesToTriangles.Add(e1, tri);
                edgesToTriangles.Add(e2, tri);
                edgesToTriangles.Add(e3, tri);

                // Also map each vertex to the set of edges originating from it.
                // We need this to traverse boundary loops later.
                if (vertsToEdges[i1] == null) vertsToEdges[i1] = new List<EdgePair>();
                vertsToEdges[i1].Add(e1);
                if (vertsToEdges[i2] == null) vertsToEdges[i2] = new List<EdgePair>();
                vertsToEdges[i2].Add(e2);
                if (vertsToEdges[i3] == null) vertsToEdges[i3] = new List<EdgePair>();
                vertsToEdges[i3].Add(e3);
            }
            int numInteriorEdges = edgesToTriangles.Count / 2;

            Vertex[] heVertices = new Vertex[m.vertices.Length];
            List<Face> heFaces = new List<Face>();
            List<Edge> heEdges = new List<Edge>();
            List<HalfEdge> hes = new List<HalfEdge>();

            // Make all the vertices we're going to need, mapped by same indices
            for (int i = 0; i < heVertices.Length; i++)
            {
                Vertex v = new Vertex();
                v.position = meshVertices[i];
                heVertices[i] = v;
            }

            // Make all the faces, mapped by same indices
            for (int i = 0; i < numTriangles; i++)
            {
                Face f = new Face();
                heFaces.Add(f);
            }

            // Make all half-edges, face by face.
            for (int triBase = 0; triBase < numTriangles; triBase++)
            {
                int triIndex = 3 * triBase;

                // For each edge in the current triangle, we make both half-edges.
                for (int vert = 0; vert < 3; vert++)
                {
                    int currOffset = vert;
                    int nextOffset = (vert + 1) % 3;

                    int v1 = meshTriangles[triIndex + currOffset];
                    int v2 = meshTriangles[triIndex + nextOffset];

                    // Make the forward-pointing half-edge.
                    EdgePair currentEdge = new EdgePair(v1, v2);
                    HalfEdge forwardHE = new HalfEdge();

                    hes.Add(forwardHE);

                    edgesToHEs.Add(currentEdge, forwardHE);

                    // Mark the new half-edge as belonging to the current face.
                    forwardHE.face = heFaces[triBase];
                    heFaces[triBase].anyHalfEdge = forwardHE;
                }

                // Now we set the "next" pointers for each half-edge in this face,
                // and also the "flip" pointers if the flip edges have already been
                // created.
                for (int vert = 0; vert < 3; vert++)
                {
                    int currOffset = vert;
                    int nextOffset = (vert + 1) % 3;
                    int lastOffset = (vert + 2) % 3;

                    int v1 = meshTriangles[triIndex + currOffset];
                    int v2 = meshTriangles[triIndex + nextOffset];
                    int v3 = meshTriangles[triIndex + lastOffset];

                    HalfEdge current = edgesToHEs[new EdgePair(v1, v2)];
                    HalfEdge nextHE = edgesToHEs[new EdgePair(v2, v3)];

                    current.next = nextHE;
                    HalfEdge flipHE;

                    if (edgesToHEs.TryGetValue(new EdgePair(v2, v1), out flipHE))
                    {
                        current.flip = flipHE;
                        flipHE.flip = current;
                    }
                }
            }

            List<EdgePair> boundaryPairs = new List<EdgePair>();
            List<HalfEdge> boundaryHEs = new List<HalfEdge>();

            int numBoundaryFaces = 0;
            int numBoundaryEdges = 0;

            // Make boundary loops
            foreach (var kv in edgesToHEs)
            {
                EdgePair edgeIndices = kv.Key;
                HalfEdge he = kv.Value;

                // If this half-edge already has a flip
                if (he.flip != null) continue;

                // Otherwise make the flip half-edge, which will be the first
                // of a boundary loop.
                EdgePair reversedEdge = new EdgePair(edgeIndices[1], edgeIndices[0]);
                HalfEdge boundaryHE = new HalfEdge();
                boundaryHE.onBoundary = true;

                boundaryPairs.Add(reversedEdge);
                boundaryHEs.Add(boundaryHE);

                // Make a new face for this boundary loop.
                Face boundaryFace = new Face();
                numBoundaryFaces++;
                heFaces.Add(boundaryFace);
                boundaryFace.anyHalfEdge = boundaryHE;
                boundaryHE.face = boundaryFace;

                // Assign the flip half-edge to the same edge as the original.
                boundaryHE.edge = he.edge;

                // The tail of this vertex is opposite the tail of the original.
                boundaryHE.tailVertex = heVertices[edgeIndices[1]];

                // Assign flips.
                he.flip = boundaryHE;
                boundaryHE.flip = he;

                numBoundaryEdges++;

                // Now we need to walk this boundary loop until the end.
                int startIndex = edgeIndices[1];
                HalfEdge prevHE = boundaryHE;
                // Search for the next edge along this boundary loop,
                // by looking for the next half-edge with no flip vertex.
                do
                {
                    EdgePair nextPair = new EdgePair();
                    foreach (EdgePair e in vertsToEdges[startIndex])
                    {
                        nextPair = e;
                        // If the next half-edge has no flip, then it's also on the boundary,
                        // so its flip is the one we want to make.
                        if (edgesToHEs[e].flip == null) break;
                    }
                    if (edgesToHEs[nextPair].flip != null) throw new System.Exception("Boundary loop hits a dead end.");

                    HalfEdge insideHE = edgesToHEs[nextPair];
                    // Make the flip of this one.
                    HalfEdge nextBoundary = new HalfEdge();
                    nextBoundary.onBoundary = true;

                    // Set flips.
                    insideHE.flip = nextBoundary;
                    nextBoundary.flip = insideHE;

                    // Since the boundary loops are pointing the opposite direction from
                    // how we're traversing, "next" from the new half-edge is actually
                    // the previous half-edge we added.
                    nextBoundary.next = prevHE;

                    // The face is the same boundary loop.
                    nextBoundary.face = boundaryFace;
                    // The edge is the same as its twin.
                    nextBoundary.edge = insideHE.edge;
                    // The vertex is the head vertex of the twin.
                    nextBoundary.tailVertex = heVertices[nextPair[1]];

                    numBoundaryEdges++;

                    // Add to lists
                    boundaryPairs.Add(new EdgePair(nextPair[1], nextPair[0]));
                    boundaryHEs.Add(nextBoundary);

                    prevHE = nextBoundary;
                    startIndex = nextPair[1];
                }
                while (startIndex != edgeIndices[0]);

                // Now prevHE comes directly after the initial half-edge, so
                // this is the initial half-edge's "next".
                boundaryHE.next = prevHE;
            }

            for (int i = 0; i < boundaryPairs.Count; i++)
            {
                edgesToHEs.Add(boundaryPairs[i], boundaryHEs[i]);
            }
            hes.AddRange(boundaryHEs);

            // Set vertices and edges
            foreach (var kv in edgesToHEs)
            {
                EdgePair edgeIndices = kv.Key;
                HalfEdge he = kv.Value;

                int baseVertex = edgeIndices[0];
                he.tailVertex = heVertices[baseVertex];
                heVertices[baseVertex].anyHalfEdge = he;

                EdgePair revEdge = new EdgePair(edgeIndices[1], edgeIndices[0]);

                // If the edge object has not been created yet, create it
                if (he.edge == null)
                {
                    Edge edgeObj = new Edge();
                    heEdges.Add(edgeObj);

                    edgeObj.anyHalfEdge = he;
                    he.edge = edgeObj;
                    he.flip.edge = edgeObj;
                }
            }

            /*
            Debug.Log("Original mesh: " +
                meshVertices.Length + " vertices, " +
                (numInteriorEdges + numBoundaryEdges / 2) + " edges, " +
                numTriangles + " faces");
            Debug.Log("Half-edge mesh: " +
                heVertices.Length + " vertices, " +
                hes.Count + " half-edges, " +
                heEdges.Count + " edges, " +
                heFaces.Count + " faces (" + numBoundaryFaces + " boundary)");
            */

            vertices = heVertices;
            edges = heEdges.ToArray();
            faces = heFaces.ToArray();
            halfEdges = hes.ToArray();

            TagIndices();

            //Validate();
            //ComputeAverageValence();
        }

        void TagIndices()
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i].Index = i;
            }

            for (int i = 0; i < edges.Length; i++)
            {
                edges[i].Index = i;
            }

            for (int i = 0; i < faces.Length; i++)
            {
                faces[i].Index = i;
            }
        }

        void Validate()
        {
            // Every edge should have an associated half-edge
            foreach (Edge e in edges)
            {
                if (e.anyHalfEdge == null) throw new System.Exception("Edge has no half-edge");
            }
            foreach (Face f in faces)
            {
                if (f.anyHalfEdge == null) throw new System.Exception("Face has no half-edge");
            }
            foreach (Vertex v in vertices)
            {
                if (v.anyHalfEdge == null) throw new System.Exception("Vertex has no half-edge");
            }

            // Every face should have a closed cycle of half-edges
            foreach (Face f in faces)
            {
                HalfEdge heStart = f.anyHalfEdge;
                HalfEdge he = heStart;

                if (heStart.onBoundary)
                {
                    int numEdges = 0;
                    do
                    {
                        numEdges++;
                        he = he.next;
                    }
                    while (he != heStart);
                    Debug.Log(numEdges + " in boundary loop");
                }

                else
                {
                    for (int i = 0; i < 3; i++)
                    {
                        he = he.next;
                        if (he.face != f) throw new System.Exception("Half-edge is not in containing face");
                    }
                    if (he != heStart) throw new System.Exception("Face doesn't have a cycle");
                }
            }

            // Every half-edge should have a twin.
            int numMissingTwins = 0;
            foreach (HalfEdge he in halfEdges)
            {
                if (he.flip == null) numMissingTwins++;
                else
                {
                    if (he.flip.flip != he) throw new System.Exception("Twin of twin isn't original half-edge");
                }
            }
        }

        void ComputeAverageValence()
        {
            float averageValence = 0;
            foreach (Vertex v in vertices)
            {
                int valence = 0;
                HalfEdge startHE = v.anyHalfEdge;
                HalfEdge he = startHE;
                do
                {
                    he = he.flip.next;
                    if (he.tailVertex != v)
                        throw new System.Exception("Half-edge tail vertex changed when iterating over neighbors");
                    valence++;
                }
                while (he != startHE);

                averageValence += valence;
            }
            averageValence /= vertices.Length;
            Debug.Log("Average valence = " + averageValence);
        }
    }
    struct TriangleTriple
    {
        int x, y, z;
        public TriangleTriple(int a, int b, int c)
        {
            x = a;
            y = b;
            z = c;
        }

        public int this[int index]
        {
            get
            {
                if (index == 0) return x;
                else if (index == 1) return y;
                else if (index == 2) return z;
                else throw new System.Exception("Cannot get index " + index + ": TriangleTriple has only three indices.");
            }
            set
            {
                if (index == 0) x = value;
                else if (index == 1) y = value;
                else if (index == 2) z = value;
                else throw new System.Exception("Cannot set index " + index + ": TriangleTriple has only three indices.");
            }
        }
    }

    struct EdgePair
    {
        int x;
        int y;

        public EdgePair(int a, int b)
        {
            x = a;
            y = b;
        }

        public override int GetHashCode()
        {
            return ((3373 * x) ^ y);
        }

        public override bool Equals(object obj)
        {
            if (obj is EdgePair)
            {
                EdgePair other = (EdgePair)obj;
                return (x == other.x) && (y == other.y);
            }
            else return false;
        }

        public int this[int index]
        {
            get
            {
                if (index == 0) return x;
                else if (index == 1) return y;
                else throw new System.Exception("Cannot get index " + index + ": EdgePair has only two indices.");
            }
            set
            {
                if (index == 0) x = value;
                else if (index == 1) y = value;
                else throw new System.Exception("Cannot set index " + index + ": EdgePair has only two indices.");
            }
        }
    }
}
