using UnityEngine;
using System.Collections.Generic;

using Telescopes;

namespace UnityDDG
{ 
    public partial class HalfEdgeMesh : MonoBehaviour
    {
        public MeshFilter targetMesh;

        HalfEdge[] halfEdges;
        Vertex[] vertices;
        Edge[] edges;
        Face[] faces;

        public HalfEdge[] HalfEdges { get { return halfEdges; } }
        public Vertex[] Vertices { get { return vertices; } }
        public Edge[] Edges { get { return edges; } }
        public Face[] Faces { get { return faces; } }

        void Start()
        {
            MeshFilter f = GetComponentInChildren<MeshFilter>();
            if (f)
            {
                targetMesh = f;
                InitFromMesh(targetMesh.mesh);
                DesignerController.instance.currentMesh = f;
            }
        }

        void CheckValid()
        {
            foreach (Vertex v in vertices)
            {
                if (!v.Valid) continue;

                HalfEdge start = v.anyHalfEdge;
                if (!start.Valid) throw new System.Exception("Valid vertex " + v.Index + " has invalid START half-edge (tail = " + start.tailVertex.Index + ")");
                HalfEdge he = start;
                do
                {
                    if (!he.Valid)
                    {
                        throw new System.Exception("Valid vertex " + v.Index + " has invalid half-edge (tail = " + he.tailVertex.Index + ")");
                    }
                    he = he.flip.next;
                }
                while (he != start);
            }
        }

        void CollapseTriangle(HalfEdge insideHE)
        {
            // If this side is a boundary loop, we just need to point
            // the predecessor to its new successor.
            if (insideHE.onBoundary)
            {
                HalfEdge prev = insideHE.previous;
                HalfEdge next = insideHE.next;
                prev.next = next;
                insideHE.Valid = false;

                // Remap the half-edge pointer, as this one is now invalid.
                insideHE.face.anyHalfEdge = next;

                return;
            }

            insideHE.face.anyHalfEdge = null;

            // Collapse the inside triangle. Mark all inside half-edges as invalid.
            HalfEdge he = insideHE;
            do
            {
                he.Valid = false;

                // If the tail vertex of this half-edge was using the half-edge
                // as its pointer, move it so it isn't invalid.
                Vertex tail = he.tailVertex;
                if (tail.anyHalfEdge == he) tail.anyHalfEdge = he.flip.next;

                he = he.next;
            }
            while (he != insideHE);

            // Remap flips of outer edges to skip over inside edges of this triangle.
            HalfEdge outsideUpper = insideHE.next.flip;
            HalfEdge outsideLower = insideHE.previous.flip;
            outsideUpper.flip = outsideLower;
            outsideLower.flip = outsideUpper;

            Edge badEdge = outsideLower.edge;
            Edge goodEdge = outsideUpper.edge;

            // Point both half-edges to only one of the Edge objects,
            // which invalidates the other one.
            outsideUpper.edge = goodEdge;
            outsideLower.edge = goodEdge;
            // Point the chosen Edge's half-edge pointer to one of these.
            goodEdge.anyHalfEdge = outsideUpper;
            badEdge.anyHalfEdge = null;
        }

        public void CollapseEdge(Edge e)
        {
            if (!e.Valid) return;

            HalfEdge nearSide = e.anyHalfEdge;
            HalfEdge farSide = nearSide.flip;

            // Assign all half-edge tails to one of the vertices.
            Vertex keepVertex = nearSide.tailVertex;
            Vertex discardVertex = farSide.tailVertex;
            HalfEdge start = discardVertex.anyHalfEdge;
            HalfEdge he = start;
            Debug.Log("Reassigning tails from " + discardVertex.Index + " (" + discardVertex.Valid + ") to "
                + keepVertex.Index + " (" + keepVertex.Valid + ")");
            do
            {
                if (!he.Valid) throw new System.Exception("Invalid half-edge reached");
                if (he.tailVertex != discardVertex) throw new System.Exception("Tail vertex changed to "
                    + he.tailVertex.Index);
                he.tailVertex = keepVertex;
                he = he.flip.next;
            }
            while (he != start);

            // Reassign the vertex's half-edge pointer, because the current
            // one may become invalid if it is the collapsed edge.
            // Search for a half-edge not in either of the faces that will be collapsed.
            while (keepVertex.anyHalfEdge.face == nearSide.face
                || keepVertex.anyHalfEdge.face == farSide.face)
            {
                keepVertex.anyHalfEdge = keepVertex.anyHalfEdge.flip.next;
                if (keepVertex.anyHalfEdge == nearSide)
                {
                    Debug.Log("Cycled around; no possible valid edges near vertex " + keepVertex.Index);
                }
            }

            // Add the merged vertex and all of its merged vertices to the
            // list of identified vertices.
            keepVertex.IdenticalVertices.AddRange(discardVertex.IdenticalVertices);
            keepVertex.IdenticalVertices.Add(discardVertex);

            // Invalidate the other vertex.
            discardVertex.Valid = false;

            Debug.Log("Collapsing triangles");
            // Collapse triangles on both sides.
            CollapseTriangle(nearSide);
            CollapseTriangle(farSide);

            // Finally edit the positions of the merged vertices.
            Vector3 midpoint = (keepVertex.position + discardVertex.position) / 2;
            keepVertex.position = midpoint;
            discardVertex.position = midpoint;
        }

        public Vector3[] ReadPositionsFromMesh()
        {
            Vector3[] positions = new Vector3[targetMesh.mesh.vertices.Length];
            for (int i = 0; i < targetMesh.mesh.vertices.Length; i++)
            {
                positions[i] = targetMesh.mesh.vertices[i];
            }

            return positions;
        }

        public void TransferPositionsToMesh()
        {
            Vector3[] newVertices = targetMesh.mesh.vertices;
            for (int i = 0; i < vertices.Length; i++)
            {
                newVertices[i] = vertices[i].position;
            }
            targetMesh.mesh.vertices = newVertices;
        }

        void CleanUpMesh()
        {
            int numInvalidFaces = 0;
            int numInvalidEdges = 0;
            int numInvalidVertices = 0;
            int numInvalidHalfEdges = 0;

            List<HalfEdge> newHalfEdges = new List<HalfEdge>();
            List<Face> newFaces = new List<Face>();
            List<Edge> newEdges = new List<Edge>();
            List<Vertex> newVertices = new List<Vertex>();

            foreach (HalfEdge he in halfEdges)
            {
                if (he.Valid) newHalfEdges.Add(he);
                else numInvalidHalfEdges++;
            }
            foreach (Face f in faces)
            {
                if (f.Valid) newFaces.Add(f);
                else numInvalidFaces++;
            }
            foreach (Edge e in edges)
            {
                if (e.Valid) newEdges.Add(e);
                else numInvalidEdges++;
            }
            foreach (Vertex v in vertices)
            {
                if (v.Valid) newVertices.Add(v);
                else numInvalidVertices++;
            }

            halfEdges = newHalfEdges.ToArray();
            faces = newFaces.ToArray();
            edges = newEdges.ToArray();
            vertices = newVertices.ToArray();

            Debug.Log(numInvalidHalfEdges + " invalid half-edges, " +
                numInvalidVertices + " invalid vertices, " +
                numInvalidEdges + " invalid edges, " +
                numInvalidFaces + " invalid faces");

            TagIndices();

            Vector3[] positions = new Vector3[vertices.Length];
            for (int i = 0; i < positions.Length; i++)
            {
                positions[i] = vertices[i].position;
            }

            Debug.Log(positions.Length + " vertices left");

            int maxIndex = 0;
            List<int> triangles = new List<int>();
            foreach (Face f in faces)
            {
                if (f.IsBoundary) continue;
                HalfEdge he = f.anyHalfEdge;
                for (int i = 0; i < 3; i++)
                {
                    triangles.Add(he.tailVertex.Index);
                    maxIndex = Mathf.Max(maxIndex, he.tailVertex.Index);
                    he = he.next;
                }
            }
            Debug.Log("Max triangle index = " + maxIndex);
            
            targetMesh.mesh.triangles = triangles.ToArray();
            targetMesh.mesh.vertices = positions;
        }

        void Update()
        {
            /*
            if (Input.GetKeyDown("h"))
            {
                HalfEdge shortest = null;
                foreach (Face f in faces)
                {
                    if (!f.Valid || f.IsBoundary) continue;

                    HalfEdge e1 = f.anyHalfEdge;
                    HalfEdge e2 = e1.next;
                    HalfEdge e3 = e2.next;

                    if (shortest == null) shortest = e1;

                    if (e1.vector.magnitude < shortest.vector.magnitude)
                        shortest = e1;
                    if (e2.vector.magnitude < shortest.vector.magnitude)
                        shortest = e2;
                    if (e3.vector.magnitude < shortest.vector.magnitude)
                        shortest = e3;
                }
                CollapseEdge(shortest.edge);

                //CleanUpMesh();
                TransferPositionsToMesh();

                CheckValid();
            }*/
        }
    }
}