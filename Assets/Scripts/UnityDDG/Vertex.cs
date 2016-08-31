using UnityEngine;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace UnityDDG
{
    public class Vertex
    {
        /// <summary>
        /// One of the half-edges whose tail is at this vertex.
        /// </summary>
        public HalfEdge anyHalfEdge;

        private Vector3 p;

        /// <summary>
        /// The spatial position of this vertex.
        /// </summary>
        public Vector3 position
        {
            get { return p; }
            set
            {
                p = value;
                foreach (Vertex v in IdenticalVertices)
                {
                    v.p = value;
                }
            }
        }

        /// <summary>
        /// The unique integer index of this vertex.
        /// </summary>
        public int Index;

        /// <summary>
        /// A list of vertices whose positions have become merged with this one.
        /// </summary>
        public List<Vertex> IdenticalVertices = new List<Vertex>();

        /// <summary>
        /// The area of the dual cell around this vertex.
        /// </summary>
        public float DualArea
        {
            get
            {
                HalfEdge start = anyHalfEdge;
                HalfEdge he = start;
                float area = 0;
                do
                {
                    Face f = he.face;
                    area += f.Area;
                    he = he.flip.next;
                }
                while (he != start);

                return area / 3;
            }
        }

        /// <summary>
        /// A vertex could be invalidated by edge collapses.
        /// </summary>
        public bool Valid = true;
    }
}
