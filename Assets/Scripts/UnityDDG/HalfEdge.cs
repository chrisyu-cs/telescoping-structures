using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace UnityDDG
{
    /// <summary>
    /// Class representing a half-edge in a half-edge mesh.
    /// </summary>
    public class HalfEdge
    {
        /// <summary>
        /// The "next" half-edge pointed to by this one.
        /// </summary>
        public HalfEdge next;
        /// <summary>
        /// The "flipped" half-edge of this one.
        /// </summary>
        public HalfEdge flip;
        /// <summary>
        /// The half-edge whose "next" points to this one.
        /// </summary>
        public HalfEdge previous
        {
            get
            {
                HalfEdge he = next;
                while (he.next != this)
                {
                    he = he.next;
                }
                return he;
            }
        }

        /// <summary>
        /// The vertex at the tail of this half-edge.
        /// </summary>
        public Vertex tailVertex;

        /// <summary>
        /// The vertex at the head of this half-edge.
        /// </summary>
        public Vertex headVertex { get { return flip.tailVertex; } }

        public Vector3 vector
        {
            get
            {
                return (headVertex.position - tailVertex.position);
            }
        }

        /// <summary>
        /// The edge containing this half-edge.
        /// </summary>
        public Edge edge;

        /// <summary>
        /// The face containing this half-edge.
        /// </summary>
        public Face face;

        /// <summary>
        /// Whether or not this half-edge is a part of a boundary loop.
        /// </summary>
        public bool onBoundary;

        /// <summary>
        /// A half-edge can be invalid for a variety of application-specific reasons,
        /// e.g. edge collapses.
        /// </summary>
        public bool Valid = true;
    }
}
