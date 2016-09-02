using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using UnityEngine;

namespace UnityDDG
{
    /// <summary>
    /// Class representing a face element of a half-edge mesh.
    /// </summary>
    public class Face
    {
        public HalfEdge anyHalfEdge;

        public bool IsBoundary
        {
            get
            {
                return anyHalfEdge.onBoundary;
            }
        }

        public Vector3 Normal
        {
            get
            {
                Vector3 edge1 = anyHalfEdge.flip.vector;
                Vector3 edge2 = anyHalfEdge.next.vector;

                return Vector3.Cross(edge1, edge2).normalized;
            }
        }

        public float Area
        {
            get
            {
                Vector3 edge1 = anyHalfEdge.flip.vector;
                Vector3 edge2 = anyHalfEdge.next.vector;

                Vector3 cross = Vector3.Cross(edge1, edge2);
                return cross.magnitude / 2;
            }
        }

        public bool Valid
        {
            get
            {
                return (anyHalfEdge != null) && anyHalfEdge.Valid;
            }
        }

        public bool Collapsed = false;
        public int Index;
    }
}
