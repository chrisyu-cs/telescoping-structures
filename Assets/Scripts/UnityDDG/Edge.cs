using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace UnityDDG
{
    public class Edge
    {
        public HalfEdge anyHalfEdge;

        public bool Valid
        {
            get
            {
                return (anyHalfEdge != null) && 
                    (anyHalfEdge.Valid && anyHalfEdge.flip.Valid) &&
                    (anyHalfEdge.tailVertex.Valid && anyHalfEdge.headVertex.Valid);
            }
        }

        public bool IsBoundary
        {
            get { return Valid && (anyHalfEdge.onBoundary || anyHalfEdge.flip.onBoundary); }
        }

        public int Index;
    }
}
