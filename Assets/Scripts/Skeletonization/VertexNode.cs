using UnityEngine;
using System.Collections.Generic;

using UnityDDG;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Telescopes
{
    class VertexNode : UnionFindNode
    {
        private UnionFind<VertexNode> containingUF;

        private ErrorQuadric quadric;
        
        private Vector3 position;
        private float currentCost;
        private HashSet<int> mergedPoints;

        public int Index;

        public ErrorQuadric Quadric
        {
            get { return quadric; }
            set { quadric = value; }
        }

        public Vector3 Position
        {
            get
            {
                if (Parent == this) return position;
                else return containingUF.Find(this).position;
            }
        }
        
        public float CurrentCost
        {
            get
            {
                if (Parent == this) return currentCost;
                else return containingUF.Find(this).currentCost;
            }
        }

        public HashSet<int> MergedPoints
        {
            get
            {
                if (Parent == this) return mergedPoints;
                else return containingUF.Find(this).mergedPoints;
            }
        }

        public override void Merge(UnionFindNode child)
        {
            VertexNode childNode = child as VertexNode;

            if (child.Parent != child) throw new System.Exception("Merge wasn't given parent node");

            quadric.AddQuadric(childNode.quadric);

            //position = (position + childNode.position) / 2;
            //position = quadric.OptimalPoint(position, childNode.position);

            int n1 = mergedPoints.Count;
            int n2 = childNode.mergedPoints.Count;
            position = ((n1 * position) + (n2 * childNode.position)) / (n1 + n2);

            currentCost = Cost(position);

            mergedPoints.UnionWith(childNode.mergedPoints);
        }
        
        public float Cost(Vector3 position)
        {
            return quadric.ErrorAtPoint(position);
        }

        public static UnionFind<VertexNode> MakeUnionFind(HalfEdgeMesh mesh)
        {
            UnionFind<VertexNode> uf = new UnionFind<VertexNode>(mesh.Vertices.Length);

            Matrix<double>[] faceQuadrics = new Matrix<double>[mesh.Faces.Length];
            for (int i = 0; i < faceQuadrics.Length; i++)
            {
                Face f = mesh.Faces[i];
                if (f.IsBoundary) continue;
                Vector3 normal = f.Normal;
                Vector3 v1 = f.anyHalfEdge.tailVertex.position;

                float d = -Vector3.Dot(v1, normal);

                Vector<double> v = DenseVector.OfArray(new double[] { normal.x, normal.y, normal.z, d });

                faceQuadrics[i] = v.OuterProduct(v);
            }

            for (int i = 0; i < uf.NumNodes; i++)
            {
                VertexNode current = uf[i];
                current.position = mesh.Vertices[i].position;
                current.containingUF = uf;

                ErrorQuadric vertQuadric = new ErrorQuadric();

                // Iterate over all faces surrounding this vertex and sum up the face quadrics
                HalfEdge start = mesh.Vertices[i].anyHalfEdge;
                HalfEdge he = start;
                do
                {
                    Face f = he.face;
                    if (!f.IsBoundary)
                    {
                        Matrix<double> faceQuadric = faceQuadrics[he.face.Index];
                        vertQuadric.AddQuadric(faceQuadric);
                    }
                    he = he.flip.next;
                }
                while (he != start);

                current.quadric = vertQuadric;
                current.mergedPoints = new HashSet<int>();
                current.mergedPoints.Add(i);
                current.Index = i;
            }

            return uf;
        }
    }

}