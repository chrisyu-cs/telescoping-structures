using UnityEngine;
using System.Collections;

using UnityDDG;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

class VertexNode : UnionFindNode
{
    protected Matrix<double> quadric;
    protected int numMerged;
    protected Vector3 position;

    public override void Merge(UnionFindNode child)
    {
        base.Merge(child);

        VertexNode childNode = child as VertexNode;
        quadric = quadric + childNode.quadric;

        position = (numMerged * position) + (childNode.numMerged * childNode.position)
            / (numMerged + childNode.numMerged);

        numMerged += childNode.numMerged;
    }

    public double Cost(Vector3 position)
    {
        DenseVector u = DenseVector.OfArray(new double[] { position.x, position.y, position.z, 1 });
        Vector<double> Qu = quadric * u;
        return u.DotProduct(Qu);
    }

    public static UnionFind<VertexNode> MakeUnionFind(HalfEdgeMesh mesh)
    {
        UnionFind<VertexNode> uf = new UnionFind<VertexNode>(mesh.Vertices.Length);

        Matrix<double>[] faceQuadrics = new Matrix<double>[mesh.Faces.Length];
        for (int i = 0; i < faceQuadrics.Length; i++)
        {
            Face f = mesh.Faces[i];
            if (f.IsBoundary) continue;

            Vector3 v1 = f.anyHalfEdge.tailVertex.position;
            Vector3 v2 = f.anyHalfEdge.next.tailVertex.position;
            Vector3 v3 = f.anyHalfEdge.next.next.tailVertex.position;

            Vector3 e1 = v2 - v1;
            Vector3 e2 = v3 - v1;

            Vector3 normal = Vector3.Cross(e1, e2);

            float d = -Vector3.Dot(v1, normal);

            Vector<double> v = DenseVector.OfArray(new double[] { normal.x, normal.y, normal.z, d });

            faceQuadrics[i] = v.OuterProduct(v);
        }
        
        for (int i = 0; i < uf.NumNodes; i++)
        {
            VertexNode current = uf[i];
            current.numMerged = 1;
            current.position = mesh.Vertices[i].position;

            Matrix<double> vertQuadric = DenseMatrix.Create(4, 4, 0);

            // Iterate over all faces surrounding this vertex and sum up the face quadrics
            HalfEdge start = mesh.Vertices[i].anyHalfEdge;
            HalfEdge he = start;
            do
            {
                Face f = he.face;
                if (!f.IsBoundary)
                {
                    Matrix<double> faceQuadric = faceQuadrics[he.face.Index];
                    vertQuadric += faceQuadric;
                }
                he = he.flip.next;
            }
            while (he != start);

            current.quadric = vertQuadric;
        }

        return uf;
    }
}
