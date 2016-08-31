using UnityEngine;
using System.Collections.Generic;

using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace UnityDDG
{
    public static class Operators
    {
        static DenseMatrix CrossProductMatrix(Vector3 vec)
        {
            DenseMatrix mat = DenseMatrix.Create(3, 3, 0);

            /* mat[0,0] = 0 */  mat[0, 1] = -vec.z; mat[0, 2] = vec.y;
            mat[1, 0] = vec.z;  /* mat[1,1] = 0 */  mat[1, 2] = -vec.x;
            mat[2, 0] = -vec.y; mat[2, 1] = vec.x;  /* mat[2,2] = 0 */

            return mat;
        }

        /// <summary>
        /// Returns I + hA, where A is the matrix mapping vertex positions to area normals.
        /// </summary>
        /// <param name="mesh"></param>
        /// <param name="h"></param>
        /// <returns></returns>
        public static SparseMatrix AreaNormalMatrix(HalfEdgeMesh mesh, double h)
        {
            int numVertices = mesh.Vertices.Length;
            List<Tuple<int, int, double>> triplets = new List<Tuple<int, int, double>>();

            // Compute the matrix row by row
            for (int i = 0; i < numVertices; i++)
            {
                HalfEdge startHE = mesh.Vertices[i].anyHalfEdge;
                HalfEdge he = startHE;

                int rowBase = 3 * i;

                DenseMatrix A_ii = DenseMatrix.Create(3, 3, 0);

                do
                {
                    // This is edge e(i -> a)
                    HalfEdge hePrev = he;
                    Vector3 f_a = hePrev.headVertex.position;

                    // This is edge e(i -> j), which is the one we're
                    // actually computing the matrix element for
                    HalfEdge heCenter = hePrev.flip.next;
                    Vector3 e_ij = heCenter.headVertex.position - heCenter.tailVertex.position;

                    // This is edge e(i -> b)
                    HalfEdge heNext = heCenter.flip.next;
                    Vector3 f_b = heNext.headVertex.position;

                    int columnBase = 3 * heCenter.headVertex.Index;

                    Vector3 w_ij = (f_a - f_b) / 12;

                    DenseMatrix A_ij = -1 * CrossProductMatrix(w_ij);

                    triplets.Add(new Tuple<int, int, double>(rowBase + 0, columnBase + 1, A_ij[0, 1]));
                    triplets.Add(new Tuple<int, int, double>(rowBase + 0, columnBase + 2, A_ij[0, 2]));
                    triplets.Add(new Tuple<int, int, double>(rowBase + 1, columnBase + 0, A_ij[1, 0]));
                    triplets.Add(new Tuple<int, int, double>(rowBase + 1, columnBase + 2, A_ij[1, 2]));
                    triplets.Add(new Tuple<int, int, double>(rowBase + 2, columnBase + 0, A_ij[2, 0]));
                    triplets.Add(new Tuple<int, int, double>(rowBase + 2, columnBase + 1, A_ij[2, 1]));

                    A_ii += A_ij;

                    he = he.flip.next;
                }
                while (he != startHE);

                A_ii *= -1;

                triplets.Add(new Tuple<int, int, double>(rowBase + 0, rowBase + 0, A_ii[0, 0]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 0, rowBase + 1, A_ii[0, 1]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 0, rowBase + 2, A_ii[0, 2]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 1, rowBase + 0, A_ii[1, 0]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 1, rowBase + 1, A_ii[1, 1]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 1, rowBase + 2, A_ii[1, 2]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 2, rowBase + 0, A_ii[2, 0]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 2, rowBase + 1, A_ii[2, 1]));
                triplets.Add(new Tuple<int, int, double>(rowBase + 2, rowBase + 2, A_ii[2, 2]));
            }

            return SparseMatrix.OfIndexed(3 * numVertices, 3 * numVertices, triplets);
        }

        public static void MassMatrix(out SparseMatrix M, out SparseMatrix M_inverse, HalfEdgeMesh mesh)
        {
            int numVertices = mesh.Vertices.Length;

            List<Tuple<int, int, double>> M_triples = new List<Tuple<int, int, double>>();
            List<Tuple<int, int, double>> M_inv_triples = new List<Tuple<int, int, double>>();

            for (int i = 0; i < numVertices; i++)
            {
                int vBase = 3 * i;
                float area = mesh.Vertices[i].DualArea;

                for (int n = 0; n < 3; n++)
                {
                    M_triples.Add(new Tuple<int, int, double>(vBase + n, vBase + n, area));
                    M_inv_triples.Add(new Tuple<int, int, double>(vBase + n, vBase + n, 1f / area));
                }
            }

            M = SparseMatrix.OfIndexed(3 * numVertices, 3 * numVertices, M_triples);
            M_inverse = SparseMatrix.OfIndexed(3 * numVertices, 3 * numVertices, M_inv_triples);
        }

        public static DenseVector VectorOfVertexPositions(HalfEdgeMesh mesh)
        {
            int numVertices = mesh.Vertices.Length;

            DenseVector vec = DenseVector.Create(3 * numVertices, 0);

            for (int i = 0; i < numVertices; i++)
            {
                Vector3 pos = mesh.Vertices[i].position;
                int vBase = 3 * i;
                vec[vBase + 0] = pos.x;
                vec[vBase + 1] = pos.y;
                vec[vBase + 2] = pos.z;
            }

            return vec;
        }
    }
}
