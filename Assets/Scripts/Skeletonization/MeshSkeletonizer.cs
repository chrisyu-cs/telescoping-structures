using UnityEngine;
using System.Collections.Generic;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using UnityDDG;

namespace Telescopes
{
    [RequireComponent(typeof(HalfEdgeMesh))]
    public class MeshSkeletonizer : MonoBehaviour
    {
        public HalfEdgeMesh mesh;

        Vector3[] originalPositions;
        Vector3[] lastPositions;

        bool animating = false;

        void Start()
        {
            mesh = GetComponent<HalfEdgeMesh>();
        }

        void NormalFlow(double h)
        {
            float startTime = Time.realtimeSinceStartup;

            //SparseMatrix I = SparseMatrix.CreateIdentity(3 * mesh.Vertices.Length);

            SparseMatrix W = Operators.AreaNormalMatrix(mesh, h);
            SparseMatrix M, M_inv;
            Operators.MassMatrix(out M, out M_inv, mesh);

            SparseMatrix A = W * M_inv * W;
            SparseMatrix B = h * M_inv * A;

            // Add identity along diagonal
            for (int i = 0; i < 3 * mesh.Vertices.Length; i++)
            {
                B[i, i] += 1;
            }

            DenseVector rhs = Operators.VectorOfVertexPositions(mesh);
            Vector<double> result = DenseVector.Create(rhs.Count, 0);

            B.SolveSparseLU(result, rhs);

            float endTime = Time.realtimeSinceStartup;

            Debug.Log("Solved in " + (endTime - startTime) + " seconds");
            
            for (int i = 0; i < mesh.Vertices.Length; i++)
            {
                double x = result[3 * i];
                double y = result[3 * i + 1];
                double z = result[3 * i + 2];

                mesh.Vertices[i].position = new Vector3((float)x, (float)y, (float)z);
            }
            
            mesh.TransferPositionsToMesh();
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKey("left shift") && Input.GetKeyDown("g") && mesh)
            {
                if (originalPositions == null)
                {
                    originalPositions = mesh.ReadPositionsFromMesh();
                }

                NormalFlow(1f);

                lastPositions = mesh.ReadPositionsFromMesh();
            }

            if (Input.GetKey("left shift") && Input.GetKeyDown("c"))
            {
                animating = !animating;

                if (!animating)
                {
                    for (int i = 0; i < mesh.Vertices.Length; i++)
                    {
                        mesh.Vertices[i].position = lastPositions[i];
                    }
                    mesh.TransferPositionsToMesh();
                }
            }

            if (animating)
            {
                float weight = Mathf.Repeat(Time.realtimeSinceStartup, 3) / 3f;

                for (int i = 0; i < mesh.Vertices.Length; i++)
                {
                    mesh.Vertices[i].position = Vector3.Lerp(originalPositions[i], lastPositions[i], weight);
                }

                mesh.TransferPositionsToMesh();
            }
        }
    }
}