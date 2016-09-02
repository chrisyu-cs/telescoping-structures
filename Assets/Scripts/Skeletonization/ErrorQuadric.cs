using UnityEngine;
using System.Collections.Generic;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Telescopes
{
    public class ErrorQuadric
    {
        public Matrix<double> quadric;

        public ErrorQuadric()
        {
            quadric = DenseMatrix.Create(4, 4, 0);
        }

        public void AddQuadric(Matrix<double> m)
        {
            quadric.Add(m, quadric);
        }
        
        public void AddQuadric(ErrorQuadric other)
        {
            quadric.Add(other.quadric, quadric);
        }

        public float ErrorAtPoint(Vector3 v)
        {
            Vector<double> u = DenseVector.OfArray(new double[] { v.x, v.y, v.z, 1 });
            Vector<double> Qu = quadric * u;
            return (float)u.DotProduct(Qu);
        }

        public Vector3 OptimalPoint(Vector3 v1, Vector3 v2)
        {
            DenseMatrix M = DenseMatrix.Create(4, 4, 0);

            M[0, 0] = quadric[0, 0]; M[0, 1] = quadric[0, 1]; M[0, 2] = quadric[0, 2]; M[0, 3] = quadric[0, 3];
            M[1, 0] = quadric[0, 1]; M[1, 1] = quadric[1, 1]; M[1, 2] = quadric[1, 2]; M[1, 3] = quadric[1, 3];
            M[2, 0] = quadric[0, 2]; M[2, 1] = quadric[1, 2]; M[2, 2] = quadric[2, 2]; M[2, 3] = quadric[2, 3];
            M[3, 0] = 0;             M[3, 1] = 0;             M[3, 2] = 0;             M[3, 3] = 1;

            DenseVector rhs = DenseVector.Create(4, 0);
            rhs[3] = 1;
            DenseVector result = DenseVector.Create(4, 0);

            double det = M.Determinant();

            if (-1e-6 < det && det < 1e-6)
            {
                Vector3 bestPoint = (v1 + v2) / 2;
                float lowestError = ErrorAtPoint(bestPoint);
                float v1error = ErrorAtPoint(v1);
                if (v1error < lowestError)
                {
                    lowestError = v1error;
                    bestPoint = v1;
                }
                float v2error = ErrorAtPoint(v2);
                if (v2error < lowestError)
                {
                    lowestError = v2error;
                    bestPoint = v2;
                }
                return bestPoint;
            }
            
            M.Inverse().Multiply(rhs, result);

            Vector3 loc = new Vector3((float)result[0], (float)result[1], (float)result[2]);

            return loc;
        }
    }
}
