using CSparse;
using CSparse.Double;
using MathNet.Numerics.LinearAlgebra.Storage;
using MathNet.Numerics.Properties;
using System;

using UnityDDG.Sparse;

/// <summary>
/// Copied from CSparse documentation: 
/// https://github.com/wo80/CSparse.NET/wiki/Math.NET-Numerics-and-CSparse
/// </summary>
namespace MathNet.Numerics.LinearAlgebra.Double.Factorization
{
    // Create an alias for CSparse's SparseLU class.
    using CSparseLU = CSparse.Double.Factorization.SparseLU;

    public class SparseLU
    {
        int n;
        CSparseLU lu;

        private SparseLU(CSparseLU lu, int n)
        {
            this.n = n;
            this.lu = lu;
        }

        /// <summary>
        /// Compute the sparse LU factorization for given matrix.
        /// </summary>
        /// <param name="matrix">The matrix to factorize.</param>
        /// <param name="ordering">The column ordering method to use.</param>
        /// <param name="tol">Partial pivoting tolerance (form 0.0 to 1.0).</param>
        /// <returns>Sparse LU factorization.</returns>
        public static SparseLU Create(SparseMatrix matrix, ColumnOrdering ordering,
            double tol = 1.0)
        {
            int n = matrix.RowCount;
            CompressedColumnStorage A = CSparseCCS.FromMathNET(matrix, ordering);

            return new SparseLU(new CSparseLU(A, ordering, tol), n);
        }

        /// <summary>
        /// Solves a system of linear equations, <c>Ax = b</c>, with A LU factorized.
        /// </summary>
        /// <param name="input">The right hand side vector, <c>b</c>.</param>
        /// <param name="result">The left hand side vector, <c>x</c>.</param>
        public void Solve(Vector<double> input, Vector<double> result)
        {
            // Check for proper arguments.
            if (input == null)
            {
                throw new ArgumentNullException("input");
            }

            if (result == null)
            {
                throw new ArgumentNullException("result");
            }

            // Check for proper dimensions.
            if (input.Count != result.Count)
            {
                throw new ArgumentException(Resources.ArgumentVectorsSameLength);
            }

            if (input.Count != n)
            {
                throw new ArgumentException("Dimensions don't match", "input");
            }

            var b = input.Storage as DenseVectorStorage<double>;
            var x = result.Storage as DenseVectorStorage<double>;

            if (b == null || x == null)
            {
                throw new NotSupportedException("Expected dense vector storage.");
            }

            lu.SolveTranspose(b.Data, x.Data);
        }
    }
}