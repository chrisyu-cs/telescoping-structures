using CSparse;

namespace MathNet.Numerics.LinearAlgebra.Double
{
    using LU = MathNet.Numerics.LinearAlgebra.Double.Factorization.SparseLU;

    public static class SparseMatrixExtensions
    {
        /// <summary>
        /// Compute the sparse LU factorization for given matrix.
        /// </summary>
        /// <param name="matrix">The matrix to factorize.</param>
        /// <param name="tol">Partial pivoting tolerance (form 0.0 to 1.0).</param>
        /// <returns></returns>
        public static LU SparseLU(this SparseMatrix matrix, double tol = 1.0)
        {
            return LU.Create(matrix, ColumnOrdering.MinimumDegreeAtPlusA, tol);
        }

        public static void SolveSparseLU(this SparseMatrix matrix, Vector<double> result, Vector<double> rhs, double tol = 1.0)
        {
            LU lu = matrix.SparseLU(tol: tol);
            lu.Solve(rhs, result);
        }
    }
}