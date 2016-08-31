using CSparse;
using CSparse.Double;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Storage;
using MathNet.Numerics.Properties;
using System;

namespace UnityDDG.Sparse
{
    public static class CSparseCCS
    {
        public static CompressedColumnStorage FromMathNET(SparseMatrix matrix,
            ColumnOrdering ordering = ColumnOrdering.MinimumDegreeAtPlusA)
        {
            int n = matrix.RowCount;

            // Check for proper dimensions.
            if (n != matrix.ColumnCount)
            {
                throw new ArgumentException(Resources.MatrixMustBeSparse);
            }

            // Get CSR storage.
            var storage = (SparseCompressedRowMatrixStorage<double>)matrix.Storage;

            // Create CSparse matrix.
            var A = new CompressedColumnStorage(n, n);

            // Assign storage arrays.
            A.ColumnPointers = storage.RowPointers;
            A.RowIndices = storage.ColumnIndices;
            A.Values = storage.Values;

            return A;
        }
    }
}
