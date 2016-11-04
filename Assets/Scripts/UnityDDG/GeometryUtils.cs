using UnityEngine;
using System.Collections.Generic;

using MIConvexHull;

namespace UnityDDG
{
    public static class GeometryUtils
    {
        /// <summary>
        /// Convert from Unity vectors to convex hull vectors
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        static DefaultVertex VertOfVector3(Vector3 p)
        {
            double[] d = new double[3];
            d[0] = p.x;
            d[1] = p.y;
            d[2] = p.z;

            DefaultVertex vert = new DefaultVertex();
            vert.Position = d;
            return vert;
        }

        /// <summary>
        /// Convert from convex hull vectors to Unity vectors
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        static Vector3 Vector3OfVert(DefaultVertex v)
        {
            return new Vector3((float)v.Position[0], (float)v.Position[1], (float)v.Position[2]);
        }

        public static Mesh ConvexHull3D(List<Vector3> points)
        {
            List<DefaultVertex> vertices = points.ConvertAll(new System.Converter<Vector3, DefaultVertex>(VertOfVector3));

            ConvexHull<DefaultVertex, DefaultConvexFace<DefaultVertex>> hull;
            hull = ConvexHull.Create<DefaultVertex, DefaultConvexFace<DefaultVertex>>(vertices);

            List<Vector3> hullVerts = new List<Vector3>();
            List<int> hullFaces = new List<int>();

            int ptIndex = 0;
            Dictionary<DefaultVertex, int> pointDict = new Dictionary<DefaultVertex, int>();

            // Extract list of vertices
            foreach (DefaultVertex v in hull.Points)
            {
                Vector3 vec = Vector3OfVert(v);
                hullVerts.Add(vec);
                pointDict.Add(v, ptIndex);
                ptIndex++;
            }

            // Extract list of faces
            foreach (var f in hull.Faces)
            {
                for (int i = 0; i < 3; i++)
                {
                    int index = pointDict[f.Vertices[i]];
                    hullFaces.Add(index);
                }
            }

            Mesh m = new Mesh();
            m.vertices = hullVerts.ToArray();
            m.triangles = hullFaces.ToArray();
            m.RecalculateNormals();

            return m;
        }
    }
}