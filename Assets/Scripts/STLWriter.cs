using System;
using System.IO;
using System.Collections.Generic;

using UnityEngine;

namespace Telescopes
{
    public static class STLWriter
    {
        delegate Vector3 VectorTransform(Vector3 v);

        public static void WriteSTLOfSegment(TelescopingSegment segment, string filename)
        {
            segment.transform.position = Vector3.zero;
            segment.shells[0].transform.rotation = Quaternion.identity;
            segment.ExtendImmediate(0);
            List<string> allLines = new List<string>();
            allLines.Add("solid " + segment.name);

            float minX = 0, minY = 0, minZ = 0;

            foreach (TelescopingShell shell in segment.shells)
            {
                VectorTransform WorldSpace = (v => shell.transform.rotation * v + shell.transform.position);

                foreach (Vector3 v in shell.mesh.vertices)
                {
                    minX = Mathf.Min(minX, WorldSpace(v).x);
                    minY = Mathf.Min(minY, WorldSpace(v).y);
                    minZ = Mathf.Min(minZ, WorldSpace(v).z);
                }
            }

            Vector3 stlOffset = new Vector3(-minX, -minY, -minZ);
            
            foreach (TelescopingShell shell in segment.shells)
            {
                VectorTransform ShellTransform = (v =>
                    shell.transform.rotation * v + shell.transform.position + stlOffset);

                List<string> shellLines = FacetsOfMesh(shell.mesh, ShellTransform); 
                allLines.AddRange(shellLines);
            }

            allLines.Add("endsolid");
            File.WriteAllLines(filename, allLines.ToArray());
        }

        public static void WriteSTLOfMesh(Mesh m, string s)
        {
            List<string> lines = STLOfMesh(m);
            File.WriteAllLines(s, lines.ToArray());
        }

        public static List<string> STLOfMesh(Mesh m)
        {
            Bounds b = m.bounds;

            VectorTransform f = (v => v - b.min);
            List<string> facets = FacetsOfMesh(m, f);

            facets.Insert(0, "solid " + m.name);
            facets.Add("endsolid");

            return facets;
        }

        delegate string StringToVector(Vector3 v);

        static void CheckPositiveOctant(Vector3 v)
        {
            if (v.x < 0 || v.y < 0 || v.z < 0)
            {
                throw new System.Exception("Vector not in positive octant " + v);
            }
        }

        static List<string> FacetsOfMesh(Mesh m, VectorTransform f)
        {
            List<string> lines = new List<string>();

            int numTriangles = m.triangles.Length / 3;

            for (int i = 0; i < numTriangles; i++)
            {
                // Unity has clockwise winding order; STL takes CCW, so reverse
                int i1 = m.triangles[3 * i + 0];
                int i2 = m.triangles[3 * i + 1];
                int i3 = m.triangles[3 * i + 2];

                Vector3 p1 = f(m.vertices[i1]);
                Vector3 p2 = f(m.vertices[i2]);
                Vector3 p3 = f(m.vertices[i3]);

                CheckPositiveOctant(p1);
                CheckPositiveOctant(p2);
                CheckPositiveOctant(p3);

                Vector3 e1 = p2 - p1;
                Vector3 e2 = p3 - p1;
                Vector3 crossNormal = Vector3.Cross(e2.normalized, e1.normalized);

                /*
                Vector3 vertAverageNormal = m.normals[i1] + m.normals[i2] + m.normals[i3];
                vertAverageNormal.Normalize();
                float dot = Vector3.Dot(crossNormal, vertAverageNormal);
                if (dot < 0)
                {
                    throw new System.Exception("cross normal doesn't match vertex normals: " + dot);
                }
                */

                StringToVector stv = (v => v.x + " " + v.y + " " + v.z);

                lines.Add("  facet normal " + stv(crossNormal));
                lines.Add("    outer loop");
                lines.Add("      vertex " + stv(p1));
                lines.Add("      vertex " + stv(p2));
                lines.Add("      vertex " + stv(p3));
                lines.Add("    endloop");
                lines.Add("  endfacet");
            }

            return lines;
        }

    }
}
