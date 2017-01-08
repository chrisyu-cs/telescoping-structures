using System;
using System.IO;
using System.Collections.Generic;

using UnityEngine;

namespace Telescopes
{
    public static class STLWriter
    {
        public delegate Vector3 VectorTransform(Vector3 v);

        public static void WriteSCADOfStructure(List<string> junctureScads, List<string> shellSTLs)
        {

        }

        public static void WriteSCADOfJunctureSTLs(string bulbSTL,
            List<string> unionSTLs,
            List<string> diffSTLs, string filename)
        {
            Debug.Log("Write scad to " + filename);

            using (StreamWriter file = new StreamWriter(filename))
            {
                file.WriteLine("union() {");

                if (bulbSTL.Length > 0)
                {
                    file.WriteLine("    difference() {");
                    file.WriteLine("        import(\"" + bulbSTL + "\", convexity=10);");
                    foreach (string subtractSTL in diffSTLs)
                    {
                        file.WriteLine("        import(\"" + subtractSTL + "\", convexity=10);");
                    }
                    file.WriteLine("    }");
                }
                else
                {
                    Debug.Log("Bulb stl is empty, not writing difference objects");
                }
                
                foreach (string addSTL in unionSTLs)
                {
                    file.WriteLine("    import(\"" + addSTL + "\", convexity=10);");
                }

                file.WriteLine("}");
            }
        }

        public static List<string> WriteSTLOfShells(TelescopeSegment segment, Vector3 minOffset)
        {
            List<string> stlNames = new List<string>();

            foreach (TelescopeShell shell in segment.shells)
            {
                string stl = "stls/" + shell.name + ".stl";
                using (StreamWriter file = new StreamWriter("scad/" + stl))
                {
                    VectorTransform ShellTransform = (v =>
                        shell.transform.rotation * v + shell.transform.position - minOffset);

                    List<string> shellLines = FacetsOfMesh(shell.mesh, ShellTransform);

                    file.WriteLine("solid " + shell.name);
                    foreach (string line in shellLines)
                    {
                        file.WriteLine(line);
                    }
                    file.WriteLine("endsolid");
                }
                stlNames.Add(stl);
            }
            return stlNames;
        }

        public static void WriteSTLOfSegment(TelescopeSegment segment, Vector3 minOffset, string filename)
        {
            Debug.Log("Write STL to " + filename);
            
            List<string> allLines = new List<string>();
            allLines.Add("solid " + segment.name);
            
            foreach (TelescopeShell shell in segment.shells)
            {
                VectorTransform ShellTransform = (v =>
                    shell.transform.rotation * v + shell.transform.position - minOffset);

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

        public static void WriteSTLOfMesh(Mesh m, string s, VectorTransform f)
        {
            List<string> lines = STLOfMesh(m, f);
            File.WriteAllLines(s, lines.ToArray());
        }

        public static List<string> STLOfMesh(Mesh m)
        {
            Bounds b = m.bounds;
            VectorTransform f = (v => v - b.min);

            return STLOfMesh(m, f);
        }

        public static List<string> STLOfMesh(Mesh m, VectorTransform f)
        {
            List<string> facets = FacetsOfMesh(m, f);

            facets.Insert(0, "solid " + m.name);
            facets.Add("endsolid");

            return facets;
        }


        delegate string StringOfVector(Vector3 v);

        static void CheckPositiveOctant(Vector3 v)
        {
            if (v.x < 0 || v.y < 0 || v.z < 0)
            {
                throw new System.Exception("Vector not in positive octant ("
                    + v.x + ", " + v.y + ", " + v.z + ")");
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

                StringOfVector sov = (v => v.x + " " + v.y + " " + v.z);

                lines.Add("  facet normal " + sov(-crossNormal));
                lines.Add("    outer loop");
                lines.Add("      vertex " + sov(p1));
                lines.Add("      vertex " + sov(p2));
                lines.Add("      vertex " + sov(p3));
                lines.Add("    endloop");
                lines.Add("  endfacet");
            }

            return lines;
        }

    }
}
