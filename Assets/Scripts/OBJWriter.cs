using UnityEngine;
using System.IO;
using System.Collections.Generic;

public static class OBJWriter {
    
    public static void ExportToOBJ(Mesh mesh, string filename)
    {
        List<string> lines = new List<string>();

        foreach (Vector3 v in mesh.vertices)
        {
            lines.Add("v " + -v.x + " " + v.y + " " + v.z);
        }

        int numTriangles = mesh.triangles.Length / 3;
        for (int i = 0; i < numTriangles; i++)
        {
            // OBJ vertex indices are 1-indexed
            int v1 = mesh.triangles[3 * i] + 1;
            int v2 = mesh.triangles[3 * i + 1] + 1;
            int v3 = mesh.triangles[3 * i + 2] + 1;

            lines.Add("f " + v1 + " " + v2 + " " + v3);
        }

        File.WriteAllLines(filename, lines.ToArray());
    }

}
