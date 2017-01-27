using UnityEngine;
using System.Collections.Generic;
using System.IO;

namespace Telescopes
{
    public class OBJSnapshotter : MonoBehaviour
    {
        SplineCanvas canvas;
        int objCount = 0;

        public float SetExtend = 0;

        // Use this for initialization
        void Start()
        {
            canvas = GetComponent<SplineCanvas>();
        }

        void DumpOBJ(TelescopeStructure structure, string filename)
        {
            using (StreamWriter file = new StreamWriter(filename))
            {
                // First write all vertices.
                int vertOffset = 1;
                List<int> vertexOffsets = new List<int>();
                // Write all vertices for all junctures
                foreach (TelescopeJuncture j in structure.junctures)
                {
                    // Record what index the first vertex should be
                    vertexOffsets.Add(vertOffset);

                    // Write all vertices
                    foreach (Vector3 vLocal in j.JunctureMesh.vertices)
                    {
                        // Write in world space
                        Vector3 v = j.transform.rotation * vLocal + j.transform.position;
                        file.WriteLine("v " + -v.x + " " + v.y + " " + v.z);
                    }

                    // Increment the offset by the vertex count we just added
                    vertOffset += j.JunctureMesh.vertexCount;
                }

                // Write all vertices for all shells
                foreach (TelescopeSegment seg in structure.segments)
                {
                    foreach (TelescopeShell s in seg.shells)
                    {
                        // Record index of first vertex
                        vertexOffsets.Add(vertOffset);

                        // Write all vertices
                        foreach (Vector3 vLocal in s.mesh.vertices)
                        {
                            // World space
                            Vector3 v = s.transform.rotation * vLocal + s.transform.position;
                            file.WriteLine("v " + -v.x + " " + v.y + " " + v.z);
                        }

                        // Increment offset
                        vertOffset += s.mesh.vertexCount;
                    }
                }

                // Now write all faces.
                int currentElement = 0;

                // Write faces for each juncture
                foreach (TelescopeJuncture j in structure.junctures)
                {
                    // Read which vertex we start numbering from
                    int startIndex = vertexOffsets[currentElement];
                    int numTriangles = j.JunctureMesh.triangles.Length / 3;
                    // Write the faces
                    for (int f = 0; f < numTriangles; f++)
                    {
                        int v1 = j.JunctureMesh.triangles[3 * f + 0] + startIndex;
                        int v2 = j.JunctureMesh.triangles[3 * f + 1] + startIndex;
                        int v3 = j.JunctureMesh.triangles[3 * f + 2] + startIndex;

                        file.WriteLine("f " + v1 + " " + v2 + " " + v3);
                    }

                    currentElement++;
                }

                // Write all faces for shells
                foreach (TelescopeSegment seg in structure.segments)
                {
                    foreach (TelescopeShell s in seg.shells)
                    {
                        int startIndex = vertexOffsets[currentElement];
                        int numTriangles = s.mesh.triangles.Length / 3;
                        // Write faces
                        for (int f = 0; f < numTriangles; f++)
                        {
                            int v1 = s.mesh.triangles[3 * f + 0] + startIndex;
                            int v2 = s.mesh.triangles[3 * f + 1] + startIndex;
                            int v3 = s.mesh.triangles[3 * f + 2] + startIndex;

                            file.WriteLine("f " + v1 + " " + v2 + " " + v3);
                        }

                        currentElement++;
                    }
                }
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (canvas && canvas.ActiveStructure && Input.GetKeyDown("\\"))
            {
                foreach (TelescopeSegment s in canvas.ActiveStructure.segments)
                {
                    s.ExtendImmediate(SetExtend);
                }
            }

            if (canvas && canvas.ActiveStructure && Input.GetKeyDown("home"))
            {
                string name = "snapshot" + objCount + ".obj";
                DumpOBJ(canvas.ActiveStructure, name);
                Debug.Log("Wrote " + name);
                objCount++;
            }
        }
    }

}