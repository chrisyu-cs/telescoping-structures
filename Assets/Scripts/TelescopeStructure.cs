using UnityEngine;
using System.IO;
using System.Collections.Generic;

namespace Telescopes
{
    public class TelescopeStructure : MonoBehaviour
    {
        public List<TelescopeJuncture> junctures;
        public List<TelescopeSegment> segments;

        private bool assigningJunctures;
        private int junctureToAssign;

        CameraControl mainCamera;

        void Awake()
        {
            junctures = new List<TelescopeJuncture>();
            segments = new List<TelescopeSegment>();

            assigningJunctures = false;
            junctureToAssign = 0;

            mainCamera = Camera.main.GetComponent<CameraControl>();
        }
        
        public Vector3 WorldSpaceMin
        {
            get
            {
                Vector3 minBounds;

                if (junctures.Count > 0) minBounds = junctures[0].WorldSpaceMin;
                else if (segments.Count > 0) minBounds = segments[0].WorldSpaceMin;
                else minBounds = Vector3.zero;

                foreach (TelescopeSegment segment in segments)
                {
                    minBounds = TelescopeUtils.VectorMin(minBounds, segment.WorldSpaceMin);
                }

                foreach (TelescopeJuncture junct in junctures)
                {
                    minBounds = TelescopeUtils.VectorMin(minBounds, junct.WorldSpaceMin);
                }

                return minBounds;
            }
        }

        public List<string> SCADOfCylinder(Vector3 cylBase, Vector3 cylEnd, float radius, Vector3 minOffset)
        {
            List<string> lines = new List<string>();

            float height = Vector3.Distance(cylBase, cylEnd);

            Vector3 axis = (cylEnd - cylBase).normalized;
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, axis);
            Vector3 euler = rotation.eulerAngles;

            Vector3 p = cylBase - minOffset;

            lines.Add("translate([" + p.x + "," + p.y + "," + p.z + "]) {");
            lines.Add("    rotate([0," + euler.y + ",0]) {");
            lines.Add("    rotate([" + euler.x + ",0,0]) {");
            lines.Add("    rotate([0,0," + euler.z + "]) {");
            lines.Add("        cylinder(h = " + height + ", r1 = "
                + radius + ", r2 = " + radius + ", center = false" + ", $fn=30);");
            lines.Add("    }");
            lines.Add("    }");
            lines.Add("    }");
            lines.Add("}");

            return lines;
        }

        /// <summary>
        /// Generates an OpenSCAD file, along with auxilliary .STL files,
        /// that can be compiled into 3D-printable geometry for this
        /// telescope structure.
        /// </summary>
        public void GenerateSCAD()
        {
            Debug.Log("Exporting structure...");

            Directory.CreateDirectory("scad");
            Directory.CreateDirectory("scad/stls");

            // Retract the structure completely
            foreach (TelescopeSegment segment in segments)
            {
                segment.ExtendImmediate(0);
                segment.SetSmallestExtension(0.1f);
            }

            Vector3 minBounds = WorldSpaceMin - new Vector3(0.2f, 0.2f, 0.2f);

            List<string> junctureScads = new List<string>();
            List<List<string>> shellSTLs = new List<List<string>>();

            // First output .scad files for all junctures
            foreach (TelescopeJuncture junct in junctures)
            {
                string scad = junct.WriteSTLOfJuncture(minBounds, junct.name);
                junctureScads.Add(scad);
            }

            // Now output .stl files for all segments
            foreach (TelescopeSegment segment in segments)
            {
                List<string> segSTLs = STLWriter.WriteSTLOfShells(segment, minBounds);
                shellSTLs.Add(segSTLs);
            }

            string fname = "scad/" + name + ".scad";
            Debug.Log("Writing OpenSCAD file of telescope to " + fname);

            // Finally output a .scad file that combines all of the things we just exported
            using (StreamWriter file = new StreamWriter(fname))
            {
                file.WriteLine("union() {");
                
                // Import all scad files
                foreach (string scadFile in junctureScads)
                {
                    file.WriteLine("    include <" + scadFile + ">;");
                }

                for (int i = 0; i < segments.Count; i++)
                {
                    List<string> segmentSTLs = shellSTLs[i];

                    float radius = Mathf.Min(segments[i].InnermostRadius * 0.5f, segments[i].OutermostHeight * 0.5f);

                    List<string> cylinder = SCADOfCylinder(segments[i].BasePointAtCenter,
                        segments[i].BasePointOnSurface, radius, minBounds);

                    if (Constants.ADD_SUPPORT_CHANNELS)
                    {
                        file.WriteLine("    difference() {");
                    }

                    file.WriteLine("        union() {");
                    // Write out all of the STLs for the segments of shells
                    foreach (string stlFile in segmentSTLs)
                    {
                        file.WriteLine("            import(\"" + stlFile + "\", convexity=10);");
                    }
                    file.WriteLine("        }");

                    if (Constants.ADD_SUPPORT_CHANNELS)
                    {
                        // Subtract a cylinder to provide a channel to remove support
                        foreach (string cylLine in cylinder)
                        {
                            file.WriteLine("        " + cylLine);
                        }
                        file.WriteLine("    }");
                    }
                }

                file.WriteLine("}");
            }
        }

        void AssignJunctureTypes()
        {
            if (junctures.Count == 0) return;

            junctureToAssign = 0;
            assigningJunctures = true;

            junctures[0].SetMaterial(DesignerController.instance.selectedTelescopeMaterial);
            mainCamera.LookAtTarget(junctures[0].transform, 2f);
        }

        void ReceiveJunctureInput()
        {
            if (Input.GetKeyDown("1"))
            {
                junctures[junctureToAssign].junctureType = JunctureType.ConvexHull;
            }
            else if (Input.GetKeyDown("2"))
            {
                junctures[junctureToAssign].junctureType = JunctureType.Sphere;
            }
            else return;

            junctures[junctureToAssign].SetMaterial(DesignerController.instance.defaultTelescopeMaterial);
            junctureToAssign++;
            // If there are no more junctures, finish
            if (junctureToAssign >= junctures.Count)
            {
                assigningJunctures = false;
                mainCamera.FreeMove();
            }
            else
            {
                junctures[junctureToAssign].SetMaterial(DesignerController.instance.selectedTelescopeMaterial);
                mainCamera.LookAtTarget(junctures[junctureToAssign].transform, 2f);
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("insert"))
            {
                AssignJunctureTypes();
            }

            if (assigningJunctures)
            {
                ReceiveJunctureInput();
            }
        }
    }

}