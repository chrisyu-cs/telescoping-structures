using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    [System.Serializable]
    public class TelescopingStructure : MonoBehaviour
    {
        public Material material;

        [Tooltip("The direction of the first shell.")]
        public Vector3 initialDirection = Vector3.up;
        [Tooltip("How many shells will be in the structure.")]
        public int numShells = 4;
        [Tooltip("The length of the first shell.")]
        public float initialLength = 1;
        [Tooltip("The radius of the first shell.")]
        public float initialRadius = 0.5f;
        [Tooltip("The amount of curvature of the structure.")]
        public float initialCurvature = 0f;
        [Tooltip("The angle that the curvature tends toward. 0 degrees = up.")]
        public float curvatureRotation = 0f;

        [Tooltip("How thick the walls of the geometry are to be.")]
        public float wallThickness = 0.1f;

        [Tooltip("The resolution of the mesh -- how many loops each cylinder should have.")]
        public static int cutsPerCylinder = 10;
        public static int verticesPerCircle = 40;

        public List<TelescopeParameters> parameters;

        private List<TelescopingShell> shells;

        // Use this for initialization
        void Start()
        {
            shells = new List<TelescopingShell>();
            initialDirection.Normalize();

            // Create an object for the first shell
            GameObject rootShellObj = new GameObject();
            rootShellObj.name = "shell0";
            rootShellObj.transform.parent = this.transform;
            rootShellObj.transform.localPosition = Vector3.zero;

            Debug.Log("Num params = " + parameters.Count);

            // Make the shell geometry
            TelescopingShell shell = rootShellObj.AddComponent<TelescopingShell>();
            shell.GenerateGeometry(parameters[0]);
            shell.setMaterial(material);

            // Shells don't know anything about their position/rotation,
            // so we set that here.
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, initialDirection);
            Quaternion roll = Quaternion.AngleAxis(parameters[0].twistFromParent, initialDirection);
            rootShellObj.transform.rotation = roll * rotation;
            shell.baseRadians = 0;

            shells.Add(shell);
            shell.isRoot = true;
            
            GameObject previousShellObj = rootShellObj;
            TelescopeParameters currentParams = parameters[0];

            for (int i = 1; i < numShells; i++)
            {
                // Make the new shell, and set the previous shell as its parent
                GameObject shellObj = new GameObject();
                shellObj.transform.parent = previousShellObj.transform;
                shellObj.name = "shell" + i;
                previousShellObj = shellObj;

                // Make the geometry, etc.
                TelescopingShell newShell = shellObj.AddComponent<TelescopingShell>();
                currentParams = currentParams + parameters[i];
                currentParams.length -= 2 * wallThickness;
                currentParams.radius -= wallThickness;
                newShell.GenerateGeometry(currentParams);
                newShell.setMaterial(material);

                // Set the shell's rest transformation relative to its parent.
                // When the shell's current extension ratio is 0, this is where
                // it is located relative to its parent.
                newShell.baseRadians = newShell.radiansOfLength(wallThickness);

                shells.Add(newShell);
            }
        }


        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("e"))
            {
                foreach (TelescopingShell ts in shells)
                {
                    ts.extendToRatio(1, 2f);
                }
            }
            else if (Input.GetKeyDown("q"))
            {
                foreach (TelescopingShell ts in shells)
                {
                    ts.extendToRatio(0, 2f);
                }
            }
            // Live-update the orientation for better testing
            /*
            initialDirection.Normalize();
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, initialDirection);
            Quaternion roll = Quaternion.AngleAxis(curvatureRotation, initialDirection);
            shells[0].transform.rotation = roll * rotation;*/
        }
    }

    [System.Serializable]
    public class TelescopeParameters
    {
        public float length;
        public float radius;
        public float thickness;
        public float curvature;
        public float twistFromParent;

        public TelescopeParameters(float l, float r, float w, float c, float t)
        {
            length = l;
            radius = r;
            thickness = w;
            curvature = c;
            twistFromParent = t;
        }

        public static TelescopeParameters operator +(TelescopeParameters t1, TelescopeParameters t2)
        {
            TelescopeParameters sum = new TelescopeParameters(t1, t2);
            return sum;
        }

        public TelescopeParameters(TelescopeParameters baseParams, TelescopeParameters diff)
        {
            length = baseParams.length + diff.length;
            radius = baseParams.radius + diff.radius;
            thickness = baseParams.thickness;
            curvature = baseParams.curvature + diff.curvature;
            twistFromParent = diff.twistFromParent;
        }
    }
}