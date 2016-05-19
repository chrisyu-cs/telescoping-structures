using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    [System.Serializable]
    public class TelescopingStructure : MonoBehaviour
    {
        public Material material;

        public GameObject fountainPrefab;

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

            // Compute the absolute parameter values from the list of diffs we are given.
            List<TelescopeParameters> concreteParams = new List<TelescopeParameters>();
            TelescopeParameters theParams = parameters[0];
            concreteParams.Add(new TelescopeParameters(parameters[0]));
            for (int i = 1; i < numShells; i++)
            {
                theParams = theParams + parameters[i];
                theParams.length -= 0; // wallThickness;
                theParams.radius -= wallThickness;
                concreteParams.Add(theParams);
            }

            // Make a pass in reverse that grows each parent so that it is large enough
            // to contain its child.
            for (int i = numShells - 1; i > 0; i--)
            {
                TelescopeUtils.growParentToChild(concreteParams[i - 1], concreteParams[i]);
            }

            // Make the shell geometry
            TelescopingShell shell = rootShellObj.AddComponent<TelescopingShell>();
            shell.GenerateGeometry(concreteParams[0]);
            shell.setMaterial(material);

            // Shells don't know anything about their position/rotation,
            // so we set that here.
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, initialDirection);
            Quaternion roll = Quaternion.AngleAxis(concreteParams[0].twistFromParent, initialDirection);
            rootShellObj.transform.rotation = roll * rotation;
            // shell.baseRadians = 0;
            shell.baseTranslation = Vector3.zero;
            shell.baseRotation = Quaternion.identity;

            shells.Add(shell);
            shell.isRoot = true;
            
            // Make all of the child shells here.
            GameObject previousShellObj = rootShellObj;
            TelescopeParameters previousParams = concreteParams[0];
            TelescopeParameters currentParams = concreteParams[0];
            for (int i = 1; i < numShells; i++)
            {
                // Make the new shell, and set the previous shell as its parent
                GameObject shellObj = new GameObject();
                shellObj.transform.parent = previousShellObj.transform;
                shellObj.name = "shell" + i;
                previousShellObj = shellObj;

                // Get the computed parameters for this and the previous shell.
                currentParams = concreteParams[i];
                previousParams = concreteParams[i - 1];

                // Make the geometry, etc.
                TelescopingShell newShell = shellObj.AddComponent<TelescopingShell>();
                newShell.GenerateGeometry(currentParams);
                newShell.setMaterial(material);

                // Set the shell's rest transformation relative to its parent.
                // When the shell's current extension ratio is 0, this is where
                // it is located relative to its parent.
                // newShell.baseRadians = newShell.radiansOfLength(wallThickness);
                newShell.baseTranslation = TelescopeUtils.childBasePosition(previousParams, currentParams);
                newShell.baseRotation = TelescopeUtils.childBaseRotation(previousParams, currentParams);
                shells.Add(newShell);
            }

            if (fountainPrefab)
            {
                GameObject fountain = Instantiate<GameObject>(fountainPrefab);
                fountain.transform.parent = previousShellObj.transform;
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

        public TelescopeParameters(TelescopeParameters toCopy)
        {
            length = toCopy.length;
            radius = toCopy.radius;
            thickness = toCopy.thickness;
            curvature = toCopy.curvature;
            twistFromParent = toCopy.twistFromParent;
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