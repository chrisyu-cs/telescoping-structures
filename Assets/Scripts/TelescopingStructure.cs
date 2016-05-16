using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class TelescopingStructure : MonoBehaviour
    {
        public Material material;

        [Tooltip("The direction of the first shell.")]
        public Vector3 initialDirection = Vector3.up;
        [Tooltip("The length of the first shell.")]
        public float initialLength = 1;
        [Tooltip("The radius of the first shell.")]
        public float initialRadius = 0.5f;
        [Tooltip("The amount of curvature of the structure.")]
        public float initialCurvature = 0f;
        [Tooltip("The direction towards which the structure curves.")]
        public Vector3 curvatureDirection = Vector3.right;
        [Tooltip("How many shells will be in the structure.")]
        public int numShells = 4;

        [Tooltip("The resolution of the mesh -- how many loops each cylinder should have.")]
        public static int cutsPerCylinder = 10;
        public static int verticesPerCircle = 40;

        private List<TelescopingShell> shells;

        // Use this for initialization
        void Start()
        {
            shells = new List<TelescopingShell>();
            initialDirection.Normalize();
            curvatureDirection.Normalize();

            GameObject rootShellObj = new GameObject();
            rootShellObj.name = "shell0";
            rootShellObj.transform.parent = this.transform;
            rootShellObj.transform.localPosition = Vector3.zero;

            TelescopingShell shell = rootShellObj.AddComponent<TelescopingShell>();
            shell.GenerateGeometry(initialLength, initialRadius, 0.1f, 
                Vector3.zero, initialDirection, initialCurvature, curvatureDirection);
            shell.setMaterial(material);

            shells.Add(shell);

            for (int i = 0; i < numShells; i++)
            {

            }
        }


        // Update is called once per frame
        void Update()
        {

        }
    }

}