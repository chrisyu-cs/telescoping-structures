using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class AddShellAxis : MonoBehaviour
    {
        public Material lineMaterial;

        private TelescopeSegment segment;
        private bool added;

        // Use this for initialization
        void Start()
        {
            added = false;
            segment = GetComponent<TelescopeSegment>();
        }

        void AddAxes()
        {
            foreach (TelescopeShell shell in segment.shells)
            {
                GameObject axisObj = new GameObject();
                TorsionImpulseCurve axis = axisObj.AddComponent<TorsionImpulseCurve>();

                List<float> impulses = new List<float>();
                impulses.Add(0);
                List<float> arcs = new List<float>();
                arcs.Add(shell.length);

                OrthonormalFrame frame = new OrthonormalFrame(Vector3.forward, Vector3.up, Vector3.left);

                axis.InitFromData(impulses, arcs, shell.curvature, -shell.torsion, frame, Vector3.zero);
                LineRenderer axisLR = axis.GetComponent<LineRenderer>();
                axisLR.useWorldSpace = false;
                axisLR.material = lineMaterial;

                axis.transform.parent = shell.transform;
                axis.transform.localPosition = Vector3.zero;
                axis.transform.localRotation = Quaternion.identity;
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("m") && !added)
            {
                added = true;
                AddAxes();
            }
        }
    } 
}
