using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class BranchingStructure : MonoBehaviour
    {
        public TelescopeSegment segment1;
        public TelescopeSegment segment2;
        public TelescopeSegment segment3;

        // Use this for initialization
        void Start()
        {

        }

        float DistanceToAxis(Vector3 point, Vector3 center, Vector3 axis)
        {
            Vector3 toPoint = point - center;
            Vector3 perp = toPoint - Vector3.Dot(toPoint, axis) * axis;

            return perp.magnitude;
        }

        string StringOfVector(Vector3 v)
        {
            return "(" + v.x + ", " + v.y + ", " + v.z + ")";
        }

        void MakeEnclosingTelescope()
        {
            Vector3 center = (segment1.transform.position
                + segment2.transform.position
                + segment3.transform.position) / 3;

            float minZ = 0;
            float maxZ = 0;
            float maxRadius = 0;

            foreach (Vector3 v in segment1.FirstShell.mesh.vertices)
            {
                minZ = Mathf.Min(v.z, minZ);
                maxZ = Mathf.Max(v.z, maxZ);

                Vector3 transformed = segment1.FirstShell.transform.position
                    + segment1.FirstShell.transform.rotation * v;
                float distance = DistanceToAxis(transformed, center, Vector3.forward);
                maxRadius = Mathf.Max(maxRadius, distance);
            }

            foreach (Vector3 v in segment2.FirstShell.mesh.vertices)
            {
                minZ = Mathf.Min(v.z, minZ);
                maxZ = Mathf.Max(v.z, maxZ);

                Vector3 transformed = segment2.FirstShell.transform.position
                    + segment2.FirstShell.transform.rotation * v;
                float distance = DistanceToAxis(transformed, center, Vector3.forward);
                maxRadius = Mathf.Max(maxRadius, distance);
            }

            foreach (Vector3 v in segment3.FirstShell.mesh.vertices)
            {
                minZ = Mathf.Min(v.z, minZ);
                maxZ = Mathf.Max(v.z, maxZ);

                Vector3 transformed = segment3.FirstShell.transform.position
                    + segment3.FirstShell.transform.rotation * v;
                float distance = DistanceToAxis(transformed, center, Vector3.forward);
                maxRadius = Mathf.Max(maxRadius, distance);
            }

            Debug.Log("Max depth = " + maxZ + ", min depth = " + minZ);
            Debug.Log("Center position = " + center);
            Debug.Log("Max radius = " + maxRadius);

            TelescopeSegment seg = CreateStraightSegment(maxZ, maxRadius);
            seg.transform.position = center - 2.25f * maxZ * Vector3.forward;
            seg.ExtendImmediate(1);

            segment1.transform.parent = seg.LastShell.transform;
            segment2.transform.parent = seg.LastShell.transform;
            segment3.transform.parent = seg.LastShell.transform;
        }

        TelescopeSegment CreateStraightSegment(float length, float radius)
        {
            GameObject segObj = new GameObject();
            TelescopeSegment seg = segObj.AddComponent<TelescopeSegment>();

            List<TelescopeParameters> paramList = new List<TelescopeParameters>();
            float thickness = Constants.WALL_THICKNESS;
            float startRadius = radius + 3 * thickness + 3 * length * Constants.TAPER_SLOPE;

            TelescopeParameters firstParam = new TelescopeParameters(length,
                startRadius, thickness, 0, 0, 0);
            paramList.Add(firstParam);

            paramList.Add(new TelescopeParameters(0, 0, 0, 0, 0, 0));
            paramList.Add(new TelescopeParameters(0, 0, 0, 0, 0, 0));

            seg.material = segment1.material;
            seg.MakeShellsFromDiffs(paramList);
            seg.transform.parent = transform;

            return seg;
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown("b"))
            {
                MakeEnclosingTelescope();
            }
        }
    }

}