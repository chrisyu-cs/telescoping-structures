using UnityEngine;
using System.Collections.Generic;
using UnityEngine.UI;

namespace Telescopes
{
    public class InputCurve : MonoBehaviour
    {
        public List<SphereNode> points;
        public Transform curvePointsParent;
        public LineRenderer line;

        public SplineControlPoint selectedControlPoint;

        private int grabbedPoint;

        // Use this for initialization
        void Start()
        {
            points = new List<SphereNode>();
        }

        int NumberSpheres
        {
            get { return curvePointsParent.childCount; }
        }

        public void SelectPoint(SplineControlPoint scp)
        {
            selectedControlPoint = scp;
        }

        public void ClearSelection()
        {
            selectedControlPoint = null;
        }

        public void MovePoint(SplineControlPoint scp, Vector3 position)
        {
            int i = scp.index;
            scp.transform.position = position;
            SphereNode node = points[i];
            node.position = position;
            points[i] = node;
        }

        public void ResizePointDiff(SplineControlPoint scp, float diff)
        {
            int i = scp.index;
            float oldRadius = points[i].radius;
            ResizePoint(scp, oldRadius + diff);
        }

        public void ResizePoint(SplineControlPoint scp, float radius)
        {
            int i = scp.index;
            scp.transform.localScale = new Vector3(2 * radius, 2 * radius, 2 * radius);
            SphereNode node = points[i];
            node.radius = radius;
            points[i] = node;
        }

        // Update is called once per frame
        void Update()
        {
            bool changed = false;
            int index = NumberSpheres;

            while (NumberSpheres < points.Count)
            {
                float radius = points[index].radius;
                changed = true;
                GameObject newSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                newSphere.transform.localScale = new Vector3(2 * radius, 2 * radius, 2 * radius);
                newSphere.transform.parent = curvePointsParent;
                SplineControlPoint controlPoint = newSphere.AddComponent<SplineControlPoint>();
                controlPoint.parentCurve = this;
                controlPoint.index = NumberSpheres - 1;
                newSphere.layer = 9;
                index++;
            }
            
            if (NumberSpheres > points.Count) {
                changed = true;
                int numToDestroy = NumberSpheres - points.Count;
                for (int i = 1; i <= numToDestroy; i++)
                {
                    Destroy(curvePointsParent.GetChild(curvePointsParent.childCount - i).gameObject);
                }
            }

            if (changed) return;
            
            for (int i = 0; i < NumberSpheres; i++)
            {
                curvePointsParent.GetChild(i).transform.position = points[i].position;
            }

            line.SetVertexCount(points.Count);

            line.SetPositions(points.ConvertAll(new System.Converter<SphereNode, Vector3>(SphereNode.GetPosition)).ToArray());
        }
    }


}