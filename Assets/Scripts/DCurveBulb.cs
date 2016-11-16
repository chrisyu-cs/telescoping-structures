using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class DCurveBulb : MonoBehaviour
    {
        public float radius = 0.5f;

        private IParameterizedCurve pc;
        public IParameterizedCurve parentCurve {
            get { return pc; }
            set
            {
                pc = value;
                previousParentTangent = parentCurve.EndTangent;
            }
        }

        private Vector3 previousParentTangent;
        public List<IParameterizedCurve> childCurves;

        public void InitFromValues(Vector3 position, float radius)
        {
            this.radius = radius;
            transform.position = position;
            transform.localScale = new Vector3(radius, radius, radius) * 2;
            childCurves = new List<IParameterizedCurve>();
        }

        public void InitFromPoint(DraggablePoint p)
        {
            transform.parent = p.transform.parent;
            InitFromValues(p.transform.position, p.radius);
        }

        public DCurveBulb Duplicate()
        {
            GameObject newBulbObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            DCurveBulb newBulb = newBulbObj.AddComponent<DCurveBulb>();

            newBulb.radius = radius;
            newBulb.transform.position = transform.position;
            newBulb.transform.localScale = transform.localScale;
            newBulb.transform.parent = transform.parent;

            newBulb.childCurves = new List<IParameterizedCurve>();

            return newBulb;
        }

        public void MoveToParentEnd()
        {
            Vector3 parentEnd = parentCurve.EndPosition;
            Vector3 tangent = parentCurve.EndTangent;

            Vector3 newPos = parentEnd + radius * tangent;
            transform.position = newPos;

            foreach (IParameterizedCurve dc in childCurves)
            {
                dc.RotateAndOffset(Quaternion.identity, newPos, radius);
            }
        }

        // Update is called once per frame
        void Update()
        {
            if (parentCurve != null)
            {
                // Compute the rotation applied by the change in tangent
                Vector3 newTangent = parentCurve.EndTangent;
                Quaternion rotationBetween = Quaternion.FromToRotation(previousParentTangent, newTangent);
                previousParentTangent = newTangent;

                Vector3 newPosition = parentCurve.EndPosition;
                Vector3 newCenter = newPosition + (radius * newTangent);

                transform.position = newCenter;
                transform.rotation = rotationBetween * transform.rotation;

                foreach (IParameterizedCurve dc in childCurves)
                {
                    if (dc != null) dc.RotateAndOffset(Quaternion.Inverse(rotationBetween), newCenter, radius);
                }
            }
        }
    }

}