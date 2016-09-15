using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class DCurveBulb : MonoBehaviour
    {
        public float radius = 0.5f;

        public IParameterizedCurve parentCurve;
        private Vector3 previousParentTangent;
        public List<IParameterizedCurve> childCurves;

        public void InitFromPoint(DraggablePoint p)
        {
            radius = p.radius;
            transform.position = p.transform.position;
            transform.localScale = new Vector3(radius, radius, radius) * 2;
            transform.parent = p.transform.parent;

            childCurves = new List<IParameterizedCurve>();
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
                Vector3 translationBetween = newCenter - transform.position;

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