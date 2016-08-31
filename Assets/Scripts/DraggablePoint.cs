using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class DraggablePoint : MonoBehaviour
    {
        public int index;
        public float radius;
        public CatmullRomSpline parentSpline;

        public SplineCanvas containingCanvas;

        public PointType Type;

        void Start()
        {
            if (Type == PointType.Spline) radius = 0.1f;
            gameObject.layer = 9;
        }

        public void AttachToBulb(DraggablePoint bulb)
        {
            if (Type == PointType.Bulb) return;
            if (parentSpline.points.Count < 4) return;

            int tangentIndex;
            if (index == 1)
            {
                tangentIndex = 0;
                parentSpline.StartBulb = bulb;
            }
            else if (index == parentSpline.points.Count - 2)
            {
                tangentIndex = parentSpline.points.Count - 1;
                parentSpline.EndBulb = bulb;
            }
            else return;
        }

        public bool IsEndPoint()
        {
            if (parentSpline.points.Count < 4) return false;
            return (index == 1) || (index == parentSpline.points.Count - 2);
        }

        public void FollowBulb(DraggablePoint bulb, Vector3 tangent)
        {
            float radius = bulb.radius;
            Vector3 newPosition = bulb.transform.position + (radius * tangent);

            Move(newPosition);
        }

        public void Move(Vector3 pos)
        {
            transform.position = pos;
            if (Type == PointType.Spline && parentSpline)
            {
                containingCanvas.mostRecentPoint = transform.position;
                parentSpline.UpdatePosition(index, pos);
            }
        }

        public void Delete()
        {
            if (Type == PointType.Spline && parentSpline)
            {
                parentSpline.DeletePoint(index);
            }
        }

        public void Duplicate()
        {
            if (Type != PointType.Spline) return;
            if (parentSpline.points.Count >= 4 && (index == 0 || index == parentSpline.points.Count - 1)) return;
            if (parentSpline)
            {
                containingCanvas.mostRecentPoint = transform.position;
                parentSpline.DuplicatePoint(index);
            }
        }

        public void Resize(float f)
        {
            if (Type == PointType.Bulb)
            {
                containingCanvas.mostRecentPoint = transform.position;
                radius = Mathf.Max(radius + f, 0.05f);
                transform.localScale = new Vector3(radius, radius, radius) * 2;
            }
        }

        public DCurveBulb ConvertToBulb()
        {
            if (Type == PointType.Bulb)
            {
                GameObject bulbObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                bulbObj.name = "discreteBulb";
                DCurveBulb bulb = bulbObj.AddComponent<DCurveBulb>();
                bulb.InitFromPoint(this);

                gameObject.SetActive(false);

                return bulb;
            }
            return null;
        }
    }

    public enum PointType
    {
        Spline, Bulb
    }
}