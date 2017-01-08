using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class DraggablePoint : MonoBehaviour
    {
        public int index;
        public float radius;
        public CatmullRomSpline parentSpline;

        List<CatmullRomSpline> splinesStart = new List<CatmullRomSpline>();
        List<CatmullRomSpline> splinesEnd = new List<CatmullRomSpline>();

        public SplineCanvas containingCanvas;

        public PointType Type;

        MeshRenderer mRenderer;

        void Start()
        {
            if (Type == PointType.Spline) radius = 0.1f;
            gameObject.layer = 9;

            if (!mRenderer) mRenderer = GetComponent<MeshRenderer>();

            if (Type == PointType.Bulb)
            {
                mRenderer.material = DesignerController.instance.defaultNodeMaterial;
            }
            else if (Type == PointType.EmptyJuncture)
            {
                mRenderer.material = DesignerController.instance.transparentNodeMaterial;
            }
        }

        public void AttachToBulb(DraggablePoint bulb)
        {
            if (Type == PointType.Bulb) return;
            if (parentSpline.points.Count < 4) return;

            if (index == 1)
            {
                parentSpline.StartBulb = bulb;
                splinesStart.Add(parentSpline);
            }
            else if (index == parentSpline.points.Count - 2)
            {
                parentSpline.EndBulb = bulb;
                splinesEnd.Add(parentSpline);
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
            else if (Type == PointType.Bulb)
            {
                foreach (CatmullRomSpline crs in splinesStart)
                {
                    crs.StartBulb = null;
                }
                foreach (CatmullRomSpline crs in splinesEnd)
                {
                    crs.EndBulb = null;
                }

                containingCanvas.DeleteBulb(this);
                Destroy(gameObject);
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

        public void ReplaceWithBulb()
        {
            if (parentSpline) parentSpline.ReplaceWithBulb(index);
        }

        public void SetSize(float f)
        {
            if (Type == PointType.Bulb || Type == PointType.EmptyJuncture)
            {
                containingCanvas.mostRecentPoint = transform.position;
                radius = Mathf.Max(f, 2 * Constants.WALL_THICKNESS);
                transform.localScale = new Vector3(radius, radius, radius) * 2;
            }
        }

        public void Resize(float f)
        {
            if (Type == PointType.Bulb || Type == PointType.EmptyJuncture)
            {
                containingCanvas.mostRecentPoint = transform.position;
                radius = Mathf.Max(radius + f, 2 * Constants.WALL_THICKNESS);
                transform.localScale = new Vector3(radius, radius, radius) * 2;
            }
        }

        public void SwitchBulbType(PointType newType)
        {
            if (!mRenderer) mRenderer = GetComponent<MeshRenderer>();

            Type = newType;

            switch (Type)
            {
                case PointType.Bulb:
                    mRenderer.material = DesignerController.instance.defaultNodeMaterial;
                    SetSize(radius);
                    GetComponent<SphereCollider>().enabled = true;
                    break;
                case PointType.Spline:
                    mRenderer.material = DesignerController.instance.defaultNodeMaterial;
                    transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                    GetComponent<SphereCollider>().enabled = true;
                    break;
                case PointType.EmptyJuncture:
                    mRenderer.material = DesignerController.instance.transparentNodeMaterial;
                    SetSize(radius);
                    GetComponent<SphereCollider>().enabled = false;
                    break;
                default:
                    break;
            }
        }

        public DCurveBulb ConvertToBulb()
        {
            if (Type != PointType.Spline)
            {
                GameObject bulbObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                bulbObj.name = "discreteBulb";
                DCurveBulb bulb = bulbObj.AddComponent<DCurveBulb>();
                bulb.InitFromPoint(this);

                gameObject.SetActive(false);

                bulb.originalType = Type;

                return bulb;
            }
            return null;
        }
    }

    public enum PointType
    {
        Spline, Bulb, EmptyJuncture
    }
}