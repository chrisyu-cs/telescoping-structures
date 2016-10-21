using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public partial class TelescopeBulb : TelescopeElement
    {
        public TelescopeElement parent;
        public int parentElementNum;
        public float offsetAlongParent;

        public Vector3 offsetFromParent;

        public bool keepLocalPositionOnStart = false;

        public TelescopingSegment parentSegment;
        public List<TelescopingSegment> childSegments;

        public GameObject sphereObject;

        public bool doOptimize = true;

        public float Radius
        {
            get
            {
                return sphereObject.transform.localScale.x / 2;
            }

            set
            {
                sphereObject.transform.localScale = new Vector3(value * 2, value * 2, value * 2);
            }
        }

        private MeshRenderer meshRenderer;

        public override int numChildElements()
        {
            return 1;
        }

        public override TelescopeElement getChildElement(int i)
        {
            return this;
        }

        public void SetParent(TelescopeElement element, int parentElementNum, float attachmentPoint)
        {
            if (element == null)
            {
                transform.parent = null;
                return;
            }

            parent = element;
            TelescopeElement parentElement = element.getChildElement(parentElementNum);
            this.parentElementNum = parentElementNum;
            this.offsetAlongParent = attachmentPoint;
            this.transform.parent = parentElement.transform;
            this.transform.localPosition = parentElement.getAttachmentLocation(attachmentPoint);
            this.transform.localRotation = parentElement.getAttachmentRotation(attachmentPoint);
        }

        public void SetParentToSegmentEnd(TelescopingSegment segment)
        {
            parent = segment;
            parentElementNum = segment.shells.Count - 1;

            parentSegment = segment;
            //SetParent(segment, segment.shells.Count - 1, 1);
        }

        float OverlapArea(Vector3 normal1, Vector3 normal2, float radius1, float radius2)
        {
            float distance = TelescopeUtils.GeodesicDistanceOnSphere(Radius, normal1, normal2);
            float area = TelescopeUtils.CircleIntersectionArea(distance, radius1, radius2);
            Debug.Log("area = " + area);
            return area;
        }

        TelescopingShell AttachedShellForChild(TelescopingSegment seg)
        {
            if (!seg.Reversed)
            {
                return seg.shells[0];
            }
            else {
                return seg.shells[seg.shells.Count - 1];
            }
        }

        TelescopingShell AttachedShellForParent(TelescopingSegment seg)
        {
            if (!seg.Reversed)
            {
                return seg.shells[seg.shells.Count - 1];
            }
            else
            {
                return seg.shells[0];
            }
        }

        Vector3 contactDirection(TelescopingShell shell, bool reversed)
        {
            if (!reversed) return shell.transform.forward;
            else
            {
                Vector3 dirLocal = shell.getLocalRotationAlongPath(1) * Vector3.forward;
                Vector3 dir = shell.transform.rotation * dirLocal;
                dir *= -1;
                return dir;
            }
        }

        public void SetMaterial(Material m)
        {
            if (!meshRenderer) meshRenderer = sphereObject.GetComponent<MeshRenderer>();
            meshRenderer.material = m;
        }

        public override void ExtendImmediate(float t)
        {
            // Nothing; bulbs don't reconfigure.
        }

        // Use this for initialization
        void Start()
        {
            if (parent)
            {
                TelescopeElement parentElement = parent.getChildElement(parentElementNum);
                transform.parent = parentElement.transform;

                if (!keepLocalPositionOnStart)
                {
                    transform.localPosition = parentElement.getAttachmentLocation(offsetAlongParent);
                    transform.localRotation = parentElement.getAttachmentRotation(offsetAlongParent);
                }
            }
        }

        public override Vector3 getAttachmentLocation(float t)
        {
            return Vector3.zero;
        }

        public override Quaternion getAttachmentRotation(float t)
        {
            return Quaternion.identity;
        }

        // Update is called once per frame
        void Update()
        {
            if (doOptimize && Input.GetKeyDown("]"))
            {
                OptimizeCollisions();
                /*
                foreach (TelescopingSegment seg in childSegments)
                {
                    Vector3 localTangent = seg.LocalContactTangent();
                    Vector3 worldTangent = transform.rotation * localTangent;

                    List<Vector3> points = new List<Vector3>();
                    points.Add(transform.position);
                    points.Add(transform.position + 10f * worldTangent);

                    TelescopeUtils.DisplayLine(points);
                }
                */
            }
        }
    }
}
