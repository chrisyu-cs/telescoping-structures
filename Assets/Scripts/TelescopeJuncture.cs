using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class TelescopeJuncture : TelescopeElement
    {
        public TelescopeElement parent;
        public int parentElementNum;
        public float offsetAlongParent;
        public Vector3 offsetFromParent;

        public bool keepLocalPositionOnStart = false;

        public TelescopingSegment parentSegment;
        public List<TelescopingSegment> childSegments;

        private MeshRenderer meshRenderer;

        public LineRenderer line;

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

            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.parent = transform;
            sphere.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            sphere.transform.localPosition = Vector3.zero;

            line = gameObject.AddComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.material = DesignerController.instance.defaultLineMaterial;
            line.SetWidth(0.1f, 0.1f);

            List<Vector3> points = new List<Vector3>();

            if (parentSegment)
            {
                points.Add(parentSegment.LastShell.transform.position - transform.position);
                points.Add(Vector3.zero);
            }

            foreach (TelescopingSegment seg in childSegments)
            {
                points.Add(seg.FirstShell.transform.position - transform.position);
                points.Add(Vector3.zero);
            }

            line.SetVertexCount(points.Count);
            line.SetPositions(points.ToArray());
        }

        public override Vector3 getAttachmentLocation(float t)
        {
            return Vector3.zero;
        }

        public override Quaternion getAttachmentRotation(float t)
        {
            return Quaternion.identity;
        }

        static int JunctureCount = 0;

        public static TelescopeJuncture CreateJuncture(Vector3 position)
        {
            // Create the actual object for the juncture
            GameObject junctureObj = new GameObject();
            junctureObj.name = "bulb" + (JunctureCount++);
            // Add the juncture component and initialize its fields
            TelescopeJuncture juncture = junctureObj.AddComponent<TelescopeJuncture>();
            juncture.transform.position = position;
            juncture.childSegments = new List<TelescopingSegment>();

            return juncture;
        }
    } 
}
