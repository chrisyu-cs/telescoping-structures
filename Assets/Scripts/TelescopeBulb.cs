using UnityEngine;
using System.Collections.Generic;
using System;

namespace Telescopes
{
    public class TelescopeBulb : TelescopeElement
    {
        public TelescopeElement parent;
        public int parentElementNum;
        public float offsetAlongParent;

        public Vector3 offsetFromParent;

        public bool keepLocalPositionOnStart = false;

        public float Radius
        {
            get
            {
                return transform.localScale.x / 2;
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

        public float OverlapError()
        {
            float sumArea = 0;

            int numChildren = transform.childCount;

            // First compute amongst the children all pairwise overlaps.
            for (int i = 0; i < numChildren; i++)
            {
                for (int j = i + 1; j < numChildren; j++)
                {
                    Transform e1 = transform.GetChild(i);
                    Transform e2 = transform.GetChild(j);

                    TelescopingSegment seg1 = e1.GetComponent<TelescopingSegment>();
                    TelescopingSegment seg2 = e2.GetComponent<TelescopingSegment>();

                    if (seg1 && seg2)
                    {
                        TelescopingShell shell1 = AttachedShellForChild(seg1);
                        TelescopingShell shell2 = AttachedShellForChild(seg2);

                        Vector3 n1 = contactDirection(shell1, seg1.Reversed);
                        Vector3 n2 = contactDirection(shell2, seg2.Reversed);

                        Debug.Log("n1, n2 = " + n1 + ", " + n2);

                        sumArea += OverlapArea(n1, n2, shell1.radius, shell2.radius);
                    }
                }
            }
            if (parent)
            {
                TelescopingSegment pSeg = parent.GetComponent<TelescopingSegment>();
                for (int i = 0; i < numChildren; i++)
                {
                    Transform c = transform.GetChild(i);
                    TelescopingSegment cSeg = c.GetComponent<TelescopingSegment>();

                    if (pSeg && cSeg)
                    {
                        TelescopingShell pShell = AttachedShellForParent(pSeg);
                        TelescopingShell cShell = AttachedShellForChild(cSeg);

                        Debug.Log("Parent reversed = " + pSeg.Reversed);

                        Vector3 nParent = contactDirection(pShell, !pSeg.Reversed);
                        Vector3 nChild = contactDirection(cShell, cSeg.Reversed);

                        Vector3 n1 = contactDirection(pShell, !pSeg.Reversed);
                        Vector3 n2 = contactDirection(cShell, cSeg.Reversed);

                        Debug.Log("n1, n2 = " + n1 + ", " + n2);

                        sumArea += OverlapArea(n1, n2, pShell.radius, cShell.radius);
                    }
                }
            }

            float surfaceArea = 4 * Mathf.PI * Radius * Radius;
            Debug.Log("error = " + 4 * sumArea + " / " + surfaceArea);

            return 4 * sumArea / surfaceArea;
        }

        public void SetMaterial(Material m)
        {
            if (!meshRenderer) meshRenderer = GetComponent<MeshRenderer>();
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
            if (Input.GetKey("left shift") && Input.GetKeyDown("r"))
            {
                float error = OverlapError();
                Color c = Color.Lerp(Color.white, Color.red, error);

                if (!meshRenderer) meshRenderer = GetComponent<MeshRenderer>();
                meshRenderer.material.color = c;
            }
        }
    }
}
