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

        float OverlapArea(TelescopingShell s1, TelescopingShell s2)
        {
            Vector3 dir1 = s1.transform.forward;
            Vector3 dir2 = s2.transform.forward;

            Debug.Log(dir1 + ", " + dir2);

            float distance = TelescopeUtils.GeodesicDistanceOnSphere(Radius, dir1, dir2);

            Debug.Log("distance = " + distance);

            float area = TelescopeUtils.CircleIntersectionArea(distance, s1.radius, s2.radius);
            Debug.Log("area = " + area);
            return area;
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

                    Debug.Log("Pair " + e1.name + ", " + e2.name);

                    if (seg1 && seg2)
                    {
                        TelescopingShell shell1 = seg1.shells[0];
                        TelescopingShell shell2 = seg2.shells[0];

                        sumArea += OverlapArea(shell1, shell2);
                    }
                }
            }
            if (parent)
            {
                // Now compute error between all children and the parent.
            }

            //float surfaceArea = 4 * Mathf.PI * Radius * Radius;

            return sumArea;
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
                transform.localPosition = parentElement.getAttachmentLocation(offsetAlongParent);
                transform.localRotation = parentElement.getAttachmentRotation(offsetAlongParent);
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
            if (Input.GetKeyDown("r"))
            {
                Debug.Log("Bulb overlap approximate error = " + OverlapError());
            }
        }
    } 
}
