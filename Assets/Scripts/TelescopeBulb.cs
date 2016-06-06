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

        public override int numChildElements()
        {
            return 1;
        }

        public override TelescopeElement getChildElement(int i)
        {
            return this;
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

        }
    } 
}
