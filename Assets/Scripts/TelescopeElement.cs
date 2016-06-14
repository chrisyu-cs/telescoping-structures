using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public abstract class TelescopeElement : MonoBehaviour
    {
        /// <summary>
        /// Return how many children are in the chain of this element, if any.
        /// </summary>
        /// <returns></returns>
        public abstract int numChildElements();

        /// <summary>
        /// Returns the ith child of this element.
        /// </summary>
        /// <param name="i"></param>
        /// <returns></returns>
        public abstract TelescopeElement getChildElement(int i);

        public abstract void ExtendImmediate(float t);

        public abstract Vector3 getAttachmentLocation(float t);

        public abstract Quaternion getAttachmentRotation(float t);
    }
}
