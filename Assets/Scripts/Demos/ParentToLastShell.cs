using UnityEngine;
using System.Collections;

using Telescopes;

namespace Telescopes.Playground
{
    public class ParentToLastShell : MonoBehaviour
    {
        public enum ParentShellMode
        {
            First, Last
        }

        public TelescopeSegment segment;

        public ParentShellMode mode;

        // Update is called once per frame
        void LateUpdate()
        {
            if (transform.parent == null)
            {
                if (mode == ParentShellMode.First)
                    transform.parent = segment.FirstShell.transform;
                else
                    transform.parent = segment.LastShell.transform;
            }
        }
    }
}