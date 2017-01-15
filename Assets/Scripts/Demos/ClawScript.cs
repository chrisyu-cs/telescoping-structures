using UnityEngine;
using System.Collections;

using Telescopes;

namespace Telescopes.Playground
{
    public class ClawScript : MonoBehaviour
    {
        private SkinnedMeshRenderer meshRenderer;
        private Animator animator;
        public FloatToTarget teddyBear;
        private bool rescued;

        private TelescopeSegment segment;

        // Use this for initialization
        void Start()
        {
            rescued = false;
            animator = GetComponent<Animator>();

            meshRenderer = GetComponentInChildren<SkinnedMeshRenderer>();
            meshRenderer.enabled = false;
        }

        // Update is called once per frame
        void LateUpdate()
        {
            if (Input.GetKeyDown("i"))
            {
                meshRenderer.enabled = true;
                segment = FindObjectOfType<TelescopeSegment>();
                if (segment)
                {
                    TelescopeShell shell = segment.LastShell;
                    transform.parent = shell.transform;
                    segment.MinExt = 0.35f;

                    segment.ExtendImmediate(0);
                }
            }

            if (Input.GetKeyDown("2"))
            {
                SplineCanvas c = FindObjectOfType<SplineCanvas>();
                c.ReloadFromFile("rescue.canvas");
            }

            if (Input.GetKeyDown("1"))
            {
                animator.SetTrigger("toggle-close");
                if (!rescued)
                {
                    rescued = true;
                    teddyBear.transform.parent = transform;
                }
                else
                {
                    teddyBear.transform.parent = null;
                    teddyBear.activate = true;
                    CameraControl cc = Camera.main.GetComponent<CameraControl>();
                    cc.FreeMove();
                }
            }
        }
    }

}