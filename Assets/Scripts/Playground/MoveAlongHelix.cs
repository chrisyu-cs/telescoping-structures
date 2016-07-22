using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class MoveAlongHelix : MonoBehaviour
    {
        public float curvature, torsion;
        public float arcLength;

        public float measuredArc;

        public float velocity;

        // Use this for initialization
        void Start()
        {
            arcLength = 0;
            measuredArc = 0;
            transform.position = TelescopeUtils.TranslateAlongHelix(curvature, torsion, arcLength);
            transform.rotation = TelescopeUtils.RotateAlongHelix(curvature, torsion, arcLength);
        }

        // Update is called once per frame
        void Update()
        {
            arcLength += Time.deltaTime;
            Vector3 pos = TelescopeUtils.TranslateAlongHelix(curvature, torsion, arcLength);
            float diff = Vector3.Distance(pos, transform.position);
            measuredArc += diff;
            transform.position = pos;
            transform.rotation = TelescopeUtils.RotateAlongHelix(curvature, torsion, arcLength);

            velocity = diff / Time.deltaTime;
        }
    }
}