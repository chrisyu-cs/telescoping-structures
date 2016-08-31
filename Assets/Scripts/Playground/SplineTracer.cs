using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class SplineTracer : MonoBehaviour
    {
        public CatmullRomSpline spline;
        public float parameter;

        // Update is called once per frame
        void Update()
        {
            if (spline)
            {
                parameter = Mathf.Clamp(parameter, 0, spline.NumSegments);
                transform.position = spline.sample(parameter);
                transform.rotation = Quaternion.LookRotation(spline.sampleTangent(parameter));
            }
        }
    }

}