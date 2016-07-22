using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    [RequireComponent(typeof(LineRenderer))]
    public class TorsionImpulseCurve : MonoBehaviour
    {

        List<CurveSegment> segments;

        public void InitFromData(List<float> impulses, List<float> lengthSteps,
            float curvature, OrthonormalFrame initialFrame)
        {
            // Create the initial segment
            CurveSegment first = new CurveSegment(curvature, 0, lengthSteps[0], initialFrame);
        }

        // Use this for initialization
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }
    }

    struct CurveSegment
    {
        // Constant curvature throughout this segment.
        float curvature;
        // Torsion impulse relative to the previous segment.
        float impulse;
        // Arc measured from the beginning of this segment.
        float arcLength;
        // Orthonormal frame at the start of this segment.
        OrthonormalFrame frame;

        public CurveSegment(float c, float i, float l, OrthonormalFrame f)
        {
            curvature = c;
            impulse = i;
            arcLength = l;
            frame = f;
        }
    } 
}