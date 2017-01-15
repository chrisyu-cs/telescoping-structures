using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class TICTracer : MonoBehaviour
    {
        public TorsionImpulseCurve dCurve;
        public float arcLength;

        public bool auto = true;

        // Use this for initialization
        void Start()
        {
            SetPositionOnCurve();
        }

        void SetPositionOnCurve()
        {
            if (!dCurve) return;

            Vector3 p = dCurve.transform.position;
            Quaternion frameR = dCurve.transform.rotation;

            transform.position = frameR * dCurve.PositionAtPoint(arcLength) + p;
            OrthonormalFrame frame = dCurve.FrameAtPoint(arcLength).RotatedBy(frameR);

            Quaternion r = Quaternion.LookRotation(frame.T, frame.N);
            transform.rotation = r;
        }

        // Update is called once per frame
        void Update()
        {
            if (dCurve)
            {
                if (auto) arcLength = Mathf.Repeat(arcLength + Time.deltaTime, dCurve.ArcLength);
                SetPositionOnCurve();
            }
        }
    }

}