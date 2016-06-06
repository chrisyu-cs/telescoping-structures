using UnityEngine;
using System.Collections;

namespace Telescopes
{
    public class SplineControlPoint : MonoBehaviour
    {
        public int index;
        public InputCurve parentCurve;

        public void Move(Vector3 pos)
        {
            parentCurve.MovePoint(this, pos);
        }

        public void ResizeDiff(float diff)
        {
            parentCurve.ResizePointDiff(this, diff);
        }
    }
}