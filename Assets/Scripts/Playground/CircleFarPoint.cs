using UnityEngine;
using System.Collections;

using Telescopes;

namespace Telescopes.Playground
{
    public class CircleFarPoint : MonoBehaviour
    {
        public Circle2D circle;
        public Transform point;

        // Use this for initialization
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            if (circle && point)
            {
                Vector3 farPoint = TelescopeUtils.FarthestPointOnCircle(circle.Center,
                    circle.Normal, circle.radius, point.position);
                transform.position = farPoint;
            }
        }
    }

}