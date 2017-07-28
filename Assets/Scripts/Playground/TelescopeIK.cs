using UnityEngine;
using System.Collections.Generic;

using Telescopes;

namespace Telescopes.Playground
{
    public class TelescopeIK : MonoBehaviour
    {
        public Transform target;
        TelescopeSegment segment;

        public float maxDegPerSecond = 20f;

        public bool FreezeFirst = false;

        // Use this for initialization
        void Start()
        {
            segment = GetComponent<TelescopeSegment>();
        }

        void AddGradientToTelescope(List<float> angles)
        {
            // Apply first telescope separately
            float firstAngle = angles[0];
            if (FreezeFirst) firstAngle = 0;

            // Since the first telescope is free, we just apply rotation about the tangent
            Vector3 firstTangent = segment.shells[0].transform.forward;
            Quaternion firstRotation = Quaternion.AngleAxis(firstAngle, firstTangent);
            segment.shells[0].transform.rotation = firstRotation * segment.shells[0].transform.rotation;

            // For the rest, just add to the twist angle
            for (int i = 1; i < angles.Count; i++)
            {
                TelescopeShell s = segment.shells[i];
                s.twistAngle = Mathf.Repeat(s.twistAngle - angles[i], 360);
                if (s.twistAngle > 180) s.twistAngle -= 360;
            }
        }

        float GradientWRTShell(TelescopeShell shell, Vector3 targetPos)
        {
            // Changing the twist angle causes the end effector to move
            // along the tangent of the circle centered on the shell base.

            // First, compute the center of the circle.
            Vector3 shellBase = shell.StartPointWS;
            Vector3 circleNormal = shell.StartTangentWS.normalized;

            Vector3 currentEndEff = segment.LastShell.EndPointWS;

            // The circle normal remains the same, but is shifted so that
            // the current end effector is in the plane.
            Vector3 fromEnd = shellBase - currentEndEff;
            Vector3 nonPlanar = Vector3.Dot(fromEnd, circleNormal) * circleNormal;
            Vector3 circleCenter = currentEndEff + (fromEnd - nonPlanar);

            // The gradient is then the cross product of the current end position
            // and the desired end position. This gives the axis of rotation
            // required to move it there.
            Vector3 centerToCurrent = currentEndEff - circleCenter;
            Vector3 currentToTarget = targetPos - currentEndEff;

            Vector3 axis = Vector3.Cross(centerToCurrent, currentToTarget);

            // We can only rotate about the fixed shell axis, however,
            // so the dot product gives the contribution from this axis.
            return Vector3.Dot(circleNormal, axis);
        }

        List<float> GradientToTargetPosition(Vector3 targetPos)
        {
            List<float> gradient = new List<float>();
            
            foreach (var shell in segment.shells)
            {
                gradient.Add(GradientWRTShell(shell, targetPos));
            }

            return gradient;
        }

        // Update is called once per frame
        void Update()
        {
            if (target)
            {
                List<float> gradient = GradientToTargetPosition(target.transform.position);

                for (int i = 0; i < gradient.Count; i++)
                {
                    gradient[i] = maxDegPerSecond * Time.deltaTime * gradient[i];
                }

                AddGradientToTelescope(gradient);
            }
        }
    }

}